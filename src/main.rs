#![no_std]
#![no_main]

use core::fmt::Write;
use panic_rtt as _;
//use nb::block;
use stm32f1xx_hal::{
    prelude::*,
    pac,
    delay::Delay,
    spi::Spi,
};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v1_compat::OldOutputPin;
use cortex_m_rt::entry;
use cortex_m::asm::delay;

use hd44780_driver::*;
use hd44780_driver::entry_mode::*;
use heapless::String;

mod max;
use max::*;
mod control;
use control::*;

#[macro_export]
macro_rules! println {
    () => {
        core:::fmt::Write::write_str(&mut jlink_rtt::NonBlockingOutput::new(), "\n").ok();
    };
    ($fmt:expr) => {
        core::fmt::Write::write_str(
            &mut jlink_rtt::NonBlockingOutput::new(),
            concat!($fmt, "\n")).ok();
    };
    ($fmt:expr, $($arg:tt)*) => {
        core::fmt::Write::write_fmt(
            &mut jlink_rtt::NonBlockingOutput::new(),
            format_args!(concat!($fmt, "\n"), $($arg)*)).ok();
    };
}


#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    //let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let ldelay = Delay::new(cp.SYST, clocks);

    let mut lcd = HD44780::new_4bit(
        OldOutputPin::new(gpioa.pa8.into_push_pull_output(&mut gpioa.crh)),
        OldOutputPin::new(gpioa.pa9.into_push_pull_output(&mut gpioa.crh)),

        OldOutputPin::new(gpioa.pa10.into_push_pull_output(&mut gpioa.crh)),
        OldOutputPin::new(gpioa.pa11.into_push_pull_output(&mut gpioa.crh)),
        OldOutputPin::new(gpioa.pa12.into_push_pull_output(&mut gpioa.crh)),
        OldOutputPin::new(gpioa.pa0.into_push_pull_output(&mut gpioa.crl)),

        ldelay,
    );

    lcd.reset();

    lcd.clear();

    let spi_sck_pin = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let spi_miso_pin = gpioa.pa6;
    let spi_mosi_pin = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    /*let spi_sck_pin = gpiob.pb3.into_alternate_push_pull(&mut gpiob.crl);
    let spi_miso_pin = gpiob.pb4;
    let spi_mosi_pin = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);*/
    let spi = Spi::spi1(
        dp.SPI1,
        (spi_sck_pin, spi_miso_pin, spi_mosi_pin),
        &mut afio.mapr,
        embedded_hal::spi::MODE_0,
        100.khz(),
        clocks,
        &mut rcc.apb2);

    let nss = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
    let reset = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);

    let mut usb = Max::new(spi, nss, reset);

    led.set_high().ok();

    println!("Max3412E revision: {}", usb.get_revision());

    //block!(timer.wait()).unwrap();

    let mut buf = [0u8; 256];
    let mut disp = heapless::String::<heapless::consts::U32>::new();
    let mut cx = 0;
    let mut cy = 0;
    lcd.set_cursor_pos(0);

    loop {
        println!("Waiting for connection");

        let speed = usb.wait_for_connection();

        println!("Connected {:?} speed device", speed);

        //usb.reset_device();

        //println!("Reset");

        delay(100 * 8000);

        //let mut ctrl_in = Endpoint::new(0x80);

        let nread = usb.control_in(0, Some(&mut buf[..8]), Request {
            direction: UsbDirection::In,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::GET_DESCRIPTOR,
            value: (descriptor_type::DEVICE as u16) << 8,
            index: 0,
            length: 8,
        }).unwrap();

        println!("device descriptor: {:x?}", &buf[..nread]);

        usb.set_max_packet_size_0(buf[7]);

        let nread = usb.control_in(0, Some(&mut buf), Request {
            direction: UsbDirection::In,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::GET_DESCRIPTOR,
            value: (descriptor_type::CONFIGURATION as u16) << 8,
            index: 0,
            length: 256,
        }).unwrap();

        println!("configuration descriptor: {:x?}", &buf[..nread]);

        let configuration_value = buf[5];

        usb.control_out(0, None, Request {
            direction: UsbDirection::Out,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::SET_ADDRESS,
            value: 1,
            index: 0,
            length: 0,
        }).unwrap();

        println!("set address to 1");

        usb.control_out(1, None, Request {
            direction: UsbDirection::Out,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Request::SET_CONFIGURATION,
            value: configuration_value as u16,
            index: 0,
            length: 0,
        }).unwrap();

        println!("set configuration to {}", configuration_value);

        let mut ep = Endpoint::new(0x81);
        let mut prev = [0u8; 8];

        loop {
            usb.wait_start_of_frame();

            let mut packet = [0u8; 8];
            usb.interrupt_in(1, &mut ep, &mut packet).unwrap();

            let bits = packet[0];
            packet[0] = 0;

            for &key in packet[1..].iter() {
                if !prev.iter().any(|&k| k == key) {
                    println!("{} {:02x}", bits, key);
                    if let Some(ch) = translate_key(bits, key) {
                        //println!("{}", ch);
                        //disp.push(ch).ok();
                        lcd.write_char(ch);

                        cx += 1;
                        if cx == 16 {
                            cx = 0;
                            cy = 1 - cy;
                            lcd.set_cursor_pos(cy * 40 + cx);
                        }
                    } else if key == 0x2a {
                        /*cx -= 1;

                        if cx == 16 {
                            cx = 0;
                            cy = 1 - cy;
                            lcd.set_cursor_pos(cy * 40 + cx);
                        }*/

                        lcd.set_cursor_mode(CursorMode::Decrement);
                        lcd.write_char(' ');
                        lcd.write_char(' ');
                        lcd.set_cursor_mode(CursorMode::Increment);

                        if cx == 0 {
                            cx = 15;
                            cy = 1 - cy;
                            lcd.set_cursor_pos(cy * 40 + cx);
                        } else {
                            cx -= 1;
                        }
                    }
                }
            }

            prev = packet;

            //println!("{:x?}", &buf[..8]);
        }

        //i += 1;

        //writeln!(rtt, "Hello {}", i).ok();
        /*if i % 10 == 0 {
            write!(rtt, "!\n");
        } else {
            write!(rtt, ".");
        }*/
    }
}

fn translate_key(bits: u8, key: u8) -> Option<char> {
    let shift = (bits & 0x22) != 0;

    for &(k, nc, sc) in KEYCODES.iter() {
        if k == key {
            return Some(if shift { sc } else { nc });
        }
    }

    None
}

const KEYCODES: &[(u8, char, char)] = &[
    (0x04, 'a', 'A'),
    (0x05, 'b', 'B'),
    (0x06, 'c', 'C'),
    (0x07, 'd', 'D'),
    (0x08, 'e', 'E'),
    (0x09, 'f', 'F'),
    (0x0a, 'g', 'G'),
    (0x0b, 'h', 'H'),
    (0x0c, 'i', 'I'),
    (0x0d, 'j', 'J'),
    (0x0e, 'k', 'K'),
    (0x0f, 'l', 'L'),
    (0x10, 'm', 'M'),
    (0x11, 'n', 'N'),
    (0x12, 'o', 'O'),
    (0x13, 'p', 'P'),
    (0x14, 'q', 'Q'),
    (0x15, 'r', 'R'),
    (0x16, 's', 'S'),
    (0x17, 't', 'T'),
    (0x18, 'u', 'U'),
    (0x19, 'v', 'V'),
    (0x1a, 'w', 'W'),
    (0x1b, 'x', 'X'),
    (0x1c, 'y', 'Y'),
    (0x1d, 'z', 'Z'),
    (0x1e, '1', '!'),
    (0x1f, '2', '"'),
    (0x20, '3', '#'),
    (0x21, '4', '¤'),
    (0x22, '5', '%'),
    (0x23, '6', '&'),
    (0x24, '7', '/'),
    (0x25, '8', '('),
    (0x26, '9', ')'),
    (0x27, '0', '='),
    (0x2c, ' ', ' '),
    (0x2d, '+', '?'),
];