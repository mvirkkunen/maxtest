use core::cmp::min;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use cortex_m::asm::delay;

use crate::println;
use crate::control::*;

pub struct Max<SPI, SS, RES> {
    spi: SPI,
    ss: SS,
    res: RES,
    peraddr: u8,
    sndtog: bool,
    rcvtog: bool,
    max_packet_size_0: u8,
}

pub struct Endpoint {
    addr: u8,
    tog: bool,
}

impl Endpoint {
    pub fn new(addr: u8) -> Self {
        Self {
            addr,
            tog: false,
        }
    }
}

impl<SPI, SS, RES> Max<SPI, SS, RES>
where
    SPI: Write<u8> + Transfer<u8>,
    <SPI as Transfer<u8>>::Error: core::fmt::Debug,
    <SPI as Write<u8>>::Error: core::fmt::Debug,
    SS: OutputPin,
    RES: OutputPin,
{
    pub fn new(spi: SPI, ss: SS, res: RES) -> Self {
        let mut max = Self {
            spi,
            ss,
            res,
            peraddr: 0xff,
            sndtog: false,
            rcvtog: false,
            max_packet_size_0: 8,
        };

        max.reset();
        max.config_host();

        max
    }

    pub fn get_revision(&mut self) -> u8 {
        self.reg_read(Reg::Revision)
    }

    pub fn wait_for_connection(&mut self) -> Speed {
        self.reg_clear(Reg::Mode, 0b00000010); // LOWSPEED
        self.reg_write(Reg::Hirq, Hirq::Condet as u8);

        let speed = loop {
            //self.wait_hirq(Hirq::Condet);
            self.reg_write(Reg::Hctl, 0b00000100); // SAMPLEBUS

            let hrsl = self.reg_read(Reg::Hrsl);

            match (hrsl >> 6) & 0b11 {
                0b10 => break Speed::High,
                0b01 => break Speed::Low,
                _ => continue,
            }
        };

        if speed == Speed::Low {
            self.reg_set(Reg::Mode, 0b00000010); // LOWSPEED
        }

        self.reg_set(Reg::Hctl, 0b00000001); // BUSRST
        self.wait_hirq(Hirq::Busevent);
        self.reg_set(Reg::Mode, 0b00001000); // SOFKAENAB
        self.wait_start_of_frame();

        self.rcvtog = false;
        self.sndtog = false;
        self.peraddr = 0xff;
        self.reg_write(Reg::Hctl, 0b01010000); // SNDTOG0, RCVTOG0

        speed
    }

    pub fn wait_start_of_frame(&mut self) {
        self.wait_hirq(Hirq::Frame);
    }

    pub fn set_max_packet_size_0(&mut self, max_packet_size_0: u8) {
        self.max_packet_size_0 = max_packet_size_0;
    }

    pub fn control_in(&mut self, addr: u8, data: Option<&mut [u8]>, mut req: Request) -> Result<usize, Hrsl> {
        req.direction = UsbDirection::In;
        self.send_setup(addr, &mut req)?;

        let mut total = 0;

        if let Some(data) = data {
            self.reg_write(Reg::Hctl, 0b00100000); // RCVTOG1

            while total < data.len() {
                self.do_xfr_wait(0x00, Hxfr::BulkIn)?;

                let count = min(self.max_packet_size_0 as usize, data.len() - total);
                let nread = self.fifo_read(&mut data[total..total + count]);

                total += nread;

                if nread < self.max_packet_size_0 as usize {
                    break;
                }
            }
        }

        self.do_xfr_wait(0x80, Hxfr::HsOut)?;

        Ok(total)
    }

    pub fn control_out(&mut self, addr: u8, data: Option<&[u8]>, mut req: Request) -> Result<(), Hrsl> {
        req.direction = UsbDirection::Out;
        self.send_setup(addr, &mut req)?;

        if let Some(data) = data {
            self.reg_write(Reg::Hctl, 0b10000000); // SNDTOG1

            let mut total = 0;

            while total < data.len() {
                let count = min(self.max_packet_size_0 as usize, data.len() - total);

                self.fifo_write(Outfifo::Snd, &data[total..total + count]);

                self.do_xfr_wait(0x00, Hxfr::BulkOut)?;

                total += count;
            }
        }

        self.do_xfr_wait(0x00, Hxfr::HsIn)?;

        Ok(())
    }

    pub fn interrupt_in(&mut self, addr: u8, ep: &mut Endpoint, data: &mut [u8]) -> Result<usize, Hrsl> {
        self.set_peraddr(addr);

        if ep.tog != self.rcvtog {
            self.reg_write(Reg::Hctl, if ep.tog { 0b00100000 } else { 0b00010000 });
        }
        self.do_xfr_wait(ep.addr, Hxfr::BulkIn)?;
        ep.tog = self.rcvtog;

        Ok(self.fifo_read(data))
    }

    fn send_setup(&mut self, addr: u8, req: &Request) -> Result<(), Hrsl> {
        let mut setup = [0u8; 8];
        req.serialize(&mut setup);

        self.set_peraddr(addr);

        self.fifo_write(Outfifo::Sud, &setup);

        self.do_xfr_wait(0x00, Hxfr::Setup)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.res.set_high().ok();

        // FDUPSPI
        self.reg_write(Reg::Pinctl, 0b00010000);

        self.reg_write(Reg::Usbctl, 0b00100000); // CHIPRES
        self.reg_write(Reg::Usbctl, 0); // Clear reset

        self.wait_usbirq(Usbirq::Oscok);
    }

    fn config_host(&mut self) {
        // DPPULLDN, DMPULLDN, HOST
        self.reg_write(
            Reg::Mode,
            0b11000001);

        // Turn on host power (kinda circuit dependent but this is a test so)
        self.reg_set(Reg::Iopins1, 0b00000001);
    }

    fn wait_hirq(&mut self, irq: Hirq) {
        loop {
            let hirq = self.reg_read(Reg::Hirq);
            //writeln!(NonBlockingOutput::new(), "{:08b} {:08b}", hirq, self.reg_read(Reg::Hien)).ok();
            if hirq & (irq as u8) != 0 {
                self.reg_write(Reg::Hirq, irq as u8);
                break;
            }
        }
    }

    fn wait_usbirq(&mut self, irq: Usbirq) {
        loop {
            let usbirq = self.reg_read(Reg::Usbirq);
            //writeln!(NonBlockingOutput::new(), "{:08b} {:08b}", hirq, self.reg_read(Reg::Hien)).ok();
            if usbirq & (irq as u8) != 0 {
                self.reg_write(Reg::Usbirq, irq as u8);
                break;
            }
        }
    }

    fn do_xfr(&mut self, ep: u8, xfr: Hxfr) -> Hrsl {
        self.reg_write(Reg::Hxfr, (xfr as u8) << 4 | (ep & 0x0f));

        self.wait_hirq(Hirq::Hxfrdn);

        let hrsl = self.reg_read(Reg::Hrsl);

        self.sndtog = (hrsl & 0b00100000) != 0;
        self.rcvtog = (hrsl & 0b00010000) != 0;

        return unsafe { core::mem::transmute(hrsl & 0x0f) };
    }

    fn do_xfr_wait(&mut self, ep: u8, xfr: Hxfr) -> Result<(), Hrsl> {
        loop {
            match self.do_xfr(ep, xfr) {
                Hrsl::Success => break,
                Hrsl::Nak => continue,
                err => return Err(err),
            }
        }

        Ok(())
    }

    fn set_peraddr(&mut self, addr: u8) {
        if addr != self.peraddr {
            self.reg_write(Reg::Peraddr, addr);
            self.peraddr = addr;
        }
    }

    fn fifo_write(&mut self, fifo: Outfifo, data: &[u8]) {
        self.ss.set_low().ok();
        self.spi.write(&[fifo as u8]).unwrap();
        self.spi.write(&data).unwrap();
        self.ss.set_high().ok();

        if fifo == Outfifo::Snd {
            self.reg_write(Reg::Sndbc, data.len() as u8);
        }
    }

    // There is only one read fifo
    fn fifo_read(&mut self, data: &mut [u8]) -> usize {
        if self.reg_read(Reg::Hirq) & Hirq::Rcvdav as u8 == 0 {
            return 0;
        }

        let count = min(data.len(), self.reg_read(Reg::Rvcbc) as usize);

        self.ss.set_low().ok();
        self.spi.write(&[1 << 3]).unwrap();
        self.spi.transfer(&mut data[..count]).unwrap();
        self.ss.set_high().ok();

        self.reg_write(Reg::Hirq, Hirq::Rcvdav as u8);

        count
    }

    fn reg_write(&mut self, reg: Reg, value: u8) {
        let mut buf = [((reg as u8) << 3) | 0x02, value];

        self.ss.set_low().ok();
        self.spi.write(&mut buf).unwrap();
        self.ss.set_high().ok();
    }

    fn reg_read(&mut self, reg: Reg) -> u8 {
        let mut buf = [((reg as u8) << 3), 0x00];

        self.ss.set_low().ok();
        self.spi.transfer(&mut buf).unwrap();
        self.ss.set_high().ok();

        buf[1]
    }

    fn reg_set(&mut self, reg: Reg, bits: u8) {
        let v = self.reg_read(reg);
        self.reg_write(reg, v | bits);
    }

    fn reg_clear(&mut self, reg: Reg, bits: u8) {
        let v = self.reg_read(reg);
        self.reg_write(reg, v & !bits);
    }
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Reg {
    Rvcbc = 6,
    Sndbc = 7,
    Usbirq = 13,
    Usbien = 14,
    Usbctl = 15,
    Cpuctl = 16,
    Pinctl = 17,
    Revision = 18,
    Iopins1 = 20,
    Iopins2 = 21,
    Gpinirq = 22,
    Gpinien = 23,
    Gpinpol = 24,
    Hirq = 25,
    Hien = 26,
    Mode = 27,
    Peraddr = 28,
    Hctl = 29,
    Hxfr = 30,
    Hrsl = 31,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hirq {
    Busevent = 0x01,
    Rwu = 0x02,
    Rcvdav = 0x04,
    Sndbav = 0x08,
    Susdn = 0x10,
    Condet = 0x20,
    Frame = 0x40,
    Hxfrdn = 0x80,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Usbirq {
    Oscok = 0b01,
    Novbus = 0x20,
    Vbus = 0x40,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hrsl {
    Success = 0x00,
    Busy = 0x01,
    Badreq = 0x02,
    Undef = 0x03,
    Nak = 0x04,
    Stall = 0x05,
    Togerr = 0x06,
    Wrongpid = 0x07,
    Badbc = 0x08,
    Piderr = 0x09,
    Pkterr = 0x0a,
    Crcerr = 0x0b,
    Kerr = 0x0c,
    Jerr = 0x0d,
    Timeout = 0x0e,
    Babble = 0x0f,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hxfr {
    Setup = 0b0001,
    BulkIn = 0b0000,
    BulkOut = 0b0010,
    HsIn = 0b1000,
    HsOut = 0b1010,
    IsoIn = 0b0100,
    IsoOut = 0b0110,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Outfifo {
    Snd = (2 << 3) | 0x02,
    Sud = (4 << 3) | 0x02,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Speed {
    High,
    Low,
}

