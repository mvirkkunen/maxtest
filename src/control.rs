#![allow(unused)]

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum UsbDirection {
    /// Host to device (OUT)
    Out = 0x00,
    /// Device to host (IN)
    In = 0x80,
}

/// Control request type.
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum RequestType {
    /// Request is a USB standard request. Usually handled by
    /// [`UsbDevice`](crate::device::UsbDevice).
    Standard = 0,
    /// Request is intended for a USB class.
    Class = 1,
    /// Request is vendor-specific.
    Vendor = 2,
    /// Reserved.
    Reserved = 3,
}

/// Control request recipient.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Recipient {
    /// Request is intended for the entire device.
    Device = 0,
    /// Request is intended for an interface. Generally, the `index` field of the request specifies
    /// the interface number.
    Interface = 1,
    /// Request is intended for an endpoint. Generally, the `index` field of the request specifies
    /// the endpoint address.
    Endpoint = 2,
    /// None of the above.
    Other = 3,
    /// Reserved.
    Reserved = 4,
}

/// A control request read from a SETUP packet.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Request {
    /// Direction of the request.
    pub direction: UsbDirection,
    /// Type of the request.
    pub request_type: RequestType,
    /// Recipient of the request.
    pub recipient: Recipient,
    /// Request code. The meaning of the value depends on the previous fields.
    pub request: u8,
    /// Request value. The meaning of the value depends on the previous fields.
    pub value: u16,
    /// Request index. The meaning of the value depends on the previous fields.
    pub index: u16,
    /// Length of the DATA stage. For control OUT transfers this is the exact length of the data the
    /// host sent. For control IN transfers this is the maximum length of data the device should
    /// return.
    pub length: u16,
}

impl Request {
    /// Standard USB control request Get Status
    pub const GET_STATUS: u8 = 0;

    /// Standard USB control request Clear Feature
    pub const CLEAR_FEATURE: u8 = 1;

    /// Standard USB control request Set Feature
    pub const SET_FEATURE: u8 = 3;

    /// Standard USB control request Set Address
    pub const SET_ADDRESS: u8 = 5;

    /// Standard USB control request Get Descriptor
    pub const GET_DESCRIPTOR: u8 = 6;

    /// Standard USB control request Set Descriptor
    pub const SET_DESCRIPTOR: u8 = 7;

    /// Standard USB control request Get Configuration
    pub const GET_CONFIGURATION: u8 = 8;

    /// Standard USB control request Set Configuration
    pub const SET_CONFIGURATION: u8 = 9;

    /// Standard USB control request Get Interface
    pub const GET_INTERFACE: u8 = 10;

    /// Standard USB control request Set Interface
    pub const SET_INTERFACE: u8 = 11;

    /// Standard USB control request Synch Frame
    pub const SYNCH_FRAME: u8 = 12;

    /// Standard USB feature Endpoint Halt for Set/Clear Feature
    pub const FEATURE_ENDPOINT_HALT: u16 = 0;

    /// Standard USB feature Device Remote Wakeup for Set/Clear Feature
    pub const FEATURE_DEVICE_REMOTE_WAKEUP: u16 = 1;

    /*pub(crate) fn parse(buf: &[u8]) -> Result<Request> {
        if buf.len() != 8 {
            return Err(UsbError::ParseError);
        }

        let rt = buf[0];
        let recipient = rt & 0b11111;

        Ok(Request {
            direction: rt.into(),
            request_type: unsafe { mem::transmute((rt >> 5) & 0b11) },
            recipient:
                if recipient <= 3 { unsafe { mem::transmute(recipient) } }
                else { Recipient::Reserved },
            request: buf[1],
            value: (buf[2] as u16) | ((buf[3] as u16) << 8),
            index: (buf[4] as u16) | ((buf[5] as u16) << 8),
            length: (buf[6] as u16) | ((buf[7] as u16) << 8),
        })
    }*/

    pub fn serialize(&self, buf: &mut [u8]) {
        buf[0] = (self.direction as u8)
            | ((self.request_type as u8) << 5)
            | (self.recipient as u8);
        buf[1] = self.request;
        buf[2] = self.value as u8;
        buf[3] = (self.value >> 8) as u8;
        buf[4] = self.index as u8;
        buf[5] = (self.index >> 8) as u8;
        buf[6] = self.length as u8;
        buf[7] = (self.length >> 8) as u8;
    }

    /// Gets the descriptor type and index from the value field of a GET_DESCRIPTOR request.
    pub fn descriptor_type_index(&self) -> (u8, u8) {
        ((self.value >> 8) as u8, self.value as u8)
    }
}

/// Standard descriptor types
#[allow(missing_docs)]
pub mod descriptor_type {
    pub const DEVICE: u8 = 1;
    pub const CONFIGURATION: u8 = 2;
    pub const STRING: u8 = 3;
    pub const INTERFACE: u8 = 4;
    pub const ENDPOINT: u8 = 5;
    pub const BOS: u8 = 15;
    pub const CAPABILITY: u8 = 16;
}

/// String descriptor language IDs.
pub mod lang_id {
    /// English (US)
    ///
    /// Recommended for use as the first language ID for compatibility.
    pub const ENGLISH_US: u16 = 0x0409;
}