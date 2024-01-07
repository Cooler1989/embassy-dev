use core::convert::From;
use core::convert::TryFrom;
use core::ops::Add;

//  use bytes::Buf;

//  use bit_set::BitSet;
//  use bitset_core::BitSet;
// use bitvec::prelude::*;

//  use byteorder::{ByteOrder, LittleEndian};

//  TODO:
//  - Still: detect burner start transition based on mapped OpenTherm responses

pub const CAPTURE_OT_FRAME_PAYLOAD_SIZE: usize = 32usize;
pub const MESSAGE_DATA_VALUE_BIT_LEN: usize = 16usize;
pub const MESSAGE_DATA_ID_BIT_LEN: usize = 8usize;
pub const MESSAGE_TYPE_BIT_LEN: usize = 3usize;
pub const OT_FRAME_SKIP_SPARE: usize = 4usize;

#[derive(Debug)]
pub enum Error {
    GenericError,
    InvalidData,
    UnknownDataId,
    InvalidStart,
    InvalidLength,
    DecodingError,
    ParityError,
}

#[repr(u8)]
pub enum MessageType {
    //  master to slave messages
    ReadData = 0x0,
    WriteData = 0x1,
    InvalidData = 0x2,
    Reserved = 0x3,
    // slave to master reponses:
    ReadAck = 0x4,
    WriteAck = 0x5,
    DataInvalid = 0x6,
    UnknownDataId = 0x7,
}

impl TryFrom<u8> for MessageType {
    type Error = ();
    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0b000 => Ok(MessageType::ReadData),
            0b001 => Ok(MessageType::WriteData),
            0b010 => Ok(MessageType::InvalidData),
            0b011 => Ok(MessageType::Reserved),
            0b100 => Ok(MessageType::ReadAck),
            0b101 => Ok(MessageType::WriteAck),
            0b110 => Ok(MessageType::DataInvalid),
            0b111 => Ok(MessageType::UnknownDataId),
            8_u8..=u8::MAX => Err(()),
        }
    }
}

//  use type-level programming as explained in hal/src/typelevel.rs
//  To annotate read/write capabilities and common features like data type it contains: u8 s8 f8.8
//  maybe information flow M->S, S->M
//  but... start simple C/C++ alike
trait OpenThermReadRequest {}
trait OpenThermWriteRequest {}

pub trait UptimeCounter {
    fn get_uptime_sec(&self) -> u32;
}

#[repr(u8)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

pub enum DataId {
    Status(u8),
    Tset(u16),
    BoilerTemperature(u16),
}

impl TryFrom<u8> for OpenThermMessageCode {
    type Error = ();
    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(OpenThermMessageCode::Status),
            0x01 => Ok(OpenThermMessageCode::TSet),
            0x25 => Ok(OpenThermMessageCode::BoilerTemperature),
            _ => Err(()),
        }
    }
}

//  impl Enum {
//      fn discriminant(&self) -> u8 {
//      }

pub struct OpenThermMessage {
    // start bit
    // parity bit
    msg_type: MessageType,
    //  spare 4b
    //  data_id: u8,
    data_value: u16,
    //  stop bit
    cmd: OpenThermMessageCode,
    data_id: DataId,
}

impl OpenThermMessage {
    pub fn new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
        let folded =
            iterator
                .clone()
                .take(CAPTURE_OT_FRAME_PAYLOAD_SIZE)
                .enumerate()
                .fold(0_u32, |acc, (i, &bit_state)| {
                    let value = acc | ((bit_state as u32) << i);
                    //  log::info!("Acc: 0x{:x}, b:{bit_state}, i: {i}", acc);
                    value
                });
        log::info!("Folded OR: 0x{:x}", folded);
        //  Check parity for whole OT Frame
        if folded.count_ones() % 2 == 1 {
            log::error!("OT Frame Parity Error: 0x{:x}", folded);
            return Err(Error::ParityError);
        }

        let data_value =
            iterator
                .clone()
                .take(MESSAGE_DATA_VALUE_BIT_LEN)
                .enumerate()
                .fold(0u16, |acc, (i, &bit_state)| {
                    let value = acc | ((bit_state as u16) << i);
                    //  log::info!("DataId decoding v[{i}] = {} => 0b{:b}", bit_state, value);
                    value
                });
        log::info!("DataValue = 0x{:x}", data_value);

        let iter = iterator.skip(MESSAGE_DATA_VALUE_BIT_LEN);
        let data_id = iter
            .clone()
            .take(MESSAGE_DATA_ID_BIT_LEN)
            .enumerate()
            .fold(0_u8, |acc, (i, &bit_state)| {
                let value = acc | ((bit_state as u8) << i);
                value
            });
        log::info!("DataId = 0x{:x}", data_id);

        let data_id: OpenThermMessageCode = match data_id.try_into() {
            Ok(msg) => msg,
            Err(_) => {
                log::error!("Unable to decode Data-Id: 0b{:b}", data_id);
                return Err(Error::DecodingError);
            }
        };

        let data_id = match data_id {
            OpenThermMessageCode::Status => DataId::Status(data_value as u8),
            OpenThermMessageCode::TSet => DataId::Tset(data_value as u16), // f8.8 TODO: use u32
            // to map float
            OpenThermMessageCode::BoilerTemperature => DataId::BoilerTemperature(data_value),
        };

        let iter = iter.skip(MESSAGE_DATA_ID_BIT_LEN + OT_FRAME_SKIP_SPARE);

        let msg_type = iter
            .clone()
            .enumerate()
            .take(MESSAGE_TYPE_BIT_LEN)
            .clone()
            .fold(0u8, |acc, (i, bit_state)| {
                let value = acc | (*bit_state as u8) << i;
                value
            });
        log::info!("MessageType raw value = 0b{:b}", msg_type);

        let msg_type: MessageType = match msg_type.try_into() {
            Ok(msg) => msg,
            Err(_) => {
                log::error!("Unable to decode MessageType: {msg_type}");
                return Err(Error::DecodingError);
            }
        };

        let mut iter = iter.skip(MESSAGE_TYPE_BIT_LEN);
        let parity = iter.next();

        Ok(Self {
            msg_type: MessageType::UnknownDataId,
            data_id: DataId::Status(0x00),
            data_value: 0x00,
            cmd: OpenThermMessageCode::Status,
        })
    }

    pub fn new(raw_value: u32) -> Result<Self, Error> {
        Ok(Self {
            msg_type: MessageType::UnknownDataId,
            data_id: DataId::Status(0x00),
            data_value: 0x00,
            cmd: OpenThermMessageCode::Status,
        })
    }
}

pub trait OpenThermInterface {
    async fn write(&mut self, cmd: OpenThermMessageCode, data: u32) -> Result<(), Error>;
    //  read need more parameters: timeout, number of expected bits etc
    async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<u32, Error>;
}

pub enum CommunicationState {
    StatusExchange,
    Control,
    Diagnostics,
}

//  possibly use 'struct Temperatore {i32}'
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub enum Temperature {
    Celsius(i16),
}

impl Add for Temperature {
    type Output = Temperature;
    fn add(self, other: Temperature) -> Temperature {
        match other {
            Temperature::Celsius(value) => match self {
                Temperature::Celsius(self_value) => Temperature::Celsius(value + self_value),
            },
        }
    }
}
