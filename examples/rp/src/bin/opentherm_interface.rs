#![no_std]

use core::ops::Add;

//  use bytes::Buf;

//  use bit_set::BitSet;
//  use bitset_core::BitSet;
// use bitvec::prelude::*;

//  use byteorder::{ByteOrder, LittleEndian};

//  TODO:
//  - Still: detect burner start transition based on mapped OpenTherm responses

pub enum Error {
    GenericError,
    InvalidData,
    UnknownDataId,
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

//  impl From<MessageType> for BitSet
//  {
//      fn from (item: MessageType) -> Self
//      {
//          match item {
//              MessageType::ReadData => BitSet::from_bytes(&[0b000]),
//              MessageType::WriteData => BitSet::from_bytes(&[0b001]),
//              MessageType::InvalidData => BitSet::from_bytes(&[0b010]),
//              MessageType::Reserved => BitSet::from_bytes(&[0b011]),
//              MessageType::ReadAck => BitSet::from_bytes(&[0b100]),
//              MessageType::WriteAck => BitSet::from_bytes(&[0b101]),
//              MessageType::DataInvalid => BitSet::from_bytes(&[0b110]),
//              MessageType::UnknownDataId => BitSet::from_bytes(&[0b111]),
//          }
//      }
//  }

//  use type-level programming as explained in hal/src/typelevel.rs
//  To annotate read/write capabilities and common features like data type it contains: u8 s8 f8.8
//  maybe information flow M->S, S->M
//  but... start simple C/C++ alike
trait OpenThermReadRequest {}
trait OpenThermWriteRequest {}

pub trait UptimeCounter {
    fn GetUptimeSec(&self) -> u32;
}

#[repr(u8)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

//  impl Enum {
//      fn discriminant(&self) -> u8 {
//      }

pub struct OpenThermMessage {
    // start bit
    // parity bit
    msg_type: MessageType,
    //  spare 4b
    data_id: u8,
    data_value: u16,
    //  stop bit
}

impl OpenThermMessage {
    pub fn new(raw_value: u32) -> Result<Self, Error> {
        Ok(Self {
            msg_type: MessageType::UnknownDataId,
            data_id: 0x00,
            data_value: 0x00,
        })
    }
}

pub trait OpenThermInterface {
    fn write(&mut self, cmd: OpenThermMessageCode, data: u32) -> Result<(), Error>;
    fn read(&self, cmd: OpenThermMessageCode) -> Result<u32, Error>;
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

impl Add for Temperature
{
    type Output = Temperature;
    fn add(self, other:Temperature) -> Temperature {
        match other {
            Temperature::Celsius(value) => {
                match self {
                    Temperature::Celsius(self_value) => {
                        Temperature::Celsius(value + self_value)
                    }
                }
            }
        }
    }
}
