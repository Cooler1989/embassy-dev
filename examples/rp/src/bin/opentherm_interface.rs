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

pub const CAPTURE_OT_FRAME_PAYLOAD_SIZE: usize = 32_usize;
pub const MESSAGE_DATA_VALUE_BIT_LEN: usize = 16_usize;
pub const MESSAGE_DATA_ID_BIT_LEN: usize = 8_usize;
pub const MESSAGE_TYPE_BIT_LEN: usize = 3_usize;
pub const OT_FRAME_SKIP_SPARE: usize = 4_usize;

#[derive(Debug)]
pub enum Error {
    GenericError,
    InvalidData,
    UnknownDataId,
    BusError,
    InvalidStart,
    InvalidLength,
    DecodingError,
    ParityError,
    CommandNotSupported,
    UnexpectedResult,
}

#[repr(u8)]
#[derive(Clone, Copy)]
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

#[derive(Copy, Clone)]
pub enum CHState {
    Enable(bool),
}

impl Default for CHState {
    fn default() -> Self {
        CHState::Enable(false)
    }
}

#[derive(Copy, Clone)]
pub enum Fault {
    Value(bool),
}

impl Default for Fault {
    fn default() -> Self {
        Fault::Value(false)
    }
}

#[derive(Copy, Clone)]
pub enum DWHState {
    Enable(bool),
}

impl Default for DWHState {
    fn default() -> Self {
        DWHState::Enable(false)
    }
}

#[derive(Copy, Clone)]
pub enum FlameState {
    Active(bool)
}

impl Default for FlameState {
    fn default() -> Self {
        FlameState::Active(false)
    }
}

enum Flag8 {
    value(u8),
}

impl From<MasterStatus> for Flag8 {
    fn from(item: MasterStatus) -> Self {
        let mut result: u8 = 0_u8;
        //  let ChEnable(item.ch_state) {
        match item.ch_enable {
            CHState::Enable(boolean) => {
                result |= (boolean as u8) << 0x0;
            }
        }
        Flag8::value(result)
    }
}

//  Data types:
//  f8.8  signed fixed point value : 1 sign bit, 7 integer bit, 8 fractional bits (two’s compliment ie. the
//  LSB of the 16bit binary number represents 1/256th of a unit).
enum Float8_8 {
    Signed(i8),
    //  TODO:
    //  fractional(u8),
}

impl From<Float8_8> for Temperature {
    fn from(item: Float8_8) -> Self {
        todo!();
    }
}

// implement From / to flag8
#[derive(Default)]
pub struct MasterStatus {
    pub ch_enable: CHState,
    pub dwh_enable: DWHState,
}

impl MasterStatus {
    fn new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
        let mut iterator = iterator;
        let ch_state = iterator.next().unwrap();
        let dwh_state = iterator.next().unwrap();
        Ok(Self {
            ch_enable: CHState::Enable(*ch_state),
            dwh_enable: DWHState::Enable(*dwh_state),
        })
    }
    pub fn new(ch: CHState, dwh: DWHState) -> Self {
        Self {
            ch_enable: ch,
            dwh_enable: dwh,
        }
    }
}

#[derive(Default)]
pub struct SlaveStatus {
    fault: Fault,
    ch_active: CHState,
    dwh_active: DWHState,
    flame_active: FlameState,
}

impl SlaveStatus {
    fn new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
        let mut iterator = iterator;
        let fault_indication = iterator.next().unwrap();
        let ch_state = iterator.next().unwrap();
        let dwh_state = iterator.next().unwrap();
        let flame_active = iterator.next().unwrap();
        Ok(Self {
            fault: Fault::Value(*fault_indication),
            ch_active: CHState::Enable(*ch_state),
            dwh_active: DWHState::Enable(*dwh_state),
            flame_active: FlameState::Active(*flame_active),
        })
    }
    pub fn new(ch_state: CHState, dwh_state: DWHState, flame: FlameState) -> Self {
        Self {
            fault: Fault::Value(false),
            ch_active: ch_state,
            dwh_active: dwh_state,
            flame_active: flame,
        }
    }
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    ControlSetpoint = 0x01,
    BoilerTemperature = 0x25,
    Unrecognized = 0xFF,
}

pub enum DataOt {
    MasterStatus(MasterStatus), //  for write
    SlaveStatus(SlaveStatus),   //  for read
    ControlSetpoint(Temperature),
    BoilerTemperature(Temperature),
}

impl TryFrom<u8> for OpenThermMessageCode {
    type Error = ();
    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            0x00 => Ok(OpenThermMessageCode::Status),
            0x01 => Ok(OpenThermMessageCode::ControlSetpoint),
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
    data_id: DataOt,
}

//  TODO: Implement iterator: https://dev.to/wrongbyte/implementing-iterator-and-intoiterator-in-rust-3nio
//?
enum OpenThermIteratorState {
    StartBit,
    MessageType3b(u8),
    Spare4b(u8),
    DataId8b(u8),
    DataValue16b(u8),
    StopBit,
    Depleted,
}

pub struct OpenThermMessageIterator<'a> {
    message: &'a OpenThermMessage,
    state: OpenThermIteratorState,
}

impl<'a> Iterator for OpenThermMessageIterator<'a> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let (new_state, return_value) = match self.state {
            OpenThermIteratorState::StartBit => (OpenThermIteratorState::MessageType3b(0), Some(true)),
            OpenThermIteratorState::MessageType3b(shift) => {
                let value = 0_u8 != (self.message.msg_type as u8) & (0x1_u8 << shift);
                if shift >= MESSAGE_TYPE_BIT_LEN as u8 - 1 {
                    (OpenThermIteratorState::Spare4b(0), Some(value))
                } else {
                    (OpenThermIteratorState::MessageType3b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::Spare4b(shift) => {
                if shift >= OT_FRAME_SKIP_SPARE as u8 - 1 {
                    (OpenThermIteratorState::StopBit, Some(false))
                } else {
                    (OpenThermIteratorState::Spare4b(shift + 1), Some(false))
                }
            }
            OpenThermIteratorState::DataId8b(shift) => {
                let value = 0_u8 != (self.message.cmd as u8) & (0x1_u8 << shift);
                if shift >= MESSAGE_DATA_ID_BIT_LEN as u8 - 1 {
                    (OpenThermIteratorState::DataValue16b(0), Some(value))
                } else {
                    (OpenThermIteratorState::DataId8b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::DataValue16b(shift) => {
                let value = 0_u16 != (self.message.data_value as u16) & (0x1_u16 << shift);
                if shift >= MESSAGE_DATA_VALUE_BIT_LEN as u8 - 1 {
                    (OpenThermIteratorState::StopBit, Some(false))
                } else {
                    (OpenThermIteratorState::DataValue16b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::StopBit => (OpenThermIteratorState::Depleted, Some(true)),
            _ => (OpenThermIteratorState::Depleted, None),
        };
        self.state = new_state;
        return_value
    }
}

impl OpenThermMessage {
    pub fn iter(&self) -> OpenThermMessageIterator {
        OpenThermMessageIterator {
            message: self,
            state: OpenThermIteratorState::StartBit,
        }
    }
    fn float8_8_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Float8_8, Error> {
        let sign = iterator.clone().next().unwrap();
        let data_value = iterator.skip(1).enumerate().fold(0i8, |acc, (i, &bit_state)| {
            let value = acc | ((bit_state as i8) << i);
            value
        });
        //  return:
        match *sign {
            true => Ok(Float8_8::Signed(-1i8 * data_value)),
            false => Ok(Float8_8::Signed(data_value)),
        }
    }

    pub fn new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
        let folded =
            iterator
                .clone()
                .take(CAPTURE_OT_FRAME_PAYLOAD_SIZE)
                .enumerate()
                .fold(0_u32, |acc, (i, &bit_state)| {
                    let value = acc | ((bit_state as u32) << i);
                    value
                });
        log::info!("Folded OR: 0x{:x}", folded);
        //  Check parity for whole OT Frame
        if folded.count_ones() % 2 == 1 {
            log::error!("OT Frame Parity Error: 0x{:x}", folded);
            return Err(Error::ParityError);
        }

        let data_value_iterator = iterator.clone().take(MESSAGE_DATA_VALUE_BIT_LEN);
        let data_float8_8 = Self::float8_8_from_iter(data_value_iterator.clone()).unwrap();
        let data_value = data_value_iterator
            .clone()
            .enumerate()
            .fold(0u16, |acc, (i, &bit_state)| {
                let value = acc | ((bit_state as u16) << i);
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
        let iter = iter.skip(MESSAGE_DATA_ID_BIT_LEN + OT_FRAME_SKIP_SPARE);

        let msg_type = iter
            .clone()
            .take(MESSAGE_TYPE_BIT_LEN)
            .enumerate()
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

        //  Deconding based on other types
        let data_id = match data_id {
            OpenThermMessageCode::Status => match msg_type {
                MessageType::ReadAck => {
                    todo!();
                    DataOt::SlaveStatus(SlaveStatus::new_from_iter(data_value_iterator)?)
                }
                MessageType::ReadData => {
                    todo!();
                    DataOt::MasterStatus(Default::default())
                }
                _ => {
                    return Err(Error::DecodingError);
                }
            },
            //  fn float8_8_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Float8_8, Error> {
            OpenThermMessageCode::ControlSetpoint => DataOt::ControlSetpoint(data_float8_8.into()), // f8.8 TODO: use u3
            // to map float
            OpenThermMessageCode::BoilerTemperature => DataOt::BoilerTemperature(data_float8_8.into()),
            _ => {
                log::error!("Unrecognized DataId");
                return Err(Error::UnknownDataId);
            }
        };

        Ok(Self {
            msg_type: msg_type,
            data_id: DataOt::MasterStatus(Default::default()),
            data_value: 0x00,
            cmd: OpenThermMessageCode::Status,
        })
    }

    pub fn new_from_data_ot(cmd_type: MessageType, data_ot: DataOt) -> Result<Self, Error> {
        Ok(Self {
            msg_type: cmd_type,
            data_id: data_ot,
            data_value: 0x00,
            cmd: OpenThermMessageCode::Status,
        })
    }
}

pub trait OpenThermInterface {
    async fn write(&mut self, cmd: DataOt) -> Result<(), Error>;
    //  read need more parameters: timeout, number of expected bits etc
    async fn read(&mut self, cmd: DataOt) -> Result<DataOt, Error>;
    async fn listen(&mut self) -> Result<DataOt, Error>;
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

impl Default for Temperature {
    fn default() -> Self {
        Temperature::Celsius(0_i16)
    }
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
