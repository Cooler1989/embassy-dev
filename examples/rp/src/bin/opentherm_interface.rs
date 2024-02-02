use core::convert::From;
use core::convert::TryFrom;
use core::ops::{Add, Sub};

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

#[derive(Clone, Copy, Debug)]
pub enum Error {
    GenericError,
    InvalidData,
    UnknownDataId,
    BusError,
    InvalidStart,
    InvalidLength,
    DecodingError,
    DataFormatError,
    ParityError,
    CommandNotSupported,
    UnexpectedResult,
    IncompatibleTypeData,
}

#[repr(u8)]
#[derive(PartialEq, Clone, Copy, Debug)]
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

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum CHState {
    Enable(bool),
}

impl Default for CHState {
    fn default() -> Self {
        CHState::Enable(false)
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Fault {
    Value(bool),
}

impl Default for Fault {
    fn default() -> Self {
        Fault::Value(false)
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum DWHState {
    Enable(bool),
}

impl Default for DWHState {
    fn default() -> Self {
        DWHState::Enable(false)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FlameState {
    Active(bool),
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
    Signed(i16),
    //  TODO:
    //  fractional(u8),
}

//  impl Float8_8 {
//      pub fn get(&self) -> i16 {
//          Self::Signed
//      }
//  }

impl TryFrom<Float8_8> for Temperature {
    type Error = ();
    fn try_from(item: Float8_8) -> Result<Self, ()> {
        if let Float8_8::Signed(v) = item {
            Ok(Temperature::Celsius(v))
        } else {
            return Err(());
        }
    }
}

impl From<Temperature> for u16 {
    fn from(item: Temperature) -> Self {
        match item {
            Temperature::Celsius(data) => data as u16,
        }
    }
    //  pub enum Temperature {
    //      Celsius(i16),
    //  }
}

// implement From / to flag8
#[derive(Copy, Clone, Default, Debug)]
pub struct MasterStatus {
    pub ch_enable: CHState,
    pub dwh_enable: DWHState,
}

const MASTER_STATUS_IGNORE_COUNT: u8 = 6;

enum MasterStatusEnumeration {
    Initial,
    Ignore(u8),
    ChEnable,
    DwhEnable,
    Depleted,
}

//  TODO: remove pub
pub struct MasterStatusIterator<'a> {
    enumeration: MasterStatusEnumeration,
    status: &'a MasterStatus,
}

impl<'a> Iterator for MasterStatusIterator<'a> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let (next, ret_value) = match self.enumeration {
            MasterStatusEnumeration::Initial => (MasterStatusEnumeration::Ignore(1), Some(false)),
            MasterStatusEnumeration::Ignore(count) => {
                let next = if count < MASTER_STATUS_IGNORE_COUNT - 1 {
                    MasterStatusEnumeration::Ignore(count + 1)
                } else {
                    MasterStatusEnumeration::DwhEnable
                };
                (next, Some(false))
            }
            MasterStatusEnumeration::DwhEnable => {
                let value = match self.status.dwh_enable {
                    DWHState::Enable(val) => val,
                };
                (MasterStatusEnumeration::ChEnable, Some(value))
            }
            MasterStatusEnumeration::ChEnable => {
                let value = match self.status.ch_enable {
                    CHState::Enable(val) => val,
                };
                (MasterStatusEnumeration::Depleted, Some(value))
            }
            MasterStatusEnumeration::Depleted => (MasterStatusEnumeration::Depleted, None),
        };
        self.enumeration = next;
        ret_value
    }
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
    pub fn iter(&self) -> MasterStatusIterator {
        MasterStatusIterator {
            enumeration: MasterStatusEnumeration::Initial,
            status: self,
        }
    }
}

#[derive(Default, Copy, Clone, Debug)]
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
    pub fn iter(&self) -> SlaveStatusIterator {
        SlaveStatusIterator {
            enumeration: SlaveStatusEnumeration::Initial,
            status: self,
        }
    }
    pub fn get_flame_active(&self) -> FlameState {
        self.flame_active
    }
}

enum SlaveStatusEnumeration {
    Initial,
    Ignore(u8),
    Fault,
    ChEnable,
    DwhEnable,
    Flame,
    Depleted,
}

const SLAVE_STATUS_IGNORE_COUNT: u8 = 4;

//  TODO: remove pub
pub struct SlaveStatusIterator<'a> {
    enumeration: SlaveStatusEnumeration,
    status: &'a SlaveStatus,
}

impl<'a> Iterator for SlaveStatusIterator<'a> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let (next, ret_value) = match self.enumeration {
            SlaveStatusEnumeration::Initial => (SlaveStatusEnumeration::Ignore(1), Some(false)),
            SlaveStatusEnumeration::Ignore(count) => {
                let next_state = if count < SLAVE_STATUS_IGNORE_COUNT - 1 {
                    SlaveStatusEnumeration::Ignore(count + 1)
                } else {
                    SlaveStatusEnumeration::Flame
                };
                (next_state, Some(false))
            }
            SlaveStatusEnumeration::Flame => match self.status.flame_active {
                FlameState::Active(val) => (SlaveStatusEnumeration::DwhEnable, Some(val)),
            },
            SlaveStatusEnumeration::DwhEnable => match self.status.dwh_active {
                DWHState::Enable(val) => (SlaveStatusEnumeration::ChEnable, Some(val)),
            },
            SlaveStatusEnumeration::ChEnable => match self.status.ch_active {
                CHState::Enable(val) => (SlaveStatusEnumeration::Fault, Some(val)),
            },
            SlaveStatusEnumeration::Fault => match self.status.fault {
                Fault::Value(val) => (SlaveStatusEnumeration::Depleted, Some(val)),
            },
            SlaveStatusEnumeration::Depleted => (SlaveStatusEnumeration::Depleted, None),
        };
        self.enumeration = next;
        ret_value
    }
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Percent {
    Value(u8),
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum OpenThermMessageCode {
    Status = 0,
    ControlSetpoint = 1,
    RelativeModulationLevel = 17,
    BoilerTemperature = 25,
    DWHTemperature = 26,
    Unrecognized = 0xFF,
}

#[derive(Copy, Clone, Debug)]
pub enum DataOt {
    MasterStatus(MasterStatus), //  for write
    SlaveStatus(SlaveStatus),   //  for read
    ControlSetpoint(Temperature),
    RelativeModulationLevel(Percent),
    BoilerTemperature(Temperature),
    DWHTemperature(Temperature),
}

impl DataOt {
    pub fn iter(&self) -> Result<DataOtIterator, Error> {
        //  OpenThermMessageIterator {
        //      message: self,
        //      state: OpenThermIteratorState::StartBit,
        //      parity_count: 0u32,
        //      data_value: 0x0,
        //  }
        let data_v = match self {
            DataOt::MasterStatus(data) => 0_u16,
            _ => {
                return Err(Error::InvalidData);
            }
        };

        //  let next_value = match self.data {
        //      DataOt::MasterStatus(status) => {
        //          //  self.message.data_id.iter();
        //      }
        //  };

        Ok(DataOtIterator {
            data: self,
            index: Some(0),
            data_value: data_v,
        })
    }
}

//  impl TryFrom<DataOt> for u16 {
//      type Error = ();
//      fn try_from(item: DataOt) -> Result<Self, Self::Error> {
//          match item {
//              DataOt::MasterStatus(status) => {
//              },
//              DataOt::BoilerTemperature(temperature) => {
//              },
//              _ => Err(()),
//          }
//      }
//  }

impl From<DataOt> for OpenThermMessageCode {
    //  type Error = ();
    fn from(item: DataOt) -> Self {
        match item {
            DataOt::MasterStatus(_) => OpenThermMessageCode::Status,
            DataOt::SlaveStatus(_) => OpenThermMessageCode::Status,
            DataOt::ControlSetpoint(_) => OpenThermMessageCode::ControlSetpoint,
            DataOt::RelativeModulationLevel(_) => OpenThermMessageCode::RelativeModulationLevel,
            DataOt::BoilerTemperature(_) => OpenThermMessageCode::BoilerTemperature,
            DataOt::DWHTemperature(_) => OpenThermMessageCode::DWHTemperature,
            // _ => Err(()),
        }
    }
}

//  use core::num::Unsigned;
use fixed::types::extra::Unsigned;

pub fn fold_unsigned<
    T: Unsigned
        + core::ops::Shl<usize>
        + core::ops::BitOr<<T as core::ops::Shl<usize>>::Output, Output = T>
        + core::convert::From<bool>,
>(
    iteratable: impl Iterator<Item = bool>,
) -> T {
    iteratable.enumerate().fold(Default::default(), |acc, (i, bit_state)| {
        let value = acc | ((Into::<T>::into(bit_state)) << i);
        value
    })
}

impl From<DataOt> for u16 {
    fn from(item: DataOt) -> Self {
        const SHIFT_SKIP_SLAVE_STATUS: usize = 8;
        match item {
            DataOt::MasterStatus(status) => status.iter().enumerate().fold(0_u16, |acc, (i, bit_state)| {
                let value = acc | ((bit_state as u16) << (7 - i + SHIFT_SKIP_SLAVE_STATUS));
                value
            }),
            DataOt::SlaveStatus(status) => status.iter().enumerate().fold(0_u8, |acc, (i, bit_state)| {
                let value = acc | ((bit_state as u8) << (7 - i));
                value
            }) as u16,
            DataOt::ControlSetpoint(temperature)
            | DataOt::BoilerTemperature(temperature)
            | DataOt::DWHTemperature(temperature) => temperature.into(),
            _ => todo!(),
        }
    }
}

pub struct DataOtIterator<'a> {
    data: &'a DataOt,
    data_value: u16,
    index: Option<usize>,
    //  iter_option: Option<dyn Iterator<Item=bool>>,
    //  pub fn new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
}

const DATA_OT_SIZE: usize = 16usize;

impl<'a> Iterator for DataOtIterator<'a> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let next_bit = match self.index {
            Some(index) => {
                if index < DATA_OT_SIZE - 1 {
                    Some(true)
                } else {
                    None
                }
            }
            _ => None,
        };
        next_bit
    }
}

impl TryFrom<u8> for OpenThermMessageCode {
    type Error = ();
    fn try_from(item: u8) -> Result<Self, Self::Error> {
        match item {
            item if item == (OpenThermMessageCode::Status as u8) => Ok(OpenThermMessageCode::Status),
            item if item == (OpenThermMessageCode::ControlSetpoint as u8) => Ok(OpenThermMessageCode::ControlSetpoint),
            item if item == (OpenThermMessageCode::RelativeModulationLevel as u8) => {
                Ok(OpenThermMessageCode::RelativeModulationLevel)
            }
            item if item == (OpenThermMessageCode::BoilerTemperature as u8) => {
                Ok(OpenThermMessageCode::BoilerTemperature)
            }
            item if item == (OpenThermMessageCode::DWHTemperature as u8) => Ok(OpenThermMessageCode::DWHTemperature),
            _ => {
                todo!();
                Err(())
            }
        }
    }
}

//  impl Enum {
//      fn discriminant(&self) -> u8 {
//      }

#[derive(Copy, Clone, Debug)]
pub struct OpenThermMessage {
    // start bit
    // parity bit
    parity: bool,
    msg_type: MessageType,
    //  spare 4b
    //  data_id: u8,
    data_id: DataOt,
    //  data_value: u16, //  TODO: remove that one
    //  stop bit
    cmd: OpenThermMessageCode,
}

//  TODO: Implement iterator: https://dev.to/wrongbyte/implementing-iterator-and-intoiterator-in-rust-3nio
//?
#[derive(Debug)]
enum OpenThermIteratorState {
    StartBit,
    Parity,
    MessageType3b(usize),
    Spare4b(usize),
    DataId8b(usize),
    DataValue16b(usize),
    StopBit,
    Depleted,
}

pub struct OpenThermMessageIterator<'a> {
    message: &'a OpenThermMessage,
    state: OpenThermIteratorState,
    parity_count: u32, //  parity count + Stop Bit which means that it should yield odd number of
    //  ones in the 33 LSb of the frame
    data_value: u16,
}

impl<'a> Iterator for OpenThermMessageIterator<'a> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let (new_state, return_value) = match self.state {
            OpenThermIteratorState::StartBit => (OpenThermIteratorState::Parity, Some(true)),
            OpenThermIteratorState::Parity => (OpenThermIteratorState::MessageType3b(0), Some(self.message.parity)),
            OpenThermIteratorState::MessageType3b(shift) => {
                let value = 0_u8 != (self.message.msg_type as u8) & (0x1_u8 << (MESSAGE_TYPE_BIT_LEN - 1 - shift));
                if shift >= MESSAGE_TYPE_BIT_LEN - 1 {
                    (OpenThermIteratorState::Spare4b(0), Some(value))
                } else {
                    (OpenThermIteratorState::MessageType3b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::Spare4b(shift) => {
                if shift >= OT_FRAME_SKIP_SPARE - 1 {
                    (OpenThermIteratorState::DataId8b(0), Some(false))
                } else {
                    (OpenThermIteratorState::Spare4b(shift + 1), Some(false))
                }
            }
            OpenThermIteratorState::DataId8b(shift) => {
                let value = 0_u8 != (self.message.cmd as u8) & (0x1_u8 << (MESSAGE_DATA_ID_BIT_LEN - 1 - shift));
                if shift >= MESSAGE_DATA_ID_BIT_LEN - 1 {
                    (OpenThermIteratorState::DataValue16b(0), Some(value))
                } else {
                    (OpenThermIteratorState::DataId8b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::DataValue16b(shift) => {
                let value = 0_u16 != self.data_value & (0x1_u16 << (MESSAGE_DATA_VALUE_BIT_LEN - 1 - shift));
                if shift >= MESSAGE_DATA_VALUE_BIT_LEN - 1 {
                    (OpenThermIteratorState::StopBit, Some(false))
                } else {
                    (OpenThermIteratorState::DataValue16b(shift + 1), Some(value))
                }
            }
            OpenThermIteratorState::StopBit => (OpenThermIteratorState::Depleted, Some(true)),
            _ => (OpenThermIteratorState::Depleted, None),
        };
        self.state = new_state;

        match return_value {
            Some(v) => {
                self.parity_count += v as u32;
            }
            _ => {}
        }
        return_value
    }
}

impl OpenThermMessage {
    pub fn iter(&self) -> OpenThermMessageIterator {
        OpenThermMessageIterator {
            message: self,
            state: OpenThermIteratorState::StartBit,
            parity_count: 0u32,
            data_value: self.data_id.into(),
        }
    }
    fn float8_8_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Float8_8, Error> {
        let data_value = iterator.clone().take(15).enumerate().fold(0_i16, |acc, (i, &bit_state)| {
            let value = acc | ((bit_state as i16) << i);
            value
        });
        let sign = iterator.skip(15).next().unwrap();
        //  return:
        match *sign {
            true => Ok(Float8_8::Signed(-1i16 * data_value)),
            false => Ok(Float8_8::Signed(data_value)),
        }
    }

    pub fn get_data(&self) -> DataOt {
        return self.data_id.clone();
    }

    pub fn get_type(&self) -> MessageType {
        return self.msg_type.clone();
    }

    pub fn folded(&self) -> u32 {
        self.iter()
            .skip(1)
            .take(CAPTURE_OT_FRAME_PAYLOAD_SIZE)
            .enumerate()
            .fold(0_u32, |acc, (i, bit_state)| {
                let value = acc | ((bit_state as u32) << (CAPTURE_OT_FRAME_PAYLOAD_SIZE - 1 - i));
                value
            })
    }

    fn decode_data_ot<'a>(
        data_id: OpenThermMessageCode,
        msg_type: MessageType,
        data_value_iterator: impl Iterator<Item = &'a bool> + Clone,
    ) -> Result<DataOt, Error> {
        let data_float8_8 = Self::float8_8_from_iter(data_value_iterator.clone()).unwrap();
        match data_id {
            OpenThermMessageCode::Status => match msg_type {
                MessageType::ReadData => {
                    //  iterate flag8 into bool flags
                    Ok(DataOt::MasterStatus(MasterStatus::new_from_iter(
                        data_value_iterator.skip(8),
                    )?))
                }
                MessageType::ReadAck => {
                    //  iterate flag8 into bool flags
                    Ok(DataOt::SlaveStatus(SlaveStatus::new_from_iter(data_value_iterator)?))
                }
                _ => {
                    return Err(Error::DecodingError);
                }
            },
            //  fn float8_8_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Float8_8, Error> {
            OpenThermMessageCode::ControlSetpoint => Ok(DataOt::ControlSetpoint(
                data_float8_8.try_into().map_err(|e| Error::DataFormatError)?,
            )),
            OpenThermMessageCode::BoilerTemperature => Ok(DataOt::BoilerTemperature(
                data_float8_8.try_into().map_err(|e| Error::DataFormatError)?,
            )),
            _ => {
                log::error!("Unrecognized DataId");
                todo!();
                return Err(Error::UnknownDataId);
            }
        }
    }

    pub fn try_new_from_iter<'a>(iterator: impl Iterator<Item = &'a bool> + Clone) -> Result<Self, Error> {
        let folded =
            iterator
                .clone()
                .take(CAPTURE_OT_FRAME_PAYLOAD_SIZE)
                .enumerate()
                .fold(0_u32, |acc, (i, &bit_state)| {
                    let value = acc | ((bit_state as u32) << (CAPTURE_OT_FRAME_PAYLOAD_SIZE - 1 - i));
                    value
                });
        log::info!("Folded OR: 0x{:x}", folded);
        //  Check parity for whole OT Frame
        if folded.count_ones() % 2 == 1 {
            log::error!("OT Frame Parity Error: 0x{:x}", folded);
            return Err(Error::ParityError);
        }

        let data_value_iterator = iterator.clone().take(MESSAGE_DATA_VALUE_BIT_LEN);
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

        let message_code: OpenThermMessageCode = match data_id.try_into() {
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
        let parity = iter.next().unwrap().clone();

        //  Deconding based on other types
        let data_id_decoded = Self::decode_data_ot(message_code, msg_type, data_value_iterator)?;

        log::info!("msg_type: {:?}", msg_type);
        log::info!("message_code: {:?}", message_code);
        log::info!("data_id_decoded: {:?}", data_id_decoded);
        Ok(Self {
            parity: parity,
            msg_type: msg_type,
            data_id: data_id_decoded,
            cmd: message_code,
        })
    }

    pub fn new_from_data_ot(cmd_type: MessageType, data_ot: DataOt) -> Result<Self, Error> {
        let cmd_decoded: OpenThermMessageCode = data_ot.into();
        let number_of_ones = (cmd_type as u8).count_ones()
            + (cmd_decoded as u8).count_ones()
            + (<DataOt as Into<u16>>::into(data_ot)).count_ones();
        Ok(Self {
            parity: if number_of_ones % 2 == 1 { true } else { false },
            msg_type: cmd_type,
            data_id: data_ot,
            cmd: cmd_decoded,
        })
    }
}

pub trait OpenThermInterface {
    async fn write(&mut self, cmd: DataOt) -> Result<(), Error>;
    //  read need more parameters: timeout, number of expected bits etc
    async fn read(&mut self, cmd: DataOt) -> Result<DataOt, Error>;
    async fn listen(&mut self) -> Result<OpenThermMessage, Error>;
    async fn send(&mut self, message: OpenThermMessage) -> Result<(), Error>;
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

impl Sub for Temperature {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        let ts = if let Temperature::Celsius(tsval) = self {
            tsval
        } else {
            0
        };
        let to = if let Temperature::Celsius(toval) = other {
            toval
        } else {
            0
        };
        Self::Celsius(ts - to)
    }
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
