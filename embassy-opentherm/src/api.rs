use core::fmt;

#[repr(u8)]
#[derive(PartialEq, Copy, Clone)]
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

impl From<MessageType> for u32
{
    fn from (item: MessageType) -> Self
    {
        item as u32
        //  match item {
        //      MessageType::ReadData => 0b000 as u32,
        //      MessageType::WriteData => 0b001 as u32,
        //      MessageType::InvalidData => 0b010 as u32,
        //      MessageType::Reserved => 0b011 as u32,
        //      MessageType::ReadAck => 0b100 as u32,
        //      MessageType::WriteAck => 0b101 as u32,
        //      MessageType::DataInvalid => 0b110 as u32,
        //      MessageType::UnknownDataId => 0b111 as u32,
        //  }
    }
}

#[repr(u8)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

#[repr(u16)]
pub enum OpenThermDataValue { u16 }

#[derive(PartialEq, Copy, Clone)]
enum OpenThermMessageValue {
    Value(u16)
}

#[derive(PartialEq, Copy, Clone)]
pub struct OpenThermMessage {
    //  data_value_: u32,

    // start bit
    // parity bit
    msg_type_: MessageType,
    //  spare 4b
    data_id_: u8,
    value_: OpenThermMessageValue,
    //  stop bit
}

impl From<OpenThermMessageValue> for u16
{
    fn from (item: OpenThermMessageValue) -> Self
    {
        match item { OpenThermMessageValue::Value(x) => x as u16 }
    }
}

impl From<OpenThermMessage> for u32
{
    fn from (item: OpenThermMessage) -> Self
    {
        //  item.data_value_ as u32
        match item.value_ { OpenThermMessageValue::Value(x) => x as u32 }
    }
}

impl OpenThermMessage {
    pub fn new(value: u32) -> Self
    {
        Self
        {
            // data_value_: value,
            msg_type_: MessageType::WriteData,
            data_id_: 0u8,
            value_: OpenThermMessageValue::Value(0u16),
        }
    }
    pub fn get_raw_data_value(self) -> u32 {
        // self.data_value_
        0u32  //  TODO: implement!!!
    }
}

impl fmt::LowerHex for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.get_raw_data_value();
        fmt::LowerHex::fmt(&val, f)
    }
}

// #[derive(PartialEq,Debug)]
impl core::fmt::Debug for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.get_raw_data_value();
        fmt::LowerHex::fmt(&val, f)
        //  f.debug_struct("OtMsg").
        //      field("data", &self.data_value_).finish()
    }
}

pub trait Error: core::fmt::Debug {
    /// Convert error to a generic I2C error kind
    ///
    /// By using this method, I2C errors freely defined by HAL implementations
    /// can be converted to a set of generic I2C errors upon which generic
    /// code can act.
    fn kind(&self) -> () /*ErrorKind*/;
}

/// This just defines the error type, to be used by the other traits.
pub trait ErrorType {
    /// Error type
    type Error: Error;
}

impl<T: ErrorType> ErrorType for &mut T {
    type Error = T::Error;
}

pub trait OpenThermDevice: ErrorType {
    async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<(), Self::Error> {
        //  place here some conversion and frame assembly
        self.transaction(cmd).await
    }
    async fn transaction(&mut self, cmd: OpenThermMessageCode) -> Result<(), Self::Error>;
}

pub trait OpenThermBus {
    type Error;
    type Output;
    async fn transact(&mut self, data: Self::Output) -> Result<Self::Output, Self::Error>;
    async fn tx(&mut self, data: Self::Output) -> Result<(), Self::Error>;
    async fn rx(&mut self) -> Result<Self::Output, Self::Error>;
}

pub trait OpenThermSlave: OpenThermBus {
    async fn wait_reception_run_callback_respond<F, ErrorSpecific>(&mut self, callback: F) -> Result<(), Self::Error>
    where
        F: Fn(Self::Output) -> Result<Self::Output, ErrorSpecific>;
}

