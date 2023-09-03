use core::fmt;

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

#[repr(u8)]
pub enum OpenThermMessageCode {
    Status = 0x00,
    TSet = 0x01,
    BoilerTemperature = 0x25,
}

#[derive(PartialEq)]
pub struct OpenThermMessage {
    data_value_: u32,
}

impl OpenThermMessage {
    pub fn new(value: u32) -> Self
    {
        Self{ data_value_: value }
    }
    pub fn get_raw_data_value(self) -> u32 {
        self.data_value_
    }
}

impl fmt::LowerHex for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.data_value_;
        fmt::LowerHex::fmt(&val, f)
    }
}

// #[derive(PartialEq,Debug)]
impl core::fmt::Debug for OpenThermMessage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let val = self.data_value_;
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

