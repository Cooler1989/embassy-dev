use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CommunicationState, MessageType, OpenThermInterface, OpenThermMessage, OpenThermMessageCode, Temperature,
    UptimeCounter,
};

pub struct BoilerSimulation {
    //  struct for testing operation of the driver used for boiler control
    boiler_temperature: Temperature,
    setpoint: Temperature,
}

impl BoilerSimulation {
    pub fn new() -> Self {
        Self {
            boiler_temperature: Temperature::Celsius(18),
            setpoint: Temperature::Celsius(16),
        }
    }
    //  accepts OpenTherm message
    //  returns the one that would be sent in response by the Boiler
    pub fn process(&self, cmd: OpenThermMessageCode, value: u32) -> Result<(OpenThermMessageCode, u32), OtError>
    {
        match cmd {
            OpenThermMessageCode::Status => {
                Ok((OpenThermMessageCode::Status, 0x0_u32))
            },
            _ => {
                return Err(OtError::CommandNotSupported);
            }
        }
    }
}

impl OpenThermInterface for BoilerSimulation {
    async fn write(&mut self, cmd: OpenThermMessageCode, data: u32) -> Result<(), OtError> {
        let response = match cmd {
            OpenThermMessageCode::Status => {}
            OpenThermMessageCode::TSet => {}
            OpenThermMessageCode::BoilerTemperature => {
                self.setpoint = Temperature::Celsius(data as i16);
            }
        };

        Ok(())
    }

    async fn read(&mut self, cmd: OpenThermMessageCode) -> Result<u32, OtError> {
        match cmd {
            OpenThermMessageCode::Status => {
                let msg = OpenThermMessage::new(0x00u32);
            }
            OpenThermMessageCode::TSet => {}
            OpenThermMessageCode::BoilerTemperature => {
                //  self.setpoint;
            }
        }
        Ok(0u32)
    }
}
