#![no_std]

use crate::opentherm_interface::{OpenThermInterface, UptimeCounter, CommunicationState, Temperature, OpenThermMessageCode, OpenThermMessage, MessageType};
use crate::opentherm_interface::{Error as OtError};

    struct BoilerSimulation{
        //  struct for testing operation of the driver used for boiler control
        boiler_temperature: Temperature,
        setpoint: Temperature,
    }

    impl OpenThermInterface for BoilerSimulation {
        fn write(&mut self, cmd: OpenThermMessageCode, data:u32) ->Result<(), OtError>
        {
            let response = match cmd {
               OpenThermMessageCode::Status => {
               },
               OpenThermMessageCode::TSet => {
               },
               OpenThermMessageCode::BoilerTemperature => {
                   self.setpoint = Temperature::Celsius(data as i16);
               }
            };

            Ok(())
        }

        fn read(&self, cmd: OpenThermMessageCode) -> Result<u32, OtError>
        {
            match cmd {
               OpenThermMessageCode::Status => {
                   let msg = OpenThermMessage::new(0x00u32);
               },
               OpenThermMessageCode::TSet => {
               },
               OpenThermMessageCode::BoilerTemperature => {
                   //  self.setpoint;
               }
            }
            Ok(0u32)
        }
    }
