use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CHState, CommunicationState, DataOt, MessageType, OpenThermInterface, OpenThermMessage, OpenThermMessageCode,
    Temperature,
};
use crate::opentherm_interface::{DWHState, FlameState, SlaveStatus};
use embassy_time::{Duration, Instant, Timer};

const TEMP_RATE_PER_60SEC: Temperature = Temperature::Celsius(5);
//  impl Default for Instant {
//      fn default() -> Self {
//          Instant::now()
//      }
//  }
//  #[derive(Default)]
pub struct BoilerSimulation {
    //  struct for testing operation of the driver used for boiler control
    boiler_temperature: Temperature,
    setpoint: Temperature,
    on_off_state: CHState,
    dwh_active: DWHState,
    flame: FlameState,
    last_process_call: Instant,
}

impl BoilerSimulation {
    pub fn new() -> Self {
        Self {
            boiler_temperature: Temperature::Celsius(18),
            setpoint: Temperature::Celsius(16),
            on_off_state: CHState::Enable(false),
            dwh_active: DWHState::Enable(true),
            flame: FlameState::Active(false),
            last_process_call: Instant::now(),
        }
    }
    //  accepts OpenTherm message
    //  returns the one that would be sent in response by the Boiler
    pub fn process(&mut self, msg: OpenThermMessage) -> Result<OpenThermMessage, OtError> {
        let cmd = msg.get_data();
        let period_last_call = Instant::now().duration_since(self.last_process_call);
        self.last_process_call = Instant::now();
        // update the boiler stat

        // elevate integer calculation 1000 times
        let time_ratio_ticks = 1000_u64 * period_last_call.as_ticks() / Duration::from_secs(60).as_ticks();
        let new_temperature = match self.boiler_temperature {
            Temperature::Celsius(int) => {
                let coef = match TEMP_RATE_PER_60SEC {
                    Temperature::Celsius(v) => 1000 * v,
                };
            }
        };

        let correct_type_response = match msg.get_type() {
            MessageType::ReadData => MessageType::ReadAck,
            MessageType::WriteData => MessageType::WriteAck,
            _ => MessageType::InvalidData,
        };

        //  Command handling:
        let data = match cmd {
            DataOt::MasterStatus(status) => {
                let slave_status = SlaveStatus::new(self.on_off_state, self.dwh_active, self.flame);
                if status.ch_enable == CHState::Enable(true) {
                    log::info!("Boiler Simulation: Got CH Enable signal set true ------------->  CH enable");
                }
                self.on_off_state = status.ch_enable;
                self.dwh_active = status.dwh_enable;
                if msg.get_type() != MessageType::ReadData {
                    log::error!("Boiler Simulation: Invalid msg type for MasterStatus");
                    return Err(OtError::IncompatibleTypeData);
                }
                DataOt::SlaveStatus(slave_status)
            }
            DataOt::ControlSetpoint(setpoint) => {
                let response = DataOt::ControlSetpoint(self.setpoint);
                self.setpoint = setpoint;
                if msg.get_type() != MessageType::WriteData {
                    log::error!("Boiler Simulation: Invalid msg type for Setpoint");
                    return Err(OtError::IncompatibleTypeData);
                }
                response
            }
            DataOt::BoilerTemperature(temperature) => {
                let response = DataOt::BoilerTemperature(self.boiler_temperature);
                if msg.get_type() != MessageType::ReadData {
                    log::error!("Boiler Simulation: Invalid msg type for BoilerTemperature");
                    return Err(OtError::IncompatibleTypeData);
                }
                response
            }
            _ => {
                log::error!("Boiler simulation: command not implemented");
                return Err(OtError::CommandNotSupported);
            }
        };
        OpenThermMessage::new_from_data_ot(correct_type_response, data)
    }
}

//  impl OpenThermInterface for BoilerSimulation {
//      async fn listen(&mut self) -> Result<DataOt, OtError> {
//          //  here comes listening
//          todo!()
//      }
//      async fn write(&mut self, cmd: DataOt) -> Result<(), OtError> {
//          let response = match cmd {
//              DataOt::MasterStatus(status) => {}
//              DataOt::ControlSetpoint(setpoint) => {}
//              DataOt::BoilerTemperature(temperature) => {
//                  self.setpoint = temperature;
//              }
//              _ => {
//                  todo!()
//              }
//          };
//
//          Ok(())
//      }
//
//      async fn read(&mut self, cmd: DataOt) -> Result<u32, OtError> {
//          match cmd {
//              DataOt::MasterStatus(status) => {
//                  let msg = OpenThermMessage::new(0x00u32);
//              }
//              DataOt::ControlSetpoint(temperature) => {}
//              DataOt::BoilerTemperature(temperature) => {
//                  //  self.setpoint;
//              }
//              _ => todo!()
//          }
//          Ok(0u32)
//      }
//  }
