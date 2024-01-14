use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CHState, CommunicationState, MessageType, OpenThermInterface, OpenThermMessage, OpenThermMessageCode, Temperature, DataOt,
};
use crate::opentherm_interface::{SlaveStatus, DWHState, FlameState};
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
    pub fn process(&mut self, cmd: DataOt) -> Result<DataOt, OtError> {
        let period_last_call = Instant::now().duration_since(self.last_process_call);
        self.last_process_call = Instant::now();
        // update the boiler stat

        // elevate integer calculation 1000 times
        let time_ratio_ticks = 1000_u64*period_last_call.as_ticks()/Duration::from_secs(60).as_ticks();
        let new_temperature = match self.boiler_temperature {
            Temperature::Celsius(int) => {
                let coef = match TEMP_RATE_PER_60SEC { Temperature::Celsius(v) => 1000*v };
            }
        };

        //  Command handling:
        match cmd {
            DataOt::MasterStatus(status) => {
                let slave_status = SlaveStatus::new(self.on_off_state, self.dwh_active, self.flame);
                self.on_off_state = status.ch_enable;
                self.dwh_active = status.dwh_enable;
                Ok(DataOt::SlaveStatus(slave_status))
            },
            _ => {
                return Err(OtError::CommandNotSupported);
            }
        }
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
