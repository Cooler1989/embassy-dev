use crate::opentherm_interface::Error as OtError;
use crate::opentherm_interface::{
    CommunicationState, OpenThermInterface, OpenThermMessageCode, Temperature, UptimeCounter,
};

const MIN_TEMPERATURE_SETPOINT: Temperature = Temperature::Celsius(16i16);
const BOOST_TEMPERATURE_DEFAULT: Temperature = Temperature::Celsius(24i16);
const BURNER_START_LOW_TRESHOLD: Temperature = Temperature::Celsius(6i16);
const BURNER_STOP_HIGH_TRESHOLD: Temperature = Temperature::Celsius(8i16);

//  The function of boiler control is to maintaing boiler state connected over the OpenTherm
//  interface
//  The most important function is to detect burner start and to react on it with boosting setpoint
//  value for specific period of time to allow to sustain flame - the origin of this requirement is that some
//  boiler producer made a faulty product that is not able to push the heat produced at start of
//  the burner due to high power used at that phase which is around 70-75%
//  After around 15 seconds power can be modulated to lower values and can operate on its own.
struct BoilerControl<D: OpenThermInterface, T: UptimeCounter> {
    state: BoilerStatus,
    communication_state: CommunicationState,
    device: D,
    boost_setpoint: Temperature,
    setpoint: Temperature,
    timer: T,
    burner_start_timestamp: u32,
}

enum BoilerStatus {
    Uninitialized, //  to be changed to type level programming
    Idle,
    BurnerStart,
    BurnerModulation,
}

impl<D: OpenThermInterface, T: UptimeCounter> BoilerControl<D, T> {
    pub fn new(opentherm_device: D, timer: T) -> BoilerControl<D, T>
    where
        D: OpenThermInterface,
    {
        //  read boiler versions ?
        //  read opentherm versions ?
        //  whichever is mandatory - multiple tries
        //  let state: BoilerStatus::Idle;
        BoilerControl::<D, T> {
            state: BoilerStatus::Idle,
            communication_state: CommunicationState::StatusExchange,
            device: opentherm_device,
            boost_setpoint: BOOST_TEMPERATURE_DEFAULT,
            setpoint: MIN_TEMPERATURE_SETPOINT,
            timer: timer,
            burner_start_timestamp: 0u32,
        }
    }

    fn set_point_control(&mut self) {
        let Temperature::Celsius(to_send) = match self.state {
            BoilerStatus::Uninitialized => MIN_TEMPERATURE_SETPOINT,
            BoilerStatus::Idle | BoilerStatus::BurnerModulation => self.setpoint,
            BoilerStatus::BurnerStart => self.setpoint + self.boost_setpoint,
        };

        self.device
            .write(OpenThermMessageCode::BoilerTemperature, to_send as u32);
    }

    fn state_transition(&mut self) {
        //  Check timer expirations and does maintain state transitions
        //  BoilerStatus::Idle->BurenerStart->BurnerModulation
    }

    // turn into await / use await driver for reading/writing OpenTherm Bus:
    pub async fn process(&mut self) -> Result<(), OtError> {
        self.state_transition(); //  check timer expiration and realize
                                 //  BoilerStatus::BurnerStart->BurnerModulation transition
        let new_state = match self.communication_state {
            CommunicationState::StatusExchange => {
                self.device.write(OpenThermMessageCode::Status, 0x00);
                CommunicationState::Control
            }
            CommunicationState::Control => {
                self.set_point_control();
                CommunicationState::Diagnostics
            }
            CommunicationState::Diagnostics => {
                if let Ok(value) = self.device.read(OpenThermMessageCode::BoilerTemperature).await {
                    value; //  compose diagnostics
                }
                CommunicationState::StatusExchange
            }
        };
        self.communication_state = new_state;
        Ok(())
    }
    //  Always or on demand / or combined?
    //  Read back value of setpoint for comparison
    pub fn set_co_temp(&mut self, setpoint: Temperature) {
        self.setpoint = setpoint;
    }
}
