use embassy_time::Duration;
use heapless::Vec;

use super::InitLevel;

pub enum CodingError {
    InitialLevelError,
    PeriodEncondingError,
    FinishExpectedError,
    NoSpaceAvailable,
}

//  __|'' manchester 'one'
//  ''|__ manchester 'zero'
//
//    __|''''|__ manchester '10'
//    ''|____|'' manchester '01'
//
//    Assumptions: first edge is always the transition from idle state and it marks start of the
//    bit.

//  TODO: Prepare data first - remove first and last long periods
pub fn manchester_decode<
    const N: usize, /*Vec max size*/
    const T: usize, /*period duration in us which specifies smalles amount of time the linee can state in one state*/
>(
    level: InitLevel,
    mut wire_state_periods: Vec<Duration, N>,
) -> Result<Vec<bool, N>, CodingError> {
    let mut output = Vec::<bool, N>::new();
    //  discard first cycle as it is not relevant because
    //  capture should start before first relevant level transition
    //  TODO: possibly do it upstream
    let _ignore = wire_state_periods.pop().unwrap();
    //  Return if initial level is not idle level
    if level != InitLevel::Low {
        return Err(CodingError::InitialLevelError);
    }

    let resolution = Duration::from_micros(T as u64).as_ticks();
    //  Initial level is very important as it determines all other transitions
    //  TODO: make it more redundant and safe to mistake one level transition and still have most
    //  of the data correct
    let level = level; // init level later used as mut to handle iterations

    //  First half of initial 'one' bit:
    match wire_state_periods.pop() {
        Some(pop_period) => {
            if (100u64 * pop_period.as_ticks() < 95u64 * resolution)
                || (100u64 * pop_period.as_ticks() > 105 * resolution)
            {
                return Err(CodingError::PeriodEncondingError);
            }
        }
        _ => {}
    }

    //  Loop over the periods; each loop starts at the middle of already determined bit and based
    //  on the nex period detects next bit level. If needed it pops next periods until it lands
    //  in the middle of just determined bit.
    while let Some(pop_period) = wire_state_periods.pop() {
        //  Check if the period is of value around of the resolution:
        if (100u64 * pop_period.as_ticks() > 95u64 * resolution) && (100u64 * pop_period.as_ticks() < 105 * resolution)
        {
            match level {
                InitLevel::Low => {
                    output.push(true).unwrap(); //  a one after one
                }
                InitLevel::High => {
                    output.push(false).unwrap(); //  a zero after zero
                }
            }
            //  strip complementary half-period part
            match wire_state_periods.pop() {
                Some(pop_period) => {
                    //  check if period is of correct value of around one resolution:
                    if 100u64 * pop_period.as_ticks() > 105 * resolution {
                        //  if the period is longer it suggest it is the last period
                        match wire_state_periods.is_empty() {
                            true => {
                                break;
                            }
                            false => {
                                return Err(CodingError::FinishExpectedError);
                            }
                        }
                    } else if 100u64 * pop_period.as_ticks() < 95u64 * resolution {
                        return Err(CodingError::PeriodEncondingError);
                    }
                    //  else: the length is withing range -> continue
                }
                _ => {
                    //  End of the capture as there is no more elements?
                    break;
                }
            }
        }
        //  Check if the period is of value around two times of the resolution:
        else if (100u64 * pop_period.as_ticks() > 195u64 * resolution)
            && (100u64 * pop_period.as_ticks() < 205u64 * resolution)
        {
            let new_level = match level {
                InitLevel::Low => {
                    true //  a one after one
                }
                InitLevel::High => {
                    false //  a zero after zero
                }
            };
            match output.push(new_level) {
                Err(_returned_element) => {
                    //  returns back elemeent that was not possible
                    return Err(CodingError::NoSpaceAvailable);
                }
                _ => { // correct
                }
            }
        } else {
            //  Half way through the bit there is non-conforming period length:
            return Err(CodingError::PeriodEncondingError);
            //  TODO: check for number of elements in the queue that left
        }
    }
    Ok(output)
}

pub fn manchester_encode<
    const N: usize, /*Vec max size*/
    const T: usize, /*period duration in us which specifies smalles amount of time the linee can state in one state*/
>(
    level: InitLevel,
    mut input: Vec<bool, N>,
) -> Result<Vec<bool, N>, CodingError> {
    let mut output = Vec::<bool, N>::new();
    while let Some(element) = input.pop() {
        match output.push(element) {
            Err(_returned_element) => {
                //  returns back elemeent that was not possible
                return Err(CodingError::NoSpaceAvailable);
            }
            _ => { // correct
            }
        }
        match output.push(!element) {
            Err(_returned_element) => {
                //  returns back elemeent that was not possible
                return Err(CodingError::NoSpaceAvailable);
            }
            _ => { // correct
            }
        }
    }
    Ok(output)
}

//  #[cfg(test)]
//  mod tests {
//  //  TODO: Write some tests
//  //
//  //  test decoder of the duration table into manchaster sync protocol (check in WIO project)
//  //
//      const VEC_SIZE: usize = 64;
//      use super::*;
//      #[test]
//      fn manchester_encode() {
//          let mut timestamps = Vec::<usize, VEC_SIZE>::new();
//          timestamps.push(2_000 as usize);
//          timestamps.push(3_000 as usize);
//          timestamps.push(4_000 as usize);
//          timestamps.push(5_000 as usize);
//          ManchesterDecode::<VEC_SIZE, 1_000>(InitLevel::Low, timestamps, );
//          ()
//      }
//  }
