use embassy_time::Duration;
use heapless::Vec;

use super::InitLevel;

//  TODO:
//  - consider changing implementation to more generic one with use of Iterators

const TRESHOLD_PERCENT_LEVEL: u32 = 5;
const HUNDRED_PERCENT: u32 = 100;

#[derive(Debug)]
pub enum CodingError {
    InitialLevelError,
    PeriodEncodingError,
    FinishExpectedError,
    NoSpaceAvailable,
    WrongInput,
}

//  __|'' manchester 'one'
//  ''|__ manchester 'zero'
//
//  __|''''|__ manchester '10'
//  ''|____|'' manchester '01'
//
//  Assumptions: first edge is always the transition from idle state and it marks start of the
//  bit.

#[derive(Debug)]
pub enum DecodingDebugEvent {
    BeforeStart,
    StartOk,
    DoublePulse,
    ShortPulse,
    ComplementaryShortPulse,
    ComplementaryLongPulseFinish,
    ComplementaryLongPulseError,
    ComplementaryPulseOk,
    HalfComplementaryStrip,
    HalfPeriod, //  pulling in half-period at the beginning of the while loop
}

#[derive(Debug)]
pub struct DataDecodingDebug {
    pub duration: Duration,
    pub event: DecodingDebugEvent,
    pub count: usize,
}

impl DataDecodingDebug {
    pub fn print_debug_info<const N: usize>(vec: Vec<DataDecodingDebug, N>) {
        for item in vec {
            match item {
                DataDecodingDebug { duration, event, count } => {
                    log::info!("ManEvent[{count}]: evt:{:?}, duration: {}", event, duration.as_ticks());
                }
            }
        }
    }
}

//  TODO: Prepare data first - remove first and last long periods
pub fn manchester_decode<const N: usize /*Vec max size*/>(
    level: InitLevel,
    mut wire_state_periods: Vec<Duration, N>,
    resolution: Duration, /*period duration which specifies smalles amount of time the line can state in one state*/
) -> Result<Vec<bool, N>, (CodingError, Vec<DataDecodingDebug, N>)> {
    let mut count = 0_usize;
    let mut output = Vec::<bool, N>::new();
    let mut err_output = Vec::<DataDecodingDebug, N>::new();
    log::info!(
        "Start manchester decoding vec of length {} elements with resolution: {} ticks",
        wire_state_periods.len(),
        resolution.as_ticks()
    );
    //  discard first cycle as it is not relevant because
    //  capture should start before first relevant level transition
    //  TODO: possibly do it upstream:
    if wire_state_periods.is_empty() {
        return Err((CodingError::WrongInput, err_output));
    }
    if let Some(ignore_first_period) = wire_state_periods.pop() {
        log::info!(
            "Ignoring the first period with {} ticks or {} periods",
            ignore_first_period.as_ticks(),
            ignore_first_period.as_ticks() / resolution.as_ticks()
        );
        err_output
            .push(DataDecodingDebug {
                duration: ignore_first_period,
                event: DecodingDebugEvent::BeforeStart,
                count: count,
            })
            .unwrap();
    }
    //  Return if initial level is not idle level
    if level != InitLevel::Low {
        return Err((CodingError::InitialLevelError, err_output));
    }

    //  Initial level is very important as it determines all other transitions
    //  TODO: make it more redundant and safe to mistake one level transition and still have most
    //  of the data correct
    let mut level = level; // init level later used as mut to handle iterations

    //  First half of initial 'one' bit:
    match wire_state_periods.pop() {
        Some(pop_period) => {
            if (HUNDRED_PERCENT * pop_period < (HUNDRED_PERCENT - TRESHOLD_PERCENT_LEVEL) * resolution)
                || (HUNDRED_PERCENT * pop_period > (HUNDRED_PERCENT + TRESHOLD_PERCENT_LEVEL) * resolution)
            {
                log::error!(
                    "First valid period should be equal to one unit period, but it is: {} ticks",
                    pop_period.as_ticks()
                );
                return Err((CodingError::PeriodEncodingError, err_output));
            }
        }
        _ => {
            log::error!("Not even one valid period found in the input");
            return Err((CodingError::PeriodEncodingError, err_output));
        }
    }
    output.push(true).unwrap(); //  First 'one' bit

    //  Loop over the periods; each loop starts in the middle of already determined bit and based
    //  on the next period detects next bit level. If needed it pops next periods until it lands
    //  in the middle of just determined bit.
    while let Some(pop_period) = wire_state_periods.pop() {
        //  log::info!("per for in_0[{count}] = {}", pop_period.as_ticks());
        err_output
            .push(DataDecodingDebug {
                duration: pop_period,
                event: DecodingDebugEvent::HalfPeriod,
                count: count,
            })
            .unwrap();
        //  Check if the period is of value around of the resolution:
        if (HUNDRED_PERCENT * pop_period > (HUNDRED_PERCENT - TRESHOLD_PERCENT_LEVEL) * resolution)
            && (HUNDRED_PERCENT * pop_period < (HUNDRED_PERCENT + TRESHOLD_PERCENT_LEVEL) * resolution)
        {
            //            b--e      begin/end for the period
            //          __|''|__|'' manchester '11'
            //          ''|__|''|__ manchester '00'
            //          aaaaa|bbbbb
            let output_candidate = match level {
                InitLevel::Low => {
                    true //  a one after one
                }
                InitLevel::High => {
                    false //  a zero after zero
                }
            };
            //  strip complementary half-period part
            match wire_state_periods.pop() {
                //                   b--e   begin/end for the period
                //              __|''|__|'' manchester '11'
                //              ''|__|''|__ manchester '00'
                //              aaaaa|bbbbb a - previous, b - current
                Some(pop_period) => {
                    //  check if period is of correct value of around one resolution:
                    if HUNDRED_PERCENT * pop_period > (HUNDRED_PERCENT + TRESHOLD_PERCENT_LEVEL) * resolution {
                        //  if the period is longer it suggest it is the last period
                        //  more detailed \ build up: if zero is last encoded: this is error. zero
                        //  would cause exact period and additional random one after that.
                        match wire_state_periods.is_empty() {
                            true => {
                                err_output
                                    .push(DataDecodingDebug {
                                        duration: pop_period,
                                        event: DecodingDebugEvent::ComplementaryLongPulseFinish,
                                        count: count,
                                    })
                                    .unwrap();
                                //  log::info!(
                                //      "per for in_half[{count}] = {} => X (last long pulse)",
                                //      pop_period.as_ticks()
                                //  );
                                break;
                            }
                            false => {
                                log::error!(
                                    "Long Complementary half-period detected but the vector has still: {} elements",
                                    wire_state_periods.len()
                                );
                                err_output
                                    .push(DataDecodingDebug {
                                        duration: pop_period,
                                        event: DecodingDebugEvent::ComplementaryLongPulseError,
                                        count: count,
                                    })
                                    .unwrap();
                                return Err((CodingError::FinishExpectedError, err_output));
                            }
                        }
                    } else if HUNDRED_PERCENT * pop_period < (HUNDRED_PERCENT - TRESHOLD_PERCENT_LEVEL) * resolution {
                        err_output
                            .push(DataDecodingDebug {
                                duration: pop_period,
                                event: DecodingDebugEvent::ComplementaryShortPulse,
                                count: count,
                            })
                            .unwrap();
                        //  log::info!(
                        //      "per for in_half[{count}] = {} => X (short pulse)",
                        //      pop_period.as_ticks()
                        //  );
                        //  because we expect at least a finished cycle
                        return Err((CodingError::PeriodEncodingError, err_output));
                    } else {
                        //  else: the length is withing range -> push & continue
                        output.push(output_candidate).unwrap(); //  handle error
                        err_output
                            .push(DataDecodingDebug {
                                duration: pop_period,
                                event: DecodingDebugEvent::ComplementaryPulseOk,
                                count: count,
                            })
                            .unwrap();
                        //  log::info!(
                        //      "per for in_half[{count}] = {} => '{output_candidate}'",
                        //      pop_period.as_ticks()
                        //  );
                    }
                }
                _ => {
                    //  End of the capture as there is no more elements?
                    break;
                }
            }
        }
        //  Check if the period is of value around two times of the resolution:
        //  which also means that we already land in the middle of the next bit
        else if (HUNDRED_PERCENT * pop_period > 2 * (HUNDRED_PERCENT - TRESHOLD_PERCENT_LEVEL) * resolution)
            && (HUNDRED_PERCENT * pop_period < 2 * (HUNDRED_PERCENT + TRESHOLD_PERCENT_LEVEL) * resolution)
        {
            //          =============================================
            //            b-----e   begin/end for the period
            //          __|'''''|__ manchester '10'
            //          ''|_____|'' manchester '01'
            //          aaaaa|bbbbb a - previous, b - current | loop iteration
            //          =============================================
            let (replace_level, new_level) = match level {
                InitLevel::Low => {
                    (InitLevel::High, false) //  a zero after one
                }
                InitLevel::High => {
                    (InitLevel::Low, true) //  a one after zero
                }
            };
            level = replace_level;
            //  log::info!("per for in_full[{count}] = {} (double pulse)", pop_period.as_ticks());
            err_output
                .push(DataDecodingDebug {
                    duration: pop_period,
                    event: DecodingDebugEvent::DoublePulse,
                    count: count,
                })
                .unwrap();
            match output.push(new_level) {
                Err(_returned_element) => {
                    //  returns back elemeent that was not possible
                    return Err((CodingError::NoSpaceAvailable, err_output));
                }
                _ => { // correct
                }
            }
        } else {
            //  TODO: check for number of elements in the queue that left
            if wire_state_periods.is_empty() {
                break;
            } else {
                //  Half way through the bit there is non-conforming period length:
                return Err((CodingError::PeriodEncodingError, err_output));
            }
        }
        count += 1;
    }
    //  log::info!("Requested DEBUG manchester decoding");
    //  DataDecodingDebug::print_debug_info(err_output);
    Ok(output)
}

pub struct ManchesterIteratorAdapter<I: Iterator<Item = bool>> {
    iterator: I,
    next_bit: Option<bool>,
}

impl<I: Iterator<Item = bool>> ManchesterIteratorAdapter<I> {
    pub fn new(iterator_arg: I) -> Self {
        Self {
            iterator: iterator_arg,
            next_bit: None,
        }
    }
}

impl<I: Iterator<Item = bool>> Iterator for ManchesterIteratorAdapter<I> {
    type Item = bool;
    fn next(&mut self) -> Option<Self::Item> {
        let ret = match self.next_bit {
            None => match self.iterator.next() {
                None => None, //  depleted
                Some(bit) => {
                    self.next_bit = Some(bit);
                    Some(!bit)
                }
            },
            Some(bit) => {
                self.next_bit = None;
                Some(bit)
            }
        };
        ret
    }
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
