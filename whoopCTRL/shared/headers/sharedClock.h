// Shared clock - microseconds that are the SAME on the M7 and the M4.
//
// Reads TIM5, the M7's Mbed microsecond timer (32-bit, 1 MHz, running from boot; see
// mbed/targets/TARGET_STM/TARGET_STM32H7/us_ticker_data.h). Every peripheral is visible to
// both cores, so both read the same counter: a timestamp taken on one core can be
// compared directly with one taken on the other.
//
// Wraps every ~71.6 minutes: always subtract as uint32_t, (later - earlier) stays correct.

#pragma once

#include <stdint.h>

namespace shared {

uint32_t nowUs();

}  // namespace shared
