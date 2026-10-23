#include "headers/sharedClock.h"

#include "stm32h7xx.h"   // STM32H7 registers (TIM5), from the Arduino mbed core

namespace shared {

uint32_t nowUs() {
  return TIM5->CNT;
}

}  // namespace shared
