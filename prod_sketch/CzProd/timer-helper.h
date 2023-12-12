#include "stm32f0xx_hal.h"


extern void Timer6000Hz();
extern void Timer4Hz();

#define CHARLIE_TIMER TIM14

HardwareTimer tim_charlie(CHARLIE_TIMER);

void TimerSetup() {
  tim_charlie.attachInterrupt(Timer6000Hz);
  tim_charlie.setMode(1, TIMER_DISABLED);
  tim_charlie.setOverflow(6000, HERTZ_FORMAT);
  tim_charlie.resume();
}

void TimerLoop() {
  static constexpr unsigned kTicksPerSec = 1000;
  static constexpr unsigned kTickStride = kTicksPerSec / 4;
  static unsigned next_tick = 0;
  if (HAL_GetTick() >= next_tick) {
    Timer4Hz();
    next_tick += kTickStride;
  }
}