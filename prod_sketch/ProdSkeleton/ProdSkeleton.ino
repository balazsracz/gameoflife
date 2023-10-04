#include "touch-sensor.h"
#include "timer-helper.h"

// These variables are set to true when a given row in the key matrix is active (has a finger somewhere).
// Note: the MENU button has row 1 + row 3 and no columns active. All other buttons have exactly one row and one
// column.
extern bool btn_row_active[4];
// These variables are set to true when a given column in the key matrix is active (has a finger somewhere).
extern bool btn_col_active[4];

static constexpr int kLed13Pin = PF0;
static constexpr int kLed14Pin = PF1;

// These three pins have LEDs 1-6 (in charlieplexing configuration)
static constexpr int kLedChA1Pin = PC14;
static constexpr int kLedChA2Pin = PC15;
static constexpr int kLedChA3Pin = PC13;
const int kLedChA[] = {kLedChA1Pin, kLedChA2Pin, kLedChA3Pin};

// These three pins have LEDs 7-12 (in charlieplexing configuration)
static constexpr int kLedChB1Pin = PA15;
static constexpr int kLedChB2Pin = PB12;
static constexpr int kLedChB3Pin = PA8;
const int kLedChB[] = {kLedChB1Pin, kLedChB2Pin, kLedChB3Pin};

// These two pins have LEDs 15-16 (in charlieplexing configuration)
static constexpr int kLedChC1Pin = PB13;
static constexpr int kLedChC2Pin = PB14;
static constexpr int kDbg1Pin = PB5;
const int kLedChC[] = {kLedChC1Pin, kLedChC2Pin, kDbg1Pin};

bool leds[24] = {0};

//  ================== SNIP =============== charlieplex handler vvv ==========

struct CharlieProgram {
  // Which pin to set to positive voltage (high).
  int pin_hi_;
  // Which pin to set to zero voltage (low).
  int pin_lo_;
  // Which pin to set to input.
  int pin_z_;
  // Which led's data to output.
  int led_no_;
};

const CharlieProgram kCharlieProgram[6] = {
  { 1, 2, 3, 0 },
  { 2, 1, 3, 1 },
  { 1, 3, 2, 2 },
  { 3, 1, 2, 3 },
  { 2, 3, 1, 4 },
  { 3, 2, 1, 5 },
};

void CharlieApply(const int* pins, bool* data, int idx) {
  if (idx >= 6) return;
  const auto& p = kCharlieProgram[idx];
  pinMode(pins[p.pin_z_], INPUT);
  pinMode(pins[p.pin_lo_], INPUT);
  pinMode(pins[p.pin_hi_], INPUT);
  pinMode(pins[p.pin_lo_], OUTPUT);
  digitalWrite(pins[p.pin_lo_], LOW);
  digitalWrite(pins[p.pin_hi_], LOW);
  pinMode(pins[p.pin_hi_], OUTPUT);
  digitalWrite(pins[p.pin_hi_], data[p.led_no_] ? HIGH : LOW);
}

// ================= ^^^^ charlieplex handler ==============


// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
  static unsigned state = 0;
  ++state;
  if (state >= 6) state = 0;
  CharlieApply(kLedChA, leds + 0, state);
  CharlieApply(kLedChB, leds + 6, state);
  CharlieApply(kLedChC, leds + 14, state);
}
// This function will be called 4 times per second by the timer.
void Timer4Hz() {
  static int ctr = 0;
  ++ctr;
  leds[13-1] = ctr & 1;
  leds[14-1] = !(ctr & 1);
  SerialUSB.printf("%08x %08x %d %d\n", *((uint32_t*)btn_row_active), *((uint32_t*)btn_col_active), HAL_GetTick(), conv_count);
}



void setup() {
  // put your setup code here, to run once:
  TouchSetup();
  TimerSetup();

  pinMode(kLed13Pin, OUTPUT);
  pinMode(kLed14Pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  TouchLoop();
  TimerLoop();

  digitalWrite(kLed13Pin, leds[13-1]);
  digitalWrite(kLed14Pin, leds[14-1]);

  for (unsigned i = 0; i < 16; ++i) {
    leds[i] = btn_row_active[i/4] || btn_col_active[i%4];
  }
}
