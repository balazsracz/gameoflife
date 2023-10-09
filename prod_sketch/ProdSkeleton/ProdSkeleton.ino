
// ================= API for touch sensor (input buttons) ===================

// Call this function once from setup().
extern void TouchSetup();
// Call this function once from loop().
extern void TouchLoop();

// These variables are set to true when a given row in the key matrix is active (has a finger somewhere).
// Note: the MENU button has row 0 + row 2 and no columns active. All other buttons have exactly one row and one
// column.
extern bool btn_row_active[4];
// These variables are set to true when a given column in the key matrix is active (has a finger somewhere).
extern bool btn_col_active[4];

// Implementation.
#include "touch-sensor.h"

// ================= definitions for output LEDs. ====================

// These two pins have an LED on them. The LED will light when you write LOW to the respective pin.
static constexpr int kLed13Pin = PF0;
static constexpr int kLed14Pin = PF1;

// These three pins have LEDs 1-6 (in charlieplexing configuration)
static constexpr int kLedChA1Pin = PC14;
static constexpr int kLedChA2Pin = PC15;
static constexpr int kLedChA3Pin = PC13;
const int kLedChA[] = { 0, kLedChA1Pin, kLedChA2Pin, kLedChA3Pin };

// These three pins have LEDs 7-12 (in charlieplexing configuration)
static constexpr int kLedChB1Pin = PB12;
static constexpr int kLedChB2Pin = PA15;
static constexpr int kLedChB3Pin = PA8;
const int kLedChB[] = { 0, kLedChB1Pin, kLedChB2Pin, kLedChB3Pin };

// These two pins have LEDs 15-16 (in charlieplexing configuration)
static constexpr int kLedChC1Pin = PB14;
static constexpr int kLedChC2Pin = PB13;
static constexpr int kDbg1Pin = PB15;
const int kLedChC[] = { 0, kLedChC1Pin, kLedChC2Pin, kDbg1Pin };

bool leds[24] = { 0 };

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

const CharlieProgram kCharlieProgramA[6] = {
  { 1, 2, 3, 0 },
  { 2, 1, 3, 1 },
  { 1, 3, 2, 2 },
  { 3, 1, 2, 3 },
  { 2, 3, 1, 4 },
  { 3, 2, 1, 5 },
};

const CharlieProgram kCharlieProgramB[6] = {
  { 1, 2, 3, 0 },
  { 2, 1, 3, 1 },
  { 1, 3, 2, 5 },
  { 3, 1, 2, 4 },
  { 2, 3, 1, 3 },
  { 3, 2, 1, 2 },
};


void CharlieApply(const CharlieProgram* pgm, const int* pins, bool* data, int idx) {
  if (idx >= 6) return;
  const auto& p = pgm[idx];
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

// ================== Global bus API ======================
// Implement this function for handling events coming from the local bus.
extern void OnGlobalEvent(uint64_t event);

// Call this function to send a broadcast event to the global bus.
extern void SendEvent(uint64_t event_id);

// Call this function once from setup().
extern void GlobalBusSetup();

// Call this function from the loop() handler.
extern void GlobalBusLoop();

#include "global-bus.h"

// ========================================================

// ================== API for timers ======================

// Call this function once from setup().
extern void TimerSetup();
// Call this function once from loop().
extern void TimerLoop();
// This function will be called 6000 times per second by the timer.
extern void Timer6000Hz();
// This function will be called 4 times per second by the timer.
extern void Timer4Hz();


// Implementation.
#include "timer-helper.h"

// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
  static unsigned state = 0;
  ++state;
  if (state >= 6) state = 0;
  CharlieApply(kCharlieProgramA, kLedChA, &leds[0], state);
  CharlieApply(kCharlieProgramB, kLedChB, &leds[6], state);
  CharlieApply(kCharlieProgramA, kLedChC, &leds[14], state);
}
// This function will be called 4 times per second by the timer.
void Timer4Hz() {
  static int ctr = 0;
  ++ctr;
  memset(leds, 0, sizeof(leds));
  leds[ctr % 16] = 1;
  SerialUSB.printf("%08x %08x %d conv_count=%d next_conv_tick=%d\n", *((uint32_t*)btn_row_active),
                   *((uint32_t*)btn_col_active), HAL_GetTick(), conv_count, next_conv_tick);
  if ((btn_col_active[1] || btn_col_active[2]) && (btn_row_active[1] || btn_row_active[2])) {
    uint64_t ev = kEventPrefix;
    if (btn_col_active[1]) ev += 6;
    else if (btn_col_active[2]) ev += 7;
    if (btn_row_active[2]) ev |= 4;
    ev -= 1;
    SendEvent(ev);
    SerialUSB.printf("event sent: %08lx%08lx\n", ev >> 32, ev & 0xfffffffful);
  }
}


// ==================== Local bus API ==================

// Call this function once from setup().
extern void LocalBusSetup();
// Call this function once from loop().
extern void LocalBusLoop();

enum Direction : uint8_t {
  kNorth,
  kEast,
  kSouth,
  kWest
};

// Sets the signal active or inactive on a localbus direction.
extern void LocalBusSignal(Direction dir, bool active);
// Returns true if the signal in a given direction is active (either from us or from the neighbor).
extern bool LocalBusIsActive(Direction dir);

#include "local-bus.h"

// ========================================================

#include "protocol-engine.h"

ProtocolEngine engine;

void OnGlobalEvent(uint64_t ev, uint16_t src) {
  engine.OnGlobalEvent(ev, src);
  SerialUSB.printf("event arrived: %08lx%08lx\n", ev >> 32, ev & 0xfffffffful);
  leds[ev & 15] = true;
}

void setup() {
  // put your setup code here, to run once:
  TimerSetup();
  TouchSetup();
  LocalBusSetup();
  GlobalBusSetup();
  engine.Setup(&SendEvent);

  pinMode(kLed13Pin, OUTPUT);
  pinMode(kLed14Pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  TouchLoop();
  TimerLoop();
  GlobalBusLoop();
  engine.Loop();

  digitalWrite(kLed13Pin, !leds[13 - 1]);
  digitalWrite(kLed14Pin, !leds[14 - 1]);

  LocalBusSignal(kNorth, btn_row_active[0] && (btn_col_active[1] || btn_col_active[2]));
  LocalBusSignal(kSouth, btn_row_active[3] && (btn_col_active[1] || btn_col_active[2]));
  LocalBusSignal(kWest, btn_col_active[0] && (btn_row_active[1] || btn_row_active[2]));
  LocalBusSignal(kEast, btn_col_active[3] && (btn_row_active[1] || btn_row_active[2]));

  leds[4] = LocalBusIsActive(kWest);
  leds[11] = LocalBusIsActive(kEast);
  leds[1] = LocalBusIsActive(kNorth);
  leds[14] = LocalBusIsActive(kSouth);
}
