#include "protocol-defs.h"
#include <AUnitVerbose.h>

//#define RUN_TESTS

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
const int kLedChA[] = { kLedChA1Pin, kLedChA2Pin, kLedChA3Pin };

// These three pins have LEDs 7-12 (in charlieplexing configuration)
static constexpr int kLedChB1Pin = PB12;
static constexpr int kLedChB2Pin = PA15;
static constexpr int kLedChB3Pin = PA8;
const int kLedChB[] = { kLedChB1Pin, kLedChB2Pin, kLedChB3Pin };

// These two pins have LEDs 15-16 (in charlieplexing configuration)
static constexpr int kLedChC1Pin = PB14;
static constexpr int kLedChC2Pin = PB13;
// This pin goes to nowhere.
static constexpr int kDbg1Pin = PB15;
const int kLedChC[] = { kLedChC1Pin, kLedChC2Pin, kDbg1Pin };

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
  { 0, 1, 2, 0 },
  { 1, 0, 2, 1 },
  { 0, 2, 1, 2 },
  { 2, 0, 1, 3 },
  { 1, 2, 0, 4 },
  { 2, 1, 0, 5 },
};

const CharlieProgram kCharlieProgramB[6] = {
  { 0, 1, 2, 0 },
  { 1, 0, 2, 1 },
  { 0, 2, 1, 5 },
  { 2, 0, 1, 4 },
  { 1, 2, 0, 3 },
  { 2, 1, 0, 2 },
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
// Implement this function for handling events coming from the global bus.
extern void OnGlobalEvent(uint64_t event, uint16_t src);

// Call this function to send a broadcast event to the global bus.
extern bool SendEvent(uint64_t event_id);

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

void MenuBlink(int counter);

// This function will be called 4 times per second by the timer.
void Timer4Hz() {
  static int ctr = 0;
  ++ctr;
  MenuBlink(ctr);
}


// ==================== Local bus API ==================

// Call this function once from setup().
extern void LocalBusSetup();
// Call this function once from loop().
extern void LocalBusLoop();

// Sets the signal active or inactive on a localbus direction.
extern void LocalBusSignal(Direction dir, bool active);
// Returns true if the signal in a given direction is active (either from us or from the neighbor).
extern bool LocalBusIsActive(Direction dir);

#include "local-bus.h"

// ========================================================

#include "protocol-engine.h"
#include "protocol-engine-default-plugin.h"

ProtocolEngine engine;

bool state[6][6];
bool next_state[6][6];

struct Delta {
  int dr;
  int dc;
};

static constexpr Delta neighbors[8] = {
  { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -1 }, { 0, 1 }, { 1, -1 }, { 1, 0 }, { 1, 1 }
};

void MenuBlink(int ctr) {
  static bool blinking = false;

  if (engine.ShouldBlinkMenu()) {
    leds[0] = leds[3] = leds[12] = leds[15] = (ctr % 2);
    blinking = true;
  } else if (blinking) {
    blinking = false;
    leds[0] = state[1][1];
    leds[3] = state[1][4];
    leds[12] = state[4][1];
    leds[15] = state[4][4];
  }
}

void SendStateReport() {
  using Defs = ::ProtocolDefs;

  uint16_t report = 0;
  for (int r = 1; r <= 4; ++r) {
    for (int c = 1; c <= 4; ++c) {
      if (state[r][c]) {
        report |= 1u << ((r - 1) * 4 + (c - 1));
      }
    }
  }
  SendEvent(Defs::CreateEvent(Defs::kStateReport, engine.GetX(), engine.GetY(), report));
}

void OnGlobalEvent(uint64_t ev, uint16_t src) {
  using Defs = ::ProtocolDefs;
  //SerialUSB.printf("event arrived: %08lx%08lx\n", ev >> 32, ev & 0xfffffffful);
  engine.OnGlobalEvent(ev, src);
  if (!Defs::IsProtocolEvent(ev)) return;
  auto cmd = Defs::GetCommand(ev);
  if (cmd == Defs::kGlobalCmd) {
    Defs::GlobalCommand gcmd = (Defs::GlobalCommand)Defs::GetArg(ev);
    switch (gcmd) {
      case Defs::kEvolveAndReport:
        {
          memset(next_state, 0, sizeof(next_state));
          for (int r = 1; r <= 4; ++r) {
            for (int c = 1; c <= 4; ++c) {
              unsigned count = 0;
              for (auto d : neighbors) {
                if (state[r + d.dr][c + d.dc]) ++count;
              }
              next_state[r][c] = (count == 3 || (count == 2 && state[r][c]));
              leds[(r - 1) * 4 + (c - 1)] = next_state[r][c];
            }
          }
          static_assert(sizeof(state) == 36, " error size ");
          memcpy(state, next_state, sizeof(state));
          SendStateReport();
          break;
        }
      case Defs::kSetStateRandom:
        {
          memset(next_state, 0, sizeof(next_state));
          uint16_t report = 0;

          for (int r = 1; r <= 4; ++r) {
            for (int c = 1; c <= 4; ++c) {
              next_state[r][c] = (rand() % 100) < 40;
              leds[(r - 1) * 4 + (c - 1)] = next_state[r][c];
              if (next_state[r][c]) {
                report |= 1u << ((r - 1) * 4 + (c - 1));
              }
            }
          }
          memcpy(state, next_state, sizeof(state));
          SendEvent(Defs::CreateEvent(Defs::kStateReport, engine.GetX(), engine.GetY(), report));
          break;
        }
      case Defs::kClearState:
        {
          memset(state, 0, sizeof(state));
          memset(next_state, 0, sizeof(next_state));
          uint16_t report = 0;
          SendEvent(Defs::CreateEvent(Defs::kStateReport, engine.GetX(), engine.GetY(), report));
          break;
        }
      default: break;
    }
  } else if (cmd == Defs::kStateReport) {
    uint8_t x = Defs::GetX(ev);
    uint8_t y = Defs::GetY(ev);
    for (unsigned idir = 0; idir < engine.neighbors_.size(); ++idir) {
      const auto& n = engine.neighbors_[idir];
      if (n.neigh_x != x || n.neigh_y != y) continue;
      // We have to import this neighbor data into the current state.
      if (idir >= 4) {
        // import 1 bit
        bool value = Defs::GetArg(ev) & (1u << n.pixel_offset);
        switch ((Direction)idir) {
          case kNorthWest:
            state[0][0] = value;
            break;
          case kNorthEast:
            state[0][5] = value;
            break;
          case kSouthEast:
            state[5][5] = value;
            break;
          case kSouthWest:
            state[5][0] = value;
            break;
        }
      } else {
        // we have to import 4 bits.
        auto arg = Defs::GetArg(ev);
        const Defs::Segment& seg = Defs::kEdgeSegments[n.neigh_dir];
        // starting row, col, delta row, col. This is COUNTERCLOCKWISE.
        int r, c, dr, dc;
        switch ((Direction)idir) {
          case kNorth:
            r = 0;
            c = 4;
            dr = 0;
            dc = -1;
            break;
          case kSouth:
            r = 5;
            c = 1;
            dr = 0;
            dc = 1;
            break;
          case kEast:
            r = 4;
            c = 5;
            dr = -1;
            dc = 0;
            break;
          case kWest:
            r = 1;
            c = 0;
            dr = 1;
            dc = 0;
            break;
        }
        unsigned bit_num = seg.bit_num;
        for (unsigned i = 0; i < 4; ++i) {
          state[r][c] = arg & (1u << bit_num);
          r += dr;
          c += dc;
          bit_num += seg.bit_stride;
        }
      }
    }
  } else if (cmd == Defs::kStateSet || cmd == Defs::kStateOr) {
    uint8_t x = Defs::GetX(ev);
    uint8_t y = Defs::GetY(ev);
    if (x != engine.GetX() || y != engine.GetY()) {
      return;
    }
    auto arg = Defs::GetArg(ev);
    for (unsigned r = 0; r < 4; ++r) {
      unsigned bit = Defs::kRowSegments[r].bit_num;
      unsigned stride = Defs::kRowSegments[r].bit_stride;
      for (unsigned c = 0; c < 4; ++c, bit += stride) {
        bool value = (arg & (1u << bit)) != 0;
        if (value) {
          state[r + 1][c + 1] = true;
          leds[r * 4 + c] = true;
        } else if (cmd == Defs::kStateSet) {
          state[r + 1][c + 1] = false;
          leds[r * 4 + c] = false;
        }
      }
    }
    SendStateReport();
    return;
  }
}

// Bit mask, bits 0..3 are rows active, bits 4..7 are columns active.
unsigned last_button_press = 0;
// timestamp when last the button count was incremented
unsigned last_button_millis = -1;
// how many millis are the buttons stable
unsigned last_button_count = 0;

void ReportButton() {
  using Defs = ::ProtocolDefs;
  unsigned num_r = (last_button_press & 1) + ((last_button_press >> 1) & 1) + ((last_button_press >> 2) & 1) + ((last_button_press >> 3) & 1);
  unsigned num_c = ((last_button_press >> 4) & 1) + ((last_button_press >> 5) & 1) + ((last_button_press >> 6) & 1) + ((last_button_press >> 7) & 1);
  int r = __builtin_ctz(last_button_press | 0x100);
  int c = __builtin_ctz((last_button_press >> 4) | 0x100);
  if (num_r == 1 && num_c == 1 && r < 4 && c < 4) {
    int btn = r * 4 + c;  // 0..15
    SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), btn));
  } else if (last_button_press == 0b0101) {
    // Menu button is row 0 and row 2 together.
    SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), 16));
  }
}

static constexpr unsigned kQuiescentButtonMillis = 20;

void ProcessButtons() {
  unsigned current_press = 0;
  for (unsigned rc = 0; rc < 4; ++rc) {
    if (btn_row_active[rc]) current_press |= (0x01u << rc);
    if (btn_col_active[rc]) current_press |= (0x10u << rc);
  }
  auto m = millis();
  if (current_press != last_button_press) {
    last_button_press = current_press;
    last_button_count = 0;
    last_button_millis = m;
  }
  if (m > last_button_millis) {
    ++last_button_count;
    last_button_millis = m;
    if (last_button_count == kQuiescentButtonMillis) {
      ReportButton();
    }
  }
}

void setup() {
  srand(nmranet_nodeid());
  // put your setup code here, to run once:
  TimerSetup();
  TouchSetup();
  LocalBusSetup();
  GlobalBusSetup();
  engine.Setup(&global_impl);

  pinMode(kLed13Pin, OUTPUT);
  pinMode(kLed14Pin, OUTPUT);
  memset(state, 0, sizeof(state));

#ifdef RUN_TESTS
  while (!Serial)
    ;
  //delay(2000);
  Serial.println("Hello!");
#endif
}

class StateBitTest : public aunit::TestOnce {
protected:
  // Picks state [r][c], asserts that it is true and clears it.
  void AssertAndClearState(int r, int c) {
    assertTrue(state[r][c]);
    state[r][c] = false;
  }
  // Asserts that all of the sstate is empty.
  void AssertEmptyState() {
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        if (state[r][c]) {
          Serial.printf("state[%d][%d] is not empty\n", r, c);
          assertFalse(state[r][c]);
        }
      }
    }
  }
};

testF(StateBitTest, NeighborReportTest) {
  using Defs = ::ProtocolDefs;
  memset(state, 0, sizeof(state));
  engine.SetupTest();
  OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, 0x85, 0x80, 1u << 12), 0);
  AssertAndClearState(0, 1);
  AssertEmptyState();
  OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, 0x85, 0x80, 1u << 14), 0);
  AssertAndClearState(0, 3);
  AssertEmptyState();
  OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, 0x85, 0x82, 1 | 4), 0);
  AssertAndClearState(5, 1);
  AssertAndClearState(5, 3);
  AssertEmptyState();
}

void loop() {
#ifdef RUN_TESTS
  aunit::TestRunner::run();
#endif
  // put your main code here, to run repeatedly:
  TouchLoop();
  TimerLoop();
  GlobalBusLoop();
  LocalBusLoop();
  engine.Loop();

  ProcessButtons();

  digitalWrite(kLed13Pin, !leds[13 - 1]);
  digitalWrite(kLed14Pin, !leds[14 - 1]);
}
