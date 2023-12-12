/** \copyright
 * Copyright (c) 2023, Corrado Zoccolo, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bitset>
#include "protocol-defs.h"
#include <AUnitVerbose.h>

// #define RUN_TESTS

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

int identity(int i) {
  return i;
}
int reverse_from_2(int i) {
  return i > 1 ? 7 - i : i;
}

void charlie6(bool* seq, int frame, int pin3, int pin1, int pin2, int (*t)(int)) {
  if (seq[t(0)] && frame == 0) {
    pinMode(pin3, INPUT);  // Pin 1
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, HIGH);
    pinMode(pin2, OUTPUT);
    digitalWrite(pin2, LOW);
  } else if (seq[t(1)] && frame == 1) {
    pinMode(pin3, INPUT);  // Pin 2
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, LOW);
    pinMode(pin2, OUTPUT);
    digitalWrite(pin2, HIGH);
  } else if (seq[t(2)] && frame == 2) {
    pinMode(pin2, INPUT);  // Pin 3
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, HIGH);
    pinMode(pin3, OUTPUT);
    digitalWrite(pin3, LOW);
  } else if (seq[t(3)] && frame == 3) {
    pinMode(pin2, INPUT);  // Pin 4
    pinMode(pin1, OUTPUT);
    digitalWrite(pin1, LOW);
    pinMode(pin3, OUTPUT);
    digitalWrite(pin3, HIGH);
  } else if (seq[t(4)] && frame == 4) {
    pinMode(pin1, INPUT);  // Pin 5
    pinMode(pin2, OUTPUT);
    digitalWrite(pin2, HIGH);
    pinMode(pin3, OUTPUT);
    digitalWrite(pin3, LOW);
  } else if (seq[t(5)] && frame == 5) {
    pinMode(pin1, INPUT);  // Pin 6
    pinMode(pin2, OUTPUT);
    digitalWrite(pin2, LOW);
    pinMode(pin3, OUTPUT);
    digitalWrite(pin3, HIGH);
  } else {
    pinMode(pin1, INPUT);
    pinMode(pin2, INPUT);
    pinMode(pin3, INPUT);
  }
}

// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
  static unsigned cnt = 0;
  if (cnt == 6 << 16) cnt = 0;
  digitalWrite(kLed13Pin, !leds[13 - 1]);
  digitalWrite(kLed14Pin, !leds[14 - 1]);
  if (leds[15 - 1] && cnt % 6 == 0) {
    digitalWrite(kLedChC1Pin, HIGH);
    digitalWrite(kLedChC2Pin, LOW);
  }
  if (leds[16 - 1] && cnt % 6 == 1) {
    digitalWrite(kLedChC2Pin, HIGH);
    digitalWrite(kLedChC1Pin, LOW);
  }
  if (cnt % 6 > 1) {
    digitalWrite(kLedChC2Pin, HIGH);
    digitalWrite(kLedChC1Pin, HIGH);
  }
  charlie6(&leds[0], cnt % 6, kLedChA3Pin, kLedChA1Pin, kLedChA2Pin, identity);
  charlie6(&leds[6], cnt % 6, kLedChB3Pin, kLedChB1Pin, kLedChB2Pin, reverse_from_2);
  ++cnt;
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

bool is_button_pressed(int* button_out) {
  int x_coord = -1;
  int y_coord = -1;
  int rows_active = 0;
  int cols_active = 0;

  for (int i = 0; i < 4; ++i) {
    if (btn_col_active[i]) {
      ++cols_active;
      x_coord = i;
    }
    if (btn_row_active[i]) {
      ++rows_active;
      y_coord = i;
    }
  }
  if (cols_active == 0 && rows_active == 2 && btn_row_active[0] && btn_row_active[2]) {
    if (button_out) *button_out = -1;
    return true;
  }
  if (cols_active == 1 && rows_active == 1) {
    int button = x_coord + y_coord * 4;
    if (button_out) *button_out = button;
    return true;
  }
  return false;
}

// This function will be called 4 times per second by the timer.
void HandleButtons() {
  using Defs = ::ProtocolDefs;
  static int prev_button = -2;
  int button;
  if (!is_button_pressed(&button)) {
    prev_button = -2;
    return;
  }
  if (button != prev_button) {
    prev_button = button;
    if (button == -1) {
      Serial.printf("Menu Button!\n");
      SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), 16));
    } else {
      Serial.printf("Button #%d!\n", button);
      SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), button));
    }
  }
}

class Matrix {
  static constexpr unsigned kHighBit = 1 << 15;
public:
  Matrix() = default;

  // x and y can vary in [-1, 4], so 6 x 6
  bool get(int x, int y) const {
    return state[y + 1][x + 1];
  }

  void set(int x, int y, bool v) {
    if (x < -1 || x > 4) return;
    if (y < -1 || y > 4) return;
    state[y + 1][x + 1] = v;
  }

  void set_all(unsigned values) {
    set_tile(0, 0, values);
  }

  // cx and cy are in [-1, 1].
  void set_tile(int cx, int cy, unsigned values) {
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        set(x + cx * 4, y + cy * 4, values & 1);
        values >>= 1;
      }
    }
  }

  void or_all(unsigned values) {
    set_all(values | to_bits());
  }

  unsigned to_bits() const {
    unsigned ret = 0;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        ret >>= 1;
        if (get(x, y)) ret |= kHighBit;
      }
    }
    return ret;
  }

  void blit_to_fb(bool* fb) const {
    for (int y = 0; y < 4; ++y)
      for (int x = 0; x < 4; ++x)
        *fb++ = get(x, y);
  }

  int count_neighbors(int x, int y) {
    return //
      get(x - 1, y - 1) + get(x, y - 1) + get(x + 1, y - 1) + // 
      get(x - 1, y)     +        0      + get(x + 1, y) + //
      get(x - 1, y + 1) + get(x, y + 1) + get(x + 1, y + 1);
  }

  unsigned evolve() {
    unsigned ret = 0;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        ret >>= 1;
        int n = count_neighbors(x, y);
        if (get(x, y)) {  // alive cell wants 2 or 3 neighbors to survive.
          if ((2 <= n) && (n <= 3)) ret |= kHighBit;
        } else {  // dead cell spawns at exactly 3 neighbors.
          if (n == 3) ret |= kHighBit;
        }
      }
    }
    set_all(ret);
    return ret;
  }

  void clear_shadow() {
    state[0] = {};
    state[5] = {};
    set_tile(-1, 0, 0);
    set_tile(1, 0, 0);
  }

  const char* dumpall() {
    static char buf[50];
    char* out = buf;
    for (auto& row : state) {
      for (size_t i = 0; i < row.size(); ++i) {
        bool bit = row[i];
        *out++ = '0' + bit;
      }
      *out++ = '\n';
    }
    *out = 0;
    return buf;
  }

private:
  std::array<std::bitset<6>, 6> state;
};

Matrix my_state;

void ReportState() {
  using Defs = ::ProtocolDefs;
  SendEvent(Defs::CreateEvent(Defs::kStateReport, engine.GetX(), engine.GetY(), my_state.to_bits()));
}

void OnGlobalEvent(uint64_t ev, uint16_t src) {
  using Defs = ::ProtocolDefs;
  engine.OnGlobalEvent(ev, src);
  if (!Defs::IsProtocolEvent(ev)) return;
  // more stuff needs to come here...
  switch (Defs::GetCommand(ev)) {
    case Defs::kGlobalCmd:
      {
        switch (Defs::GetArg(ev)) {
          case Defs::kSetStateRandom:
            {
              my_state = Matrix();
              unsigned val = 0;
              for (int i = 0; i < 16; ++i) {
                val <<= 1;
                if (rand() < RAND_MAX / 100 * 30) val |= 1;
              }
              my_state.set_all(val);
              break;
            }
          case Defs::kClearState:
            my_state = Matrix();
            break;
          case Defs::kEvolveAndReport:
            my_state.evolve();
            my_state.clear_shadow();
          case Defs::kReportState: 
            ReportState();
            break;
        }
        my_state.blit_to_fb(leds);
        break;
      }
    case Defs::kStateSet:
      {
        uint8_t x = Defs::GetX(ev);
        uint8_t y = Defs::GetY(ev);
        if (x != engine.GetX() || y != engine.GetY()) {
          break;
        }
        unsigned new_state = Defs::GetArg(ev);
        my_state.set_all(new_state);
        ReportState();
        my_state.blit_to_fb(leds);
        break;
      }
    case Defs::kStateOr:
      {
        uint8_t x = Defs::GetX(ev);
        uint8_t y = Defs::GetY(ev);
        if (x != engine.GetX() || y != engine.GetY()) {
          break;
        }
        unsigned new_state = Defs::GetArg(ev);
        my_state.or_all(new_state);
        my_state.blit_to_fb(leds);
        break;
      }
    case Defs::kStateReport:
      {
        // This comes from neighbors.
        int x = Defs::GetX(ev);
        int y = Defs::GetY(ev);
        if (x == engine.GetX() && y == engine.GetY()) break;
        unsigned new_state = Defs::GetArg(ev);
        
        int dx = x - engine.GetX(), dy = y - engine.GetY();
        if (abs(dx) <= 1 && abs(dy) <= 1)
          my_state.set_tile(dx, dy, new_state);
        break;
      }
    case Defs::kButtonPressed: break;
  }
}

void MenuBlink(int ctr) {
  static bool blinking = false;

  /// @todo take into account what menu leds should be blinked.
  if (engine.GetMenuLeds() != 0) {
    leds[0] = leds[3] = leds[12] = leds[15] = (ctr % 2);
    blinking = true;
  } else if (blinking) {
    blinking = false;
    my_state.blit_to_fb(leds);
  }
}

#ifdef RUN_TESTS
// Notes so far:
// - The test requires StateSet to also StateReport, but this is not intuitive.
// - The test requires evolve() to clear the shadow states. In prod, this should not be the case.
// - There is no test for the button functionality!
// - MenuBlink is present in prod, but not in instruction.

test(SetReportClearState) {
  SerialUSB.printf("State test\n");
  using Defs = ::ProtocolDefs;
  engine.SetupTest();
  // Clear state test.
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kClearState), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  assertEqual(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, 0), LastSentEvent());
  SendEvent(0);

  // Set state test.
  OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX, engine.kTestY, 0x5467), 0);
  assertEqual(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, 0x5467), LastSentEvent());
  SendEvent(0);
  assertEqual(0ull, LastSentEvent());

  // Report state test.
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  assertEqual(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, 0x5467), LastSentEvent());
  SendEvent(0);

  // anoter clear state test.
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kClearState), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  assertEqual(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, 0), LastSentEvent());
}

test(SetRandomState) {
  SerialUSB.printf("Random test\n");
  using Defs = ::ProtocolDefs;
  engine.SetupTest();
  // Clear state test.
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kClearState), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kSetStateRandom), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  uint16_t st1 = LastSentEvent() & 0xffffu;
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kSetStateRandom), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  uint16_t st2 = LastSentEvent() & 0xffffu;
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kSetStateRandom), 0);
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
  uint16_t st3 = LastSentEvent() & 0xffffu;

  assertNotEqual(0u, (unsigned)st1);
  assertNotEqual(0u, (unsigned)st2);
  assertNotEqual(0u, (unsigned)st3);
  assertNotEqual(st1, st2);
  assertNotEqual(st2, st3);
}

String EventToString(uint64_t ev) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  String ret("0123456789012345");
  for (unsigned i = 0; i < 16; ++i) {
    ret[15 - i] = kHexDigits[ev & 0xf];
    ev >>= 4;
  }
  return ret;
}

class EvolutionTest : public aunit::TestOnce {
protected:
  void AssertEvent(uint64_t ev1, uint64_t ev2) {
    assertEqual(EventToString(ev1), EventToString(ev2));
  }

  void TestEvolution(uint16_t state_before, uint16_t state_after) {
    using Defs = ::ProtocolDefs;
    engine.SetupTest();

    SerialUSB.printf("evolution test: before %04x after %04x\n", state_before, state_after);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX, engine.kTestY, state_before), 0);
    OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kEvolveAndReport), 0);
    uint64_t last = LastSentEvent();
    uint64_t exp = Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, state_after);
    AssertEvent(exp, last);
  }

  // Tests an extended evolution with neighbor state.
  // r0..r5 are top to bottom rows, each 6 bit long. bit5 is right neighbor, 4..1 are state, 0 is left neighbor.
  // row 0 is top neighbors, row 5 is bottom neighbors. The actual test is running mirrored to how the bits are
  // written as a person.
  void TestXEvolution(uint8_t r0, uint8_t r1, uint8_t r2, uint8_t r3, uint8_t r4, uint8_t r5, uint16_t exp) {
    using Defs = ::ProtocolDefs;
    engine.SetupTest();

    // top right
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX + 1, engine.kTestY - 1, r0 & 0x20 ? 1u << 12 : 0), 0);
    // top left
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX - 1, engine.kTestY - 1, r0 & 0x01 ? 1u << 15 : 0), 0);
    // top
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY - 1, ((r0 >> 1) & 0xF) << 12), 0);
    // left
    uint16_t lstate = ((r1 & 1) << 3) | ((r2 & 1) << 7) | ((r3 & 1) << 11) | ((r4 & 1) << 15);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX - 1, engine.kTestY, lstate), 0);
    // right
    uint16_t rstate = ((r1 & 0x20) >> 5) | ((r2 & 0x20) >> 1) | ((r3 & 0x20) << 3) | ((r4 & 0x20) << 7);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX + 1, engine.kTestY, rstate), 0);
    // bottom left
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX - 1, engine.kTestY + 1, r5 & 0x01 ? 1u << 3 : 0), 0);
    // bottom right
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX + 1, engine.kTestY + 1, r5 & 0x20 ? 1u << 0 : 0), 0);
    // bottom
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY + 1, ((r5 >> 1) & 0xF) << 0), 0);
    // state
    uint16_t st = ((r1 >> 1) & 0xf) | (((r2 >> 1) & 0xf) << 4) | (((r3 >> 1) & 0xf) << 8) | (((r4 >> 1) & 0xf) << 12);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX, engine.kTestY, st), 0);

    /*
    SerialUSB.printf("\nbefore:\n");
    for (unsigned r = 0; r < 6; ++r) {
      for (unsigned c = 0; c < 6; ++c) {
        SerialUSB.printf("%d", state[r][c] ? 1 : 0);
      }
      SerialUSB.printf("\n");
    }
    */
    OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kEvolveAndReport), 0);
    uint64_t last = LastSentEvent();
    uint64_t expev = Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, exp);
    AssertEvent(expev, last);
  }
};

testF(EvolutionTest, Middle) {
  SerialUSB.printf("Middle test\n");

  using Defs = ::ProtocolDefs;
  TestEvolution(0x7000, 0x2200);
  TestEvolution(0x0700, 0x2220);
  TestEvolution(0x0070, 0x0222);
  TestEvolution(0x0007, 0x0022);
  // Same flipped
  TestEvolution(0x2220, 0x0700);
  TestEvolution(0x0222, 0x0070);
  // Add another step.
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kEvolveAndReport), 0);
  AssertEvent(Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, 0x0222), LastSentEvent());
}

testF(EvolutionTest, TopNeighbor) {
  SerialUSB.printf("Top test\n");

  TestXEvolution(0b000111, 0, 0, 0, 0, 0, 0x0001);
  TestXEvolution(0b001110, 0, 0, 0, 0, 0, 0x0002);
  TestXEvolution(0b011100, 0, 0, 0, 0, 0, 0x0004);
  TestXEvolution(0b111000, 0, 0, 0, 0, 0, 0x0008);
}

testF(EvolutionTest, BottomNeighbor) {
  SerialUSB.printf("Bottom test\n");

  TestXEvolution(0, 0, 0, 0, 0, 0b000111, 0x1000);
  TestXEvolution(0, 0, 0, 0, 0, 0b001110, 0x2000);
  TestXEvolution(0, 0, 0, 0, 0, 0b011100, 0x4000);
  TestXEvolution(0, 0, 0, 0, 0, 0b111000, 0x8000);
}

testF(EvolutionTest, LeftNeighbor) {
  SerialUSB.printf("Left test\n");

  TestXEvolution(0b100000, 0b100000, 0b100000, 0, 0, 0, 0x0008);
  TestXEvolution(0, 0b100000, 0b100000, 0b100000, 0, 0, 0x0080);
  TestXEvolution(0, 0, 0b100000, 0b100000, 0b100000, 0, 0x0800);
  TestXEvolution(0, 0, 0, 0b100000, 0b100000, 0b100000, 0x8000);
}

testF(EvolutionTest, RightNeighbor) {
  SerialUSB.printf("Right test\n");

  TestXEvolution(1, 1, 1, 0, 0, 0, 0x0001);
  TestXEvolution(0, 1, 1, 1, 0, 0, 0x0010);
  TestXEvolution(0, 0, 1, 1, 1, 0, 0x0100);
  TestXEvolution(0, 0, 0, 1, 1, 1, 0x1000);
}

#endif

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
  pinMode(kLedChC1Pin, OUTPUT);
  pinMode(kLedChC2Pin, OUTPUT);

#ifdef RUN_TESTS
  while (!Serial)
    ;
  //delay(2000);
  Serial.printf("Running tests! %d\n", 7);
#endif
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
  HandleButtons();
}