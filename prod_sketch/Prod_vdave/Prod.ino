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

bool ledFixed[16] = { 0 } ;
bool buttonState[20] = { 0 };

bool gameState[12][12] = { 0 };
bool nextState[4][4];

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


bool ShouldLedShow(int i) {
  //return leds[i] || ledFixed[i];
  return gameState[4 + (i / 4)][4 + (i % 4)];
}

// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
  static int ctr = 0;
  ++ctr;

  HandleButtons();

  digitalWrite(kLedChA1Pin, false);
  digitalWrite(kLedChA2Pin, false);
  digitalWrite(kLedChA3Pin, false);
  if (ctr % 6 == 0 && ShouldLedShow(0)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA2Pin, OUTPUT);
    pinMode(kLedChA3Pin, INPUT);
    digitalWrite(kLedChA1Pin, true);
    digitalWrite(kLedChA2Pin, false);
  } else if (ctr % 6 == 1 && ShouldLedShow(1)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA2Pin, OUTPUT);
    pinMode(kLedChA3Pin, INPUT);
    digitalWrite(kLedChA1Pin, false);
    digitalWrite(kLedChA2Pin, true);
  } else if (ctr % 6 == 2 && ShouldLedShow(2)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA2Pin, INPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    digitalWrite(kLedChA1Pin, true);
    digitalWrite(kLedChA3Pin, false);
  } else if (ctr % 6 == 3 && ShouldLedShow(3)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA2Pin, INPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    digitalWrite(kLedChA1Pin, false);
    digitalWrite(kLedChA3Pin, true);
  } else if (ctr % 6 == 4 && ShouldLedShow(4)) {
    pinMode(kLedChA1Pin, INPUT);
    pinMode(kLedChA2Pin, OUTPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    digitalWrite(kLedChA2Pin, true);
    digitalWrite(kLedChA3Pin, false);
  } else if (ctr % 6 == 5 && ShouldLedShow(5)) {
    pinMode(kLedChA1Pin, INPUT);
    pinMode(kLedChA2Pin, OUTPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    digitalWrite(kLedChA2Pin, false);
    digitalWrite(kLedChA3Pin, true);
  }

  digitalWrite(kLedChB1Pin, false);
  digitalWrite(kLedChB2Pin, false);
  digitalWrite(kLedChB3Pin, false);
  if (ctr % 6 == 0 && ShouldLedShow(6)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    pinMode(kLedChB3Pin, INPUT);
    digitalWrite(kLedChB1Pin, true);
    digitalWrite(kLedChB2Pin, false);
  } else if (ctr % 6 == 1 && ShouldLedShow(7)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    pinMode(kLedChB3Pin, INPUT);
    digitalWrite(kLedChB1Pin, false);
    digitalWrite(kLedChB2Pin, true);
  } else if (ctr % 6 == 2 && ShouldLedShow(8)) {
    pinMode(kLedChB1Pin, INPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    digitalWrite(kLedChB2Pin, false);
    digitalWrite(kLedChB3Pin, true);
  } else if (ctr % 6 == 3 && ShouldLedShow(9)) {
    pinMode(kLedChB1Pin, INPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    digitalWrite(kLedChB2Pin, true);
    digitalWrite(kLedChB3Pin, false);
  } else if (ctr % 6 == 4 && ShouldLedShow(10)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB2Pin, INPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    digitalWrite(kLedChB1Pin, false);
    digitalWrite(kLedChB3Pin, true);
  } else if (ctr % 6 == 5 && ShouldLedShow(11)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB2Pin, INPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    digitalWrite(kLedChB1Pin, true);
    digitalWrite(kLedChB3Pin, false);
  } 
  digitalWrite(kLed13Pin, !ShouldLedShow(12));
  digitalWrite(kLed14Pin, !ShouldLedShow(13));
  if (ctr % 16 == 14 && ShouldLedShow(14)) {
    digitalWrite(kLedChC1Pin, true);
    digitalWrite(kLedChC2Pin, false);
  } else if (ctr % 16 == 15 && ShouldLedShow(15)) {
    digitalWrite(kLedChC1Pin, false);
    digitalWrite(kLedChC2Pin, true);
  } else if (ctr % 16 == 0) {
    digitalWrite(kLedChC1Pin, false);
    digitalWrite(kLedChC2Pin, false);
  }
}


// This function will be called 4 times per second by the timer.
void Timer4Hz() {
  static int ctr2 = 0;
  ++ctr2;

  //for (int i = 0; i < 16; ++i) {
  //  leds[i] = ctr2 % 2 == 1;
  //}
  //for (int i = 0; i < 16; ++i) {
  //  leds[i] = ctr2 % 16 == i;
  //}
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

void HandleButtons() {
  for (int i = 0; i < 17; ++i) {
    bool state = false;
    if (i < 16) {
      int row = i / 4;
      int col = i % 4;
      state = btn_row_active[row] && btn_col_active[col];
    } else  {
      state = btn_row_active[0] && btn_row_active[2];
    }

    if (state != buttonState[i]) {
      buttonState[i] = state;
      if (state) {
        if (i < 16) {
          ledFixed[i] = !ledFixed[i];
        }
        SendEvent(::ProtocolDefs::CreateEvent(::ProtocolDefs::Command::kButtonPressed, engine.GetX(), engine.GetY(), i));
        Serial.printf("ButtonEvent: (%d, %d): %d\n", engine.GetX(), engine.GetY(), i);
      }
    }
  }
}

void OnGlobalEvent(uint64_t ev, uint16_t src) {
  using Defs = ::ProtocolDefs;
  engine.OnGlobalEvent(ev, src);
  if (!Defs::IsProtocolEvent(ev)) return;

  Defs::Command cmd = Defs::GetCommand(ev);
  if (cmd == Defs::Command::kGlobalCmd) {
    Defs::GlobalCommand gcmd = static_cast<Defs::GlobalCommand>(Defs::GetArg(ev));
    if (gcmd == Defs::GlobalCommand::kReportState) {
      uint16_t state = 0;
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          if (gameState[y + 4][x + 4]) state |= 1u << (y * 4 + x);
        }
      }
      SendEvent(Defs::CreateEvent(Defs::Command::kStateReport, engine.GetX(), engine.GetY(), state));
    } else if (gcmd == Defs::GlobalCommand::kClearState) {
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          gameState[y + 4][x + 4] = false;
        }
      }
    } else if (gcmd == Defs::GlobalCommand::kSetStateRandom) {
      // Use 30% random fill rate.
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          gameState[y + 4][x + 4] = (rand() % 100) < 30;
        }
      }
    } else if (gcmd == Defs::GlobalCommand::kEvolveAndReport) {
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          int numLiveNeighbors = 0;
          for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
              if (dy == 0 && dx == 0) continue;
              if (gameState[4 + y + dy][4 + x + dx]) {
                numLiveNeighbors++;
              }
            }
          }
          if (gameState[4 + y][4 + x]) {
            nextState[y][x] = numLiveNeighbors == 2 || numLiveNeighbors == 3;
          } else {
            nextState[y][x] = numLiveNeighbors == 3;
          }
        }
      }
      uint16_t state = 0;
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          gameState[y + 4][x + 4] = nextState[y][x];
          if (gameState[y + 4][x + 4]) state |= 1u << (y * 4 + x);
        }
      }
      SendEvent(Defs::CreateEvent(Defs::Command::kStateReport, engine.GetX(), engine.GetY(), state));
    }
  } else if (cmd == Defs::Command::kStateSet) {
    const int targetX = Defs::GetX(ev);
    const int targetY = Defs::GetY(ev);
    const int posDX = targetX - engine.GetX();
    const int posDY = targetY - engine.GetY();
    if (posDY < -1 || posDY > 1 || posDX < -1 || posDX > 1) return;
    const int offsetX = (posDX + 1) * 4;
    const int offsetY = (posDY + 1) * 4;
    const uint16_t newState = Defs::GetArg(ev);
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        gameState[y + offsetY][x + offsetX] = (newState >> (4 * y + x)) & 1;
      }
    }

    if (posDX == 0 && posDY == 0) {
      uint16_t state = 0;
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
          if (gameState[y + 4][x + 4]) state |= 1u << (y * 4 + x);
        }
      }
      SendEvent(Defs::CreateEvent(Defs::Command::kStateReport, engine.GetX(), engine.GetY(), state));
    }
  } else if (cmd == Defs::Command::kStateOr) {
    const int targetX = Defs::GetX(ev);
    const int targetY = Defs::GetY(ev);
    if (targetX != engine.GetX() || targetY != engine.GetY()) return;
    uint16_t newState = Defs::GetArg(ev);
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        gameState[y + 4][x + 4] |= (newState >> (4 * y + x)) & 1;
      }
    }
    
    uint16_t state = 0;
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        if (gameState[y + 4][x + 4]) state |= 1u << (y * 4 + x);
      }
    }
    SendEvent(Defs::CreateEvent(Defs::Command::kStateReport, engine.GetX(), engine.GetY(), state));
  } else if (cmd == Defs::Command::kStateReport) {
    const int targetX = Defs::GetX(ev);
    const int targetY = Defs::GetY(ev);
    const int posDX = targetX - engine.GetX();
    const int posDY = targetY - engine.GetY();
    if (posDX == 0 && posDY == 0) return;
    if (posDY < -1 || posDY > 1 || posDX < -1 || posDX > 1) return;
    const int offsetX = (posDX + 1) * 4;
    const int offsetY = (posDY + 1) * 4;
    const uint16_t newState = Defs::GetArg(ev);
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        gameState[y + offsetY][x + offsetX] = (newState >> (4 * y + x)) & 1;
      }
    }
  }
}

#ifdef RUN_TESTS

test(SetReportClearState) {
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
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX - 1, engine.kTestY - 1, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX, engine.kTestY - 1, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX + 1, engine.kTestY - 1, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX - 1, engine.kTestY, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX + 1, engine.kTestY, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX - 1, engine.kTestY + 1, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX, engine.kTestY + 1, 0), 0);
    OnGlobalEvent(Defs::CreateEvent(Defs::kStateSet, engine.kTestX + 1, engine.kTestY + 1, 0), 0);
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
  TestXEvolution(0b000111, 0, 0, 0, 0, 0, 0x0001);
  TestXEvolution(0b001110, 0, 0, 0, 0, 0, 0x0002);
  TestXEvolution(0b011100, 0, 0, 0, 0, 0, 0x0004);
  TestXEvolution(0b111000, 0, 0, 0, 0, 0, 0x0008);
}

testF(EvolutionTest, BottomNeighbor) {
  TestXEvolution(0, 0, 0, 0, 0, 0b000111, 0x1000);
  TestXEvolution(0, 0, 0, 0, 0, 0b001110, 0x2000);
  TestXEvolution(0, 0, 0, 0, 0, 0b011100, 0x4000);
  TestXEvolution(0, 0, 0, 0, 0, 0b111000, 0x8000);
}

testF(EvolutionTest, LeftNeighbor) {
  TestXEvolution(0b100000, 0b100000, 0b100000, 0, 0, 0, 0x0008);
  TestXEvolution(0, 0b100000, 0b100000, 0b100000, 0, 0, 0x0080);
  TestXEvolution(0, 0, 0b100000, 0b100000, 0b100000, 0, 0x0800);
  TestXEvolution(0, 0, 0, 0b100000, 0b100000, 0b100000, 0x8000);
}

testF(EvolutionTest, RightNeighbor) {
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
  Serial.printf("Hello! %d\n", 42);
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
}
