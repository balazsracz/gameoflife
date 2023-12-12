/** \copyright
 * Copyright (c) 2023, Markus Heule, Balazs Racz
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

#include "protocol-defs.h"
#include <AUnitVerbose.h>

//#define RUN_TESTS
//#define DEBUG_OUTPUT
//#define DEMO_MODE

bool debugEvolve = false;
bool debugClearState = false;

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

// ================= definitions for state machine ====================
const constexpr int kNodeLedCount = 4 * 4;
bool ourState[kNodeLedCount] = { 0 };
bool northState[kNodeLedCount] = { 0 };
bool northEastState[kNodeLedCount] = { 0 };
bool eastState[kNodeLedCount] = { 0 };
bool southEastState[kNodeLedCount] = { 0 };
bool southState[kNodeLedCount] = { 0 };
bool southWestState[kNodeLedCount] = { 0 };
bool westState[kNodeLedCount] = { 0 };
bool northWestState[kNodeLedCount] = { 0 };

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

// ================== API for timers ======================

// Call this function once from setup().
extern void TimerSetup();
// Call this function once from loop().
extern void TimerLoop();
// This function will be called 6000 times per second by the timer.
extern void Timer6000Hz();
// This function will be called 4 times per second by the timer.
extern void Timer4Hz();

// Enums used in Implementation
enum ApplyArgOperation {
  APPLY_SET = 0,
  APPLY_OR = 1
};

// Implementation.
#include "timer-helper.h"

String EventToString(uint64_t ev) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  String ret("0123456789012345");
  for (unsigned i = 0; i < 16; ++i) {
    ret[15 - i] = kHexDigits[ev & 0xf];
    ev >>= 4;
  }
  return ret;
}

bool IsMenuPressed() {
  return btn_row_active[0] && btn_row_active[2];
}

// Returns 0 for no button pressed
int GetButtonNrPressed() {
  int nr = 0;
  if (btn_col_active[0]) { nr += 1; }
  if (btn_col_active[1]) { nr += 2; }
  if (btn_col_active[2]) { nr += 3; }
  if (btn_col_active[3]) { nr += 4; }
  if (btn_row_active[0]) { nr += 0; }
  if (btn_row_active[1]) { nr += 4; }
  if (btn_row_active[2]) { nr += 8; }
  if (btn_row_active[3]) { nr += 12; }
#ifdef DEBUG_OUTPUT
  if (nr > 0) {
    Serial.printf("GetButtonNrPressed: %d\n", nr);
  }
#endif
  return nr;
}

// group should cycle through 0..5
void updateLeds(int group) {
  // Charliplexing 1-6
  digitalWrite(kLedChA1Pin, LOW);
  digitalWrite(kLedChA2Pin, LOW);
  digitalWrite(kLedChA3Pin, LOW);
  pinMode(kLedChA1Pin, INPUT);
  pinMode(kLedChA2Pin, INPUT);
  pinMode(kLedChA3Pin, INPUT);
  if ((leds[1 - 1] || leds[2 - 1]) && (group == 0 || group == 1)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA2Pin, OUTPUT);
    if (leds[1 - 1] && (group == 0)) {
      digitalWrite(kLedChA1Pin, HIGH);
    }
    if (leds[2 - 1] && (group == 1)) {
      digitalWrite(kLedChA2Pin, HIGH);
    }
  } else if ((leds[3 - 1] || leds[4 - 1]) && (group == 2 || group == 3)) {
    pinMode(kLedChA1Pin, OUTPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    if (leds[3 - 1] && (group == 2)) {
      digitalWrite(kLedChA1Pin, HIGH);
    }
    if (leds[4 - 1] && (group == 3)) {
      digitalWrite(kLedChA3Pin, HIGH);
    }
  } else if ((leds[5 - 1] || leds[6 - 1]) && (group == 4 || group == 5)) {
    pinMode(kLedChA2Pin, OUTPUT);
    pinMode(kLedChA3Pin, OUTPUT);
    if (leds[5 - 1] && (group == 4)) {
      digitalWrite(kLedChA2Pin, HIGH);
    }
    if (leds[6 - 1] && (group == 5)) {
      digitalWrite(kLedChA3Pin, HIGH);
    }
  }
  // Charlieplexing 7-12
  digitalWrite(kLedChB1Pin, LOW);
  digitalWrite(kLedChB2Pin, LOW);
  digitalWrite(kLedChB3Pin, LOW);
  pinMode(kLedChB1Pin, INPUT);
  pinMode(kLedChB2Pin, INPUT);
  pinMode(kLedChB3Pin, INPUT);
  if ((leds[7 - 1] || leds[8 - 1]) && (group == 0 || group == 1)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    if (leds[7 - 1] && (group == 0)) {
      digitalWrite(kLedChB1Pin, HIGH);
    }
    if (leds[8 - 1] && (group == 1)) {
      digitalWrite(kLedChB2Pin, HIGH);
    }
  } else if ((leds[9 - 1] || leds[10 - 1]) && (group == 2 || group == 3)) {
    // pinMode(kLedChB1Pin, INPUT);
    pinMode(kLedChB2Pin, OUTPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    if (leds[9 - 1] && (group == 2)) {
      digitalWrite(kLedChB3Pin, HIGH);
    }
    if (leds[10 - 1] && (group == 3)) {
      digitalWrite(kLedChB2Pin, HIGH);
    }
  } else if ((leds[11 - 1] || leds[12 - 1]) && (group == 4 || group == 5)) {
    pinMode(kLedChB1Pin, OUTPUT);
    pinMode(kLedChB3Pin, OUTPUT);
    if (leds[11 - 1] && (group == 4)) {
      digitalWrite(kLedChB3Pin, HIGH);
    }
    if (leds[12 - 1] && (group == 5)) {
      digitalWrite(kLedChB1Pin, HIGH);
    }
  }
  // Directly connected leds 13 & 14
  digitalWrite(kLed13Pin, (leds[13 - 1] ? LOW : HIGH));
  digitalWrite(kLed14Pin, (leds[14 - 1] ? LOW : HIGH));
  // Charlieplexing 15-16
  if (leds[15 - 1] && leds[16 - 1]) {
    digitalWrite(kLedChC1Pin, (group == 0) ? HIGH : LOW);
    digitalWrite(kLedChC2Pin, (group == 1) ? HIGH : LOW);
  } else if (leds[15 - 1]) {
    digitalWrite(kLedChC1Pin, (group == 0) ? HIGH : LOW);
  } else if (leds[16 - 1]) {
    digitalWrite(kLedChC2Pin, (group == 0) ? HIGH : LOW);
  } else {
    digitalWrite(kLedChC1Pin, LOW);
    digitalWrite(kLedChC2Pin, LOW);
  }
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

uint16_t EncodeState(bool* state) {
  uint16_t result = 0;
  for (int bitNum = 0; bitNum < kNodeLedCount; bitNum++) {
    if (state[bitNum]) {
      result |= (1u << bitNum);
    }
  }
  return result;
}

void ApplyState(bool* state, enum ApplyArgOperation op, uint16_t arg) {
  for (int bitNum = 0; bitNum < kNodeLedCount; bitNum++) {
    switch (op) {
      case APPLY_SET:
        state[bitNum] = (arg & 0x01);
      case APPLY_OR:
        if (arg & 0x01) {
          state[bitNum] = true;
        }
    }
    arg >>= 1;
  }
}

int CountLifeNeighbours(int idx) {
  int numLife = 0;
  switch (idx) {
    case 1 - 1:
      if (northState[13 - 1]) numLife++;
      if (northState[14 - 1]) numLife++;
      if (ourState[2 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[5 - 1]) numLife++;
      if (westState[8 - 1]) numLife++;
      if (westState[4 - 1]) numLife++;
      if (northWestState[16 - 1]) numLife++;
      break;
    case 2 - 1:
      if (northState[14 - 1]) numLife++;
      if (northState[15 - 1]) numLife++;
      if (ourState[3 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[5 - 1]) numLife++;
      if (ourState[1 - 1]) numLife++;
      if (northState[13 - 1]) numLife++;
      break;
    case 3 - 1:
      if (northState[15 - 1]) numLife++;
      if (northState[16 - 1]) numLife++;
      if (ourState[4 - 1]) numLife++;
      if (ourState[8 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[2 - 1]) numLife++;
      if (northState[14 - 1]) numLife++;
      break;
    case 4 - 1:
      if (northState[16 - 1]) numLife++;
      if (northEastState[13 - 1]) numLife++;
      if (eastState[1 - 1]) numLife++;
      if (eastState[5 - 1]) numLife++;
      if (ourState[8 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[3 - 1]) numLife++;
      if (northState[15 - 1]) numLife++;
      break;
    case 5 - 1:
      if (ourState[1 - 1]) numLife++;
      if (ourState[2 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[9 - 1]) numLife++;
      if (westState[12 - 1]) numLife++;
      if (westState[8 - 1]) numLife++;
      if (westState[4 - 1]) numLife++;
      break;
    case 6 - 1:
      if (ourState[2 - 1]) numLife++;
      if (ourState[3 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[9 - 1]) numLife++;
      if (ourState[5 - 1]) numLife++;
      if (ourState[1 - 1]) numLife++;
      break;
    case 7 - 1:
      if (ourState[3 - 1]) numLife++;
      if (ourState[4 - 1]) numLife++;
      if (ourState[8 - 1]) numLife++;
      if (ourState[12 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[2 - 1]) numLife++;
      break;
    case 8 - 1:
      if (ourState[4 - 1]) numLife++;
      if (eastState[1 - 1]) numLife++;
      if (eastState[5 - 1]) numLife++;
      if (eastState[9 - 1]) numLife++;
      if (ourState[12 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[3 - 1]) numLife++;
      break;
    case 9 - 1:
      if (ourState[5 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[14 - 1]) numLife++;
      if (ourState[13 - 1]) numLife++;
      if (westState[16 - 1]) numLife++;
      if (westState[12 - 1]) numLife++;
      if (westState[8 - 1]) numLife++;
      break;
    case 10 - 1:
      if (ourState[6 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[15 - 1]) numLife++;
      if (ourState[14 - 1]) numLife++;
      if (ourState[13 - 1]) numLife++;
      if (ourState[9 - 1]) numLife++;
      if (ourState[5 - 1]) numLife++;
      break;
    case 11 - 1:
      if (ourState[7 - 1]) numLife++;
      if (ourState[8 - 1]) numLife++;
      if (ourState[12 - 1]) numLife++;
      if (ourState[16 - 1]) numLife++;
      if (ourState[15 - 1]) numLife++;
      if (ourState[14 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[6 - 1]) numLife++;
      break;
    case 12 - 1:
      if (ourState[8 - 1]) numLife++;
      if (eastState[5 - 1]) numLife++;
      if (eastState[9 - 1]) numLife++;
      if (eastState[13 - 1]) numLife++;
      if (ourState[16 - 1]) numLife++;
      if (ourState[15 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[7 - 1]) numLife++;
      break;
    case 13 - 1:
      if (ourState[9 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      if (ourState[14 - 1]) numLife++;
      if (southState[2 - 1]) numLife++;
      if (southState[1 - 1]) numLife++;
      if (southWestState[4 - 1]) numLife++;
      if (westState[16 - 1]) numLife++;
      if (westState[12 - 1]) numLife++;
      break;
    case 14 - 1:
      if (ourState[10 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      if (ourState[15 - 1]) numLife++;
      if (southState[3 - 1]) numLife++;
      if (southState[2 - 1]) numLife++;
      if (southState[1 - 1]) numLife++;
      if (ourState[13 - 1]) numLife++;
      if (ourState[9 - 1]) numLife++;
      break;
    case 15 - 1:
      if (ourState[11 - 1]) numLife++;
      if (ourState[12 - 1]) numLife++;
      if (ourState[16 - 1]) numLife++;
      if (southState[4 - 1]) numLife++;
      if (southState[3 - 1]) numLife++;
      if (southState[2 - 1]) numLife++;
      if (ourState[14 - 1]) numLife++;
      if (ourState[10 - 1]) numLife++;
      break;
    case 16 - 1:
      if (ourState[12 - 1]) numLife++;
      if (eastState[9 - 1]) numLife++;
      if (eastState[13 - 1]) numLife++;
      if (southEastState[1 - 1]) numLife++;
      if (southState[4 - 1]) numLife++;
      if (southState[3 - 1]) numLife++;
      if (ourState[15 - 1]) numLife++;
      if (ourState[11 - 1]) numLife++;
      break;
  }
  return numLife;
}

void DumpState() {
  Serial.printf("%d %d %d %d %d %d\n", northWestState[16 - 1], northState[13 - 1], northState[14 - 1], northState[15 - 1], northState[16 - 1], northEastState[13 - 1]);
  Serial.printf("%d %d %d %d %d %d\n", westState[4 - 1], ourState[1 - 1], ourState[2 - 1], ourState[3 - 1], ourState[4 - 1], eastState[1 - 1]);
  Serial.printf("%d %d %d %d %d %d\n", westState[8 - 1], ourState[5 - 1], ourState[6 - 1], ourState[7 - 1], ourState[8 - 1], eastState[5 - 1]);
  Serial.printf("%d %d %d %d %d %d\n", westState[12 - 1], ourState[9 - 1], ourState[10 - 1], ourState[11 - 1], ourState[12 - 1], eastState[9 - 1]);
  Serial.printf("%d %d %d %d %d %d\n", westState[16 - 1], ourState[13 - 1], ourState[14 - 1], ourState[15 - 1], ourState[16 - 1], eastState[13 - 1]);
  Serial.printf("%d %d %d %d %d %d\n", southWestState[4 - 1], southState[1 - 1], southState[2 - 1], southState[3 - 1], southState[4 - 1], southEastState[1 - 1]);
}

void Evolve() {
  bool newState[kNodeLedCount];
  memcpy(newState, ourState, sizeof(newState));
  for (int i = 0; i < kNodeLedCount; i++) {
    const int liveNeighbours = CountLifeNeighbours(i);
    if (ourState[i]) {
      // Live cell with less than 2 live neighbours dies (underpopulation)
      if (liveNeighbours < 2) newState[i] = false;
      // Live cell with more than 3 live neighbours dies (overpopulation)
      if (liveNeighbours > 3) newState[i] = false;
    } else {
      // Dead cell becomes alive if it has extactly 3 live neighbours
      if (liveNeighbours == 3) newState[i] = true;
    }
  }
#ifdef DEBUG_OUTPUT
  if (debugEvolve) {
    Serial.printf("Evolve before:\n");
    DumpState();
  }
#endif
  // Need to clear neighbours state to ensure that on next iteration if we don't hear from them again we con't keep them alive.
  ClearState();
  memcpy(ourState, newState, sizeof(ourState));
#ifdef DEBUG_OUTPUT
  if (debugEvolve) {
    Serial.printf("Evolve after:\n");
    DumpState();
  }
#endif
}

void ReportState() {
  using Defs = ::ProtocolDefs;
  const uint16_t arg = EncodeState(ourState);
#ifdef DEBUG_OUTPUT
  Serial.printf("SendEvent ourState %04x\n", arg);
#endif
  SendEvent(Defs::CreateEvent(Defs::kStateReport, engine.GetX(), engine.GetY(), EncodeState(ourState)));
}

void ClearState() {
#ifdef DEBUG_OUTPUT
  if (debugClearState) {
    Serial.printf("Before ClearState\n");
    DumpState();
  }
#endif
  memset(ourState, false, kNodeLedCount);
  memset(northState, false, kNodeLedCount);
  memset(northEastState, false, kNodeLedCount);
  memset(eastState, false, kNodeLedCount);
  memset(southEastState, false, kNodeLedCount);
  memset(southState, false, kNodeLedCount);
  memset(southWestState, false, kNodeLedCount);
  memset(westState, false, kNodeLedCount);
  memset(northWestState, false, kNodeLedCount);
#ifdef DEBUG_OUTPUT
  if (debugClearState) {
    Serial.printf("After ClearState\n");
    DumpState();
  }
#endif DEBUG_OUTPUT
}

void OnGlobalEvent(uint64_t ev, uint16_t src) {
  using Defs = ::ProtocolDefs;
  engine.OnGlobalEvent(ev, src);
  if (!Defs::IsProtocolEvent(ev)) return;
  // more stuff needs to come here...
  switch (Defs::GetCommand(ev)) {
    case Defs::kGlobalCmd:
      switch (Defs::GetArg(ev)) {
        case Defs::kSetStateRandom:
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent received kGlobalCmd/kSetStateRandom from %d\n", src);
#endif
          for (int i = 0; i < sizeof(ourState); i++) {
            ourState[i] = ((rand() % 100) < 35);
          }
          break;
        case Defs::kClearState:
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent received kGlobalCmd/kClearState from %d\n", src);
#endif
          ClearState();
          break;
        case Defs::kReportState:
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent received kGlobalCmd/kReportState from %d\n", src);
#endif
          ReportState();
          break;
        case Defs::kEvolveAndReport:
          Evolve();
          ReportState();
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent received kGlobalCmd/kEvolveAndReport from %d\n", src);
#endif
          break;
      }
      break;
    case Defs::kStateSet:
      if ((engine.GetX() == Defs::GetX(ev)) && (engine.GetY() == Defs::GetY(ev))) {
#ifdef DEBUG_OUTPUT
        Serial.printf("OnGlobalEvent kStateSet to %d for node %d/%d (us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
        ApplyState(ourState, APPLY_SET, Defs::GetArg(ev));
      } else {
#ifdef DEBUG_OUTPUT
        Serial.printf("OnGlobalEvent kStateSet to %d for node %d/%d (not us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
      }
      break;
    case Defs::kStateOr:
      if ((engine.GetX() == Defs::GetX(ev)) && (engine.GetY() == Defs::GetY(ev))) {
#ifdef DEBUG_OUTPUT
        Serial.printf("OnGlobalEvent kStateOr to %d for node %d/%d (us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
        ApplyState(ourState, APPLY_OR, Defs::GetArg(ev));
      } else {
#ifdef DEBUG_OUTPUT
        Serial.printf("OnGlobalEvent kStateOr to %d for node %d/%d (not us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
      }
      break;
    case Defs::kStateReport:
      {
        const int offsetX = Defs::GetX(ev) - engine.GetX();
        const int offsetY = Defs::GetY(ev) - engine.GetY();
        if ((offsetX == 0) && (offsetY == 0)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
        } else if ((offsetX == 0) && (offsetY == -1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (north peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(northState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == 1) && (offsetY == -1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (north east peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(northEastState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == 1) && (offsetY == 0)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (east peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(eastState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == 1) && (offsetY == 1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (south east peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(southEastState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == 0) && (offsetY == 1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (south peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(southState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == -1) && (offsetY == 1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (south west peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(southWestState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == -1) && (offsetY == 0)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (west peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(westState, APPLY_SET, Defs::GetArg(ev));
        } else if ((offsetX == -1) && (offsetY == -1)) {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (north west peer) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
          ApplyState(northWestState, APPLY_SET, Defs::GetArg(ev));
        } else {
#ifdef DEBUG_OUTPUT
          Serial.printf("OnGlobalEvent kStateReport with payload %d for node %d/%d (other node) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
#endif
        }
      }
      break;
    case Defs::kButtonPressed:
      if ((engine.GetX() == Defs::GetX(ev)) && (engine.GetY() == Defs::GetY(ev))) {
        Serial.printf("OnGlobalEvent kButtonPressed button %d for node %d/%d (us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
      } else {
        Serial.printf("OnGlobalEvent kButtonPressed button %d for node %d/%d (not us) from %d\n", Defs::GetArg(ev), Defs::GetX(ev), Defs::GetY(ev), src);
      }
      break;
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
  OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kReportState), 0);
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
    debugEvolve = true;
    OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kEvolveAndReport), 0);
    uint64_t last = LastSentEvent();
    uint64_t exp = Defs::CreateEvent(Defs::kStateReport, engine.kTestX, engine.kTestY, state_after);
    AssertEvent(exp, last);
    debugEvolve = false;
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

    debugEvolve = true;
    OnGlobalEvent(Defs::CreateGlobalCmd(Defs::kEvolveAndReport), 0);
    debugEvolve = false;
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


// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
  static int ctr = 0;
  ++ctr;
  updateLeds(ctr % 6);
}


// This function will be called 4 times per second by the timer.
void Timer4Hz() {
  static int ctr = 0;
  ++ctr;

  using Defs = ::ProtocolDefs;
  static int lastButtonPressedId = -1;
  if (IsMenuPressed()) {
    if (lastButtonPressedId == 16) {
      // Still pressing Menu
    } else {
      lastButtonPressedId = 16;
      SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), lastButtonPressedId));
    }
  } else {
    const int currentButtonPressedId = GetButtonNrPressed() - 1;
    if (currentButtonPressedId == -1) {
      // No button pressed
      lastButtonPressedId = -1;
    } else {
      if (lastButtonPressedId == currentButtonPressedId) {
        // Still pressing the same button
      } else {
        lastButtonPressedId = currentButtonPressedId;
        SendEvent(Defs::CreateEvent(Defs::kButtonPressed, engine.GetX(), engine.GetY(), lastButtonPressedId));
      }
    }
  }

#ifndef DEMO_MODE
  memcpy(leds, ourState, sizeof(ourState));
#else
  static bool isRollingMode = true;
  if (IsMenuPressed()) {
    isRollingMode = !isRollingMode;
#ifdef DEBUG_OUTPUT
    Serial.printf("isRollingMode: %d\n", isRollingMode);
#endif
    return;
  }

  int lidIdx = -1;
  if (isRollingMode) {
    lidIdx = (ctr % 16);
#ifdef DEBUG_OUTPUT
    Serial.printf("Rolling lidIdx: %d\n", lidIdx);
#endif
    for (int i = 0; i < 16; i++) {
      leds[i] = (i <= lidIdx);
    }
  } else {
    lidIdx = (GetButtonNrPressed() - 1);
    if (lidIdx >= 0) {
      leds[lidIdx] = !leds[lidIdx];
#ifdef DEBUG_OUTPUT
      Serial.printf("Button lidIdx: %d is now %d\n", lidIdx, leds[lidIdx]);
#endif
    }
  }
#endif
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

  //digitalWrite(kLed13Pin, !leds[13 - 1]);
  //digitalWrite(kLed14Pin, !leds[14 - 1]);
}
