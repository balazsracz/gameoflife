#include "touch-sensor.h"
#include "timer-helper.h"

// These variables are set to true when a given row in the key matrix is active (has a finger somewhere).
// Note: the MENU button has row 1 + row 3 and no columns active. All other buttons have exactly one row and one
// column.
extern bool btn_row_active[4];
// These variables are set to true when a given column in the key matrix is active (has a finger somewhere).
extern bool btn_col_active[4];


// This function will be called 6000 times per second by the timer.
void Timer6000Hz() {
}
// This function will be called 4 times per second by the timer.
void Timer4Hz() {
}



void setup() {
  // put your setup code here, to run once:
  TouchSetup();
  TimerSetup();
}

void loop() {
  // put your main code here, to run repeatedly:
  TouchLoop();
  TimerLoop();
}
