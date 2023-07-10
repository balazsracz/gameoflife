/*
  Test app

  Various hardware features will be tested in this sketch. 

  To compile, use these settings:
  - board manager = stm32
  - Board = "generic STM32F0 series"
  - Board part number = "Generic STM32F072CBTx"
  - Upload method = "Stm32CubeProgrammer (DFU)"

  To upload, press finger firmly on the FLASH pad, then use a metal coin two hit the RESET pad. Then click upload in the Arduino toolbar.
*/

static constexpr int LED13_PIN = PF0;
static constexpr int LED14_PIN = PF1;

static constexpr int MENU_PIN = PB15;

#define BLINKER_TIMER TIM14

HardwareTimer timCharlie(BLINKER_TIMER);

void charliehandler(void) {
  static int i = 0;
  i++;
  digitalWrite(LED14_PIN, (i%6) == 0);
}


void setup() {
  pinMode(LED13_PIN, OUTPUT);
  pinMode(LED14_PIN, OUTPUT);
  pinMode(MENU_PIN, INPUT_PULLUP);

  timCharlie.attachInterrupt(charliehandler);
  timCharlie.setMode(1, TIMER_DISABLED);
  timCharlie.setOverflow(12, HERTZ_FORMAT);
  timCharlie.resume();
}

void loop() {
  static int i = 0;
  i++;
  delay(1);
  //digitalWrite(LED14_PIN, 0);
  //digitalWrite(LED14_PIN, digitalRead(MENU_PIN));
  if (i % 1000 < 500) {
    digitalWrite(LED13_PIN, HIGH);
  } else {
    digitalWrite(LED13_PIN, LOW);
  }
}