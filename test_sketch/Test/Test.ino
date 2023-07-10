/*
  Test app

  Various hardware features will be tested in this sketch. 

  To compile, use these settings:
  - board manager = stm32
  - Board = "generic STM32F0 series"
  - Board part number = "Generic STM32F072CBTx"
  - Upload method = "Stm32CubeProgrammer (DFU)"
*/

static constexpr auto LED13_PIN = PF0;
static constexpr auto LED14_PIN = PF1;

static constexpr auto MENU_PIN = PB15;

void setup() {
  pinMode(LED13_PIN, OUTPUT);
  pinMode(LED14_PIN, OUTPUT);
  pinMode(MENU_PIN, INPUT_PULLUP);
}

void loop() {
  static int i = 0;
  i++;
  delay(1);
  digitalWrite(LED14_PIN, 0);
  //digitalWrite(LED14_PIN, digitalRead(MENU_PIN));
  if (i % 1000 < 500) {
    digitalWrite(LED13_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  } else {
    digitalWrite(LED13_PIN, LOW);   // turn the LED off by making the voltage LOW
  }
}