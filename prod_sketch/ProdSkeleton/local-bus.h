#include "wiring_digital.h"
#include "digital_io.h"

static constexpr int kNorthTxPin = PA2;
static constexpr int kNorthRxPin = PA3;
static constexpr int kSouthTxPin = PA9;
static constexpr int kSouthRxPin = PA10;
static constexpr int kEastTxPin = PB10;
static constexpr int kEastRxPin = PB11;
static constexpr int kWestTxPin = PA0;
static constexpr int kWestRxPin = PA1;

// Sets all pins of local bus to open-drain mode.
void LocalGpioOD() {
  for (auto pin : { LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3, LL_GPIO_PIN_9, LL_GPIO_PIN_10 }) {
    LL_GPIO_SetPinOutputType(GPIOA, pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOA, pin, LL_GPIO_PULL_UP);
  }
  for (auto pin : { LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11 }) {
    LL_GPIO_SetPinOutputType(GPIOB, pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, pin, LL_GPIO_PULL_UP);
  }
}

// Prepares the local connections for GPIO mode.
void LocalBusGpio() {
  for (auto pin : { kEastRxPin, kWestRxPin, kNorthRxPin, kSouthRxPin }) {
    pinMode(pin, INPUT_PULLUP);
  }
  for (auto pin : { kEastTxPin, kWestTxPin, kNorthTxPin, kSouthTxPin }) {
    pinMode(pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(pin, HIGH);
  }
  LocalGpioOD();
}

HardwareSerial serial_north(kNorthRxPin, kNorthTxPin);
HardwareSerial serial_south(kSouthRxPin, kSouthTxPin);
HardwareSerial serial_east(kEastRxPin, kEastTxPin);
HardwareSerial serial_west(kWestRxPin, kWestTxPin);

// Sets up the local connections for serial bus mode.
void LocalBusSerial() {
  serial_north.begin(9600);
  LocalGpioOD();
  serial_south.begin(9600);
  LocalGpioOD();
  serial_east.begin(9600);
  LocalGpioOD();
  serial_west.begin(9600);
  LocalGpioOD();
}

// Sets up local bus in default mode (GPIO).
void LocalBusSetup() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  LocalBusGpio();
}

// Sets the signal active or inactive on a local bus direction.
void LocalBusSignal(Direction dir, bool active) {
  int pin = NP;
  switch (dir) {
    case kNorth:
      pin = kNorthTxPin;
      break;
    case kEast:
      pin = kEastTxPin;
      break;
    case kSouth:
      pin = kSouthTxPin;
      break;
    case kWest:
      pin = kWestTxPin;
      break;
  }
  digitalWrite(pin, active ? LOW : HIGH);
}

// Returns true if the signal in a given direction is active (either from us or from the neighbor).
bool LocalBusIsActive(Direction dir) {
  int pin = NP;
  switch (dir) {
    case kNorth:
      pin = kNorthRxPin;
      break;
    case kEast:
      pin = kEastRxPin;
      break;
    case kSouth:
      pin = kSouthRxPin;
      break;
    case kWest:
      pin = kWestRxPin;
      break;
  }
  return digitalRead(pin) == LOW;
}

void LocalBusLoop() {}