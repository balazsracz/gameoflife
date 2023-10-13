/** \copyright
 * Copyright (c) 2020, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 *
 * \file Stm32CanUSB.ino
 * 
 * Example Arduino sketch for the Stm32 showing how to make a USB-CAN
 * adapter with minimal size and dependencies.
 *
 * @author Balazs Racz
 * @date 23 March 2020
 */

#include <Arduino.h>
#include <OpenMRNLite.h>

/// Which serial port to use.
/// =========================
/// IMPORTANT: TO USE THIS SKETCH, YOU MUST SET Tools->USB
/// Support TO CDC (either one).
/// =========================
#define SERIAL_PORT SerialUSB

/// Specifies which Stm32 Pin should be used for CAN_TX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_TX_PIN PB_9

/// Specifies which Stm32 Pin should be used for CAN_RX signal. The default
/// matches the pinout of the OpenLCB Dev Kit daughterboard.
#define CAN_RX_PIN PB_8

// These two pins have an LED on them. The LED will light when you write LOW to the respective pin.
static constexpr int kLed13Pin = PF0;
static constexpr int kLed14Pin = PF1;

Stm32Can Can("/dev/can0");

OVERRIDE_CONST_TRUE(gc_generate_newlines);

/// These objects perform the crossbar switching of CAN frames.
Executor<1> openmrn_executor{NO_THREAD()};
Service openmrn_service(&openmrn_executor);
CanHubFlow openmrn_can_hub(&openmrn_service);
std::unique_ptr<Executable> can_bridge;
std::unique_ptr<Executable> serial_bridge;

// Fixes the CAN ports for open-drain operation.
void fixOD() {
  for (auto pin : { LL_GPIO_PIN_8, LL_GPIO_PIN_9, LL_GPIO_PIN_10, LL_GPIO_PIN_11 }) {
    LL_GPIO_SetPinOutputType(GPIOB, pin, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, pin, LL_GPIO_PULL_UP);
  }  
}

/// Arduino setup routine. Initializes the CAN-bus and the serial port.
void setup() {
  pinMode(kLed13Pin, OUTPUT);
  pinMode(kLed14Pin, OUTPUT);
  digitalWrite(kLed13Pin, HIGH);
  digitalWrite(kLed14Pin, HIGH);  
  // Baud rate does not matter for USB ports.
  SERIAL_PORT.begin(115200);
  arduino_can_pinmap(CAN_TX_PIN, CAN_RX_PIN);
  fixOD();
  pinMode(LED_BUILTIN, OUTPUT);
  // Ties the two ports into the Hub.
  can_bridge.reset(new openmrn_arduino::CanBridge(&Can, &openmrn_can_hub));
  fixOD();
  serial_bridge.reset(new openmrn_arduino::SerialBridge<decltype(SERIAL_PORT)>(
      &SERIAL_PORT, &openmrn_can_hub));
}

/// Arduino loop routine. Calls the OpenMRN software to do its work.
void loop() {
  openmrn_executor.loop_some();

  // transmit busy LED.
  digitalWrite(kLed14Pin, (CAN->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) != 0 ? LOW : HIGH);
  static uint32_t rx_led_timeout = 0;
  if (Can.available()) {
    // There is a CAN packet to read.
    digitalWrite(kLed13Pin, LOW);
    rx_led_timeout = HAL_GetTick() + 2;
  }
  can_bridge->run();
  if (HAL_GetTick() > rx_led_timeout) {
    digitalWrite(kLed13Pin, HIGH);
  }
  serial_bridge->run();
}
