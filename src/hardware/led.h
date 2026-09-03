/**
 * @file led.h
 * @brief LED Driver for STM32
 */
#pragma once

#include <Arduino.h>
#include "config/io_mapping.h"

namespace hardware {

class LED {
 public:
  LED() : value(0) {}

  bool init(uint32_t pin = LED_PIN) {
    this->pin = pin;
    pinMode(pin, OUTPUT);
    set(0);
    return true;
  }

  void set(uint8_t val) {
    value = val;
    // On-board PC13 LED: active LOW
    digitalWrite(pin, (val & 0x01) ? LOW : HIGH);
  }

  uint8_t get() const {
    return value;
  }

  void toggle() {
    set(value ^ 0x01);
  }

 private:
  uint32_t pin = LED_PIN;
  uint8_t value = 0;
};

}  // namespace hardware
