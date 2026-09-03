/**
 * @file fan.h
 * @brief Suction Fan Interface for STM32
 */
#pragma once

#include <Arduino.h>

namespace hardware {

class Fan {
 public:
  Fan(uint32_t pin = 0) : pin(pin), duty(0.0f) {}

  void drive(float duty) {
    this->duty = duty;
  }

  float get_duty() const {
    return duty;
  }

 private:
  uint32_t pin;
  float duty;
};

}  // namespace hardware
