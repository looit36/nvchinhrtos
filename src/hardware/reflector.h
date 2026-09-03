/**
 * @file reflector.h
 * @brief Optical IR Wall Reflector Interface for STM32
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <array>
#include <freertospp/semphr.h>

namespace hardware {

class Reflector {
 public:
  static constexpr int CH_SIZE = 4;

  Reflector() {
    for (int i = 0; i < CH_SIZE; ++i) value[i] = 0;
  }

  bool init() {
    return true;
  }

  void sampling_sync(portTickType xBlockTime = 0) const {
    (void)xBlockTime;
  }

  void csv() {}

  int16_t value[CH_SIZE];
};

}  // namespace hardware
