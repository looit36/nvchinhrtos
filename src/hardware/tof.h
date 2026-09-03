/**
 * @file tof.h
 * @brief ToF Sensor Interface for STM32
 */
#pragma once

#include <Arduino.h>
#include <ctrl/accumulator.h>

namespace hardware {

class ToF {
 public:
  ToF() : enabled(false) {
    log.clear(255);
  }

  bool init() {
    enabled = true;
    log.clear(255);
    return true;
  }

  void enable() { enabled = true; }
  void disable() { enabled = false; }
  bool is_enabled() const { return enabled; }
  bool isValid() const { return false; }

  int16_t getDistance() const { return 255; }
  int16_t getRangeRaw() const { return 255; }
  uint32_t passedTimeMs() const { return 100; }
  const ctrl::Accumulator<int16_t, 4>& getLog() const { return log; }
  void print() {}

 private:
  bool enabled;
  ctrl::Accumulator<int16_t, 4> log;
};

}  // namespace hardware
