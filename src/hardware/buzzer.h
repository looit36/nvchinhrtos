/**
 * @file buzzer.h
 * @brief Buzzer Interface for STM32
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>

namespace hardware {

class Buzzer {
 public:
  enum Music : uint8_t {
    SELECT,
    CANCEL,
    CONFIRM,
    SUCCESSFUL,
    ERROR,
    UP,
    DOWN,
    COMPLETE,
    BOOT,
    SHUTDOWN,
    TIMEOUT,
    EMERGENCY,
    MAZE_BACKUP,
    MAZE_RESTORE,
    CALIBRATION,
    AEBS,
    SHORT6,
    SHORT7,
    SHORT8,
    SHORT9,
    MUSIC_MAX,
  };

 public:
  static Buzzer* get_instance() {
    static Buzzer instance;
    return &instance;
  }

  bool init(uint32_t pin = 0) {
    (void)pin;
    return true;
  }

  void play(const enum Music music, TickType_t xTicksToWait = 0) {
    (void)music;
    (void)xTicksToWait;
  }
};

}  // namespace hardware
