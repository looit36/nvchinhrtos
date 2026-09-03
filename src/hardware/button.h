/**
 * @file button.h
 * @brief Button Driver for STM32 PB1 (Active HIGH when pressed)
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "config/io_mapping.h"

namespace hardware {

class Button {
 public:
  Button() : flags(0), pin(BUTTON_PIN), counter(0) {}

  bool init(uint32_t pin = BUTTON_PIN) {
    this->pin = pin;

    // Chân PB1 chế độ INPUT, NO PULL-UP, NO PULL-DOWN
    pinMode(pin, INPUT);

    flags = 0x00;
    xTaskCreate([](void* arg) { static_cast<Button*>(arg)->task(); },
                "Button", 256, this, 1, NULL);
    return true;
  }

  uint32_t get_pin() const { return pin; }

  union {
    uint8_t flags;
    struct {
      uint8_t pressed : 1;         /**< clicked & released */
      uint8_t long_pressed_1 : 1;  /**< long-pressed level 1 (>400ms) */
      uint8_t long_pressed_2 : 1;  /**< long-pressed level 2 (>2s) */
      uint8_t long_pressed_3 : 1;  /**< long-pressed level 3 (>10s) */
      uint8_t pressing : 1;        /**< currently pressing */
      uint8_t long_pressing_1 : 1; /**< currently long-pressing level 1 */
      uint8_t long_pressing_2 : 1; /**< currently long-pressing level 2 */
      uint8_t long_pressing_3 : 1; /**< currently long-pressing level 3 */
    };
  };

  bool is_pressed() {
    if (pressed) {
      pressed = 0;
      return true;
    }
    return false;
  }

  bool is_active() const {
    return digitalRead(pin) == HIGH;
  }

  bool is_pressing() const {
    return pressing;
  }

 private:
  static constexpr int button_sampling_time_ms = 20;
  static constexpr int button_time_press = 1;         // 20ms
  static constexpr int button_time_long_press_1 = 20; // 400ms
  static constexpr int button_time_long_press_2 = 100;// 2000ms
  static constexpr int button_time_long_press_3 = 500;// 10000ms

  uint32_t pin;
  int counter;

  void update() {
    // Nút PB1: Nhấn thì được kéo lên mức CAO (HIGH)
    bool active = (digitalRead(pin) == HIGH);
    if (active) {
      if (counter < button_time_long_press_3 + 1)
        counter++;
      if (counter >= button_time_long_press_3)
        long_pressing_3 = 1;
      else if (counter >= button_time_long_press_2)
        long_pressing_2 = 1;
      else if (counter >= button_time_long_press_1)
        long_pressing_1 = 1;
      if (counter >= button_time_press)
        pressing = 1;
    } else {
      if (counter >= button_time_long_press_3)
        long_pressed_3 = 1;
      else if (counter >= button_time_long_press_2)
        long_pressed_2 = 1;
      else if (counter >= button_time_long_press_1)
        long_pressed_1 = 1;
      else if (counter >= button_time_press)
        pressed = 1;

      counter = 0;
      pressing = 0;
      long_pressing_1 = 0;
      long_pressing_2 = 0;
      long_pressing_3 = 0;
    }
  }

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(button_sampling_time_ms));
      update();
    }
  }
};

}  // namespace hardware
