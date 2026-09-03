/**
 * @file hardware.h
 * @brief Hardware Manager for STM32 MicroMouse
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include "hardware/buzzer.h"
#include "hardware/button.h"
#include "hardware/encoder.h"
#include "hardware/fan.h"
#include "hardware/imu.h"
#include "hardware/led.h"
#include "hardware/motor.h"
#include "hardware/reflector.h"
#include "hardware/tof.h"
#include "config/model.h"
#include "config/io_mapping.h"
#include "app_log.h"

namespace hardware {

class Hardware {
 public:
  /* Driver */
  Buzzer* bz = nullptr;
  LED* led = nullptr;
  Motor* mt = nullptr;
  Fan* fan = nullptr;

  /* Sensor */
  Button* btn = nullptr;
  IMU* imu = nullptr;
  Encoder* enc = nullptr;
  Reflector* rfl = nullptr;
  ToF* tof = nullptr;

 public:
  Hardware() {}

  bool init() {
    bool result = true;

    // 1. Khởi tạo Buzzer & LED
    bz = Buzzer::get_instance();
    bz->init();

    led = new LED();
    led->init(LED_PIN);

    // 2. Khởi tạo Nút bấm PB1 (chế độ INPUT)
    btn = new Button();
    btn->init(BUTTON_PIN);

    // 3. Khởi tạo Pin ADC1 đọc điện áp Pin
    analogReadResolution(12);
    pinMode(BAT_VOL_PIN, INPUT_ANALOG);
    batteryCheck();

    // 4. Khởi tạo Motor (TIM4 PWM @ 100kHz)
    mt = new Motor();
    mt->free();

    // 5. Khởi tạo Fan
    fan = new Fan();

    // 6. Khởi tạo Encoder (MT6701 TIM3/TIM2)
    enc = new Encoder();
    if (!enc->init()) {
      LOGE("Encoder init failed!");
      result = false;
    }

    // 7. Khởi tạo IMU (BMI160 SPI2)
    imu = new IMU();
    if (!imu->init()) {
      LOGW("IMU init warning (BMI160 not ready or ID mismatch)!");
      // Không block toàn bộ hệ thống để người dùng vẫn thao tác được menu & test động cơ
    }

    // 8. Khởi tạo Cảm biến quang & ToF stub
    rfl = new Reflector();
    rfl->init();

    tof = new ToF();
    tof->init();

    LOGI("Hardware initialization complete!");
    return result;
  }

  static float getBatteryVoltage() {
    analogReadResolution(12);
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
      sum += analogRead(BAT_VOL_PIN);
      delayMicroseconds(20);
    }
    float raw_avg = (float)sum / 8.0f;
    float v = raw_avg * model::BatteryMultiplier;
    return (v > 1.0f && v < 15.0f) ? v : 7.4f;
  }

  bool batteryCheck() {
    float voltage = getBatteryVoltage();
    LOGI("Battery Voltage: %.2f [V]", (double)voltage);
    bz->play(Buzzer::BOOT);
    return true;
  }
};

}  // namespace hardware
