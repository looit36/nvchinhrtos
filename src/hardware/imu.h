/**
 * @file imu.h
 * @brief BMI160 SPI Driver for STM32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include <task.h>
#include <freertospp/mutex.h>
#include <freertospp/semphr.h>
#include <mutex>
#include "config/io_mapping.h"
#include "config/model.h"
#include "app_log.h"

namespace hardware {

class IMU {
 public:
  static constexpr float Ts = 1e-3f;

  IMU() : hspi2(IMU_SPI_MOSI, IMU_SPI_MISO, IMU_SPI_SCK),
          bmi160Settings(5000000, MSBFIRST, SPI_MODE0) {}

  bool init() {
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);

    hspi2.setMOSI(IMU_SPI_MOSI);
    hspi2.setMISO(IMU_SPI_MISO);
    hspi2.setSCLK(IMU_SPI_SCK);
    hspi2.begin();

    delay(20);

    // Dummy read để ép BMI160 vào SPI mode
    hspi2.beginTransaction(bmi160Settings);
    digitalWrite(IMU_SPI_CS, LOW);
    hspi2.transfer(0x7F | 0x80);
    hspi2.transfer(0x00);
    digitalWrite(IMU_SPI_CS, HIGH);
    hspi2.endTransaction();
    delay(10);

    // Soft reset
    writeRegisterSPI(0x7E, 0xB6);
    delay(100);

    // Dummy read lại sau soft-reset
    readRegisterSPI(0x7F);
    delay(10);

    // Kiểm tra CHIP ID
    uint8_t chip_id = readRegisterSPI(0x00);
    if (chip_id != 0xD1) {
      LOGE("BMI160 not found! ID: 0x%02X", chip_id);
      return false;
    }
    LOGI("BMI160 initialized successfully!");

    // Wake up Accel & Gyro
    writeRegisterSPI(0x7E, 0x11); // Accel normal mode
    delay(10);
    writeRegisterSPI(0x7E, 0x15); // Gyro normal mode
    delay(100);

    // Cấu hình Range: Accel ±2g, Gyro ±2000 dps
    writeRegisterSPI(0x41, 0x03);
    delay(10);
    writeRegisterSPI(0x43, 0x00);
    delay(10);

    // Cấu hình ODR & Lọc thông thấp phần cứng (Hardware Low-Pass Filter):
    // 0x40 (ACC_CONF): 0x0B -> ODR = 800Hz, OSR4 hardware LPF (cutoff 80Hz)
    writeRegisterSPI(0x40, 0x0B);
    delay(10);
    // 0x42 (GYR_CONF): 0x0C -> ODR = 1600Hz, OSR4 hardware LPF (cutoff 127Hz)
    writeRegisterSPI(0x42, 0x0C);
    delay(10);

    // Calibrate offset
    calibration();

    xTaskCreate([](void* arg) { static_cast<IMU*>(arg)->task(); },
                "IMU", 512, this, 6, NULL);
    return true;
  }

  void calibration() {
    float gz_sum = 0.0f, az_sum = 0.0f;
    const int N = 300;

    // Discard transient
    for (int i = 0; i < 50; i++) {
      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        sampling_sync(pdMS_TO_TICKS(10));
      } else {
        raw_update();
        delay(1);
      }
    }

    for (int i = 0; i < N; i++) {
      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        sampling_sync(pdMS_TO_TICKS(10));
      } else {
        raw_update();
        delay(1);
      }
      gz_sum += raw_gyro_z;
      az_sum += raw_accel_x;
    }

    std::lock_guard<freertospp::Mutex> lock(mutex);
    gyro_offset = gz_sum / (float)N;
    accel_offset = az_sum / (float)N;
    angle = 0.0f;
    LOGI("IMU Calib Done. Gyro Offset: %.4f dps", (double)gyro_offset);
  }

  bool sampling_sync(portTickType xBlockTime = pdMS_TO_TICKS(50)) const {
    return sampling_end_semaphore.take(xBlockTime);
  }

  float get_gyro() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return gyro_rad; // rad/s
  }

  float get_accel() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return accel_mm; // mm/s^2
  }

  float get_angular_accel() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return angular_accel; // rad/s^2
  }

  float get_angle() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return angle; // rad
  }

  void reset_angle() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    angle = 0.0f;
  }

  void print() {
    LOGI("IMU Gyro: %.2f rad/s, Accel: %.2f mm/s2, Angle: %.2f deg",
         (double)get_gyro(), (double)get_accel(), (double)(get_angle() * 180.0f / PI));
  }

  void csv() {
    BTSerial.printf("0,%.3f,%.3f,%.3f\n", get_gyro(), get_accel(), get_angle());
  }

 private:
  SPIClass hspi2;
  SPISettings bmi160Settings;

  float raw_gyro_z = 0.0f;
  float raw_accel_x = 0.0f;
  float gyro_offset = 0.0f;
  float accel_offset = 0.0f;

  float gyro_rad = 0.0f;       // rad/s
  float prev_gyro_rad = 0.0f;
  float accel_mm = 0.0f;       // mm/s^2
  float angular_accel = 0.0f;  // rad/s^2
  float angle = 0.0f;          // rad

  mutable freertospp::Semaphore sampling_end_semaphore;
  freertospp::Mutex mutex;

  void writeRegisterSPI(uint8_t reg, uint8_t data) {
    hspi2.beginTransaction(bmi160Settings);
    digitalWrite(IMU_SPI_CS, LOW);
    hspi2.transfer(reg);
    hspi2.transfer(data);
    digitalWrite(IMU_SPI_CS, HIGH);
    hspi2.endTransaction();
  }

  uint8_t readRegisterSPI(uint8_t reg) {
    hspi2.beginTransaction(bmi160Settings);
    digitalWrite(IMU_SPI_CS, LOW);
    hspi2.transfer(reg | 0x80);
    uint8_t val = hspi2.transfer(0x00);
    digitalWrite(IMU_SPI_CS, HIGH);
    hspi2.endTransaction();
    return val;
  }

  void readRegistersSPI(uint8_t reg, uint8_t *buffer, uint8_t length) {
    hspi2.beginTransaction(bmi160Settings);
    digitalWrite(IMU_SPI_CS, LOW);
    hspi2.transfer(reg | 0x80);
    for (uint8_t i = 0; i < length; i++) {
      buffer[i] = hspi2.transfer(0x00);
    }
    digitalWrite(IMU_SPI_CS, HIGH);
    hspi2.endTransaction();
  }

  void raw_update() {
    uint8_t data[12];
    readRegistersSPI(0x0C, data, 12);
    int16_t rawGZ = (int16_t)((data[5] << 8) | data[4]);
    int16_t rawAX = (int16_t)((data[7] << 8) | data[6]);

    raw_gyro_z = (float)rawGZ / 16.4f; // deg/s
    raw_accel_x = (float)rawAX / 16384.0f * 9806.65f; // mm/s^2
  }

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      sampling_end_semaphore.give();
    }
  }

  void update() {
    raw_update();

    std::lock_guard<freertospp::Mutex> lock(mutex);
    // Tính vận tốc góc (rad/s)
    float gz_dps = (raw_gyro_z - gyro_offset);
    prev_gyro_rad = gyro_rad;
    const float raw_rad = gz_dps * (PI / 180.0f);
    gyro_rad = 0.75f * raw_rad + 0.25f * prev_gyro_rad; // Lọc rung động cơ tần số cao

    // Tính gia tốc tịnh tiến phương dọc (mm/s^2)
    accel_mm = (raw_accel_x - accel_offset);

    // Gia tốc góc (rad/s^2)
    angular_accel = (gyro_rad - prev_gyro_rad) / Ts;

    // Tích lũy góc quay (rad)
    angle += gyro_rad * Ts;
  }
};

}  // namespace hardware
