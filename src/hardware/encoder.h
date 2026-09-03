/**
 * @file encoder.h
 * @brief MT6701 ABZ Hardware Timer Encoder Driver for STM32
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <freertospp/mutex.h>
#include <freertospp/semphr.h>
#include <mutex>
#include "config/io_mapping.h"
#include "config/model.h"
#include "app_log.h"

namespace hardware {

class Encoder {
 public:
  Encoder() {}

  bool init() {
    // 1. Cấu hình TIM3 cho Motor Trái (PB4: TIM3_CH1, PB5: TIM3_CH2)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = TIM_ENCODERMODE_TI12; // Đếm 4X trên cả TI1 và TI2
    TIM3->CCMR1 = 0x3131;              // Lọc nhiễu Filter = 3
    TIM3->CCER = 0x0013;               // CC1E=1, CC1P=1 (Đảo cực tính TI1), CC2E=1
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->CR1 |= TIM_CR1_CEN;
    lastRawCnt[0] = TIM3->CNT;

    // 2. Cấu hình TIM2 cho Motor Phải (PA15: TIM2_CH1, PB3: TIM2_CH2)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    TIM2->SMCR = TIM_ENCODERMODE_TI12; // Đếm 4X
    TIM2->CCMR1 = 0x3131;              // Lọc nhiễu Filter = 3
    TIM2->CCER = 0x0013;               // Đếm tăng khi quay tiến
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
    lastRawCnt[1] = TIM2->CNT;

    totalCounts[0] = 0;
    totalCounts[1] = 0;
    positions[0] = 0.0f;
    positions[1] = 0.0f;

    xTaskCreate([](void* arg) { static_cast<Encoder*>(arg)->task(); },
                "Encoder", 512, this, 6, NULL);
    return true;
  }

  float get_position(uint8_t ch) {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return (ch < 2) ? positions[ch] : 0.0f;
  }

  int32_t get_total_count(uint8_t ch) {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    return (ch < 2) ? totalCounts[ch] : 0;
  }

  void clear_offset() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    totalCounts[0] = totalCounts[1] = 0;
    positions[0] = positions[1] = 0.0f;
  }

  void csv() {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    BTSerial.printf("%ld,%ld\n", totalCounts[0], totalCounts[1]);
  }

  bool sampling_sync(portTickType xBlockTime = pdMS_TO_TICKS(50)) const {
    return sampling_end_semaphore.take(xBlockTime);
  }

 private:
  uint32_t lastRawCnt[2] = {0, 0};
  int32_t totalCounts[2] = {0, 0};
  float positions[2] = {0.0f, 0.0f};

  mutable freertospp::Semaphore sampling_end_semaphore;
  freertospp::Mutex mutex;

  void task() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
      update();
      sampling_end_semaphore.give();
    }
  }

  void update() {
    // 1. TIM3 Left Encoder (16-bit)
    uint32_t cur0 = TIM3->CNT;
    int16_t diff0 = (int16_t)((uint16_t)cur0 - (uint16_t)lastRawCnt[0]);
    lastRawCnt[0] = cur0;

    // 2. TIM2 Right Encoder (32-bit)
    uint32_t cur1 = TIM2->CNT;
    int32_t diff1 = (int32_t)(cur1 - lastRawCnt[1]);
    lastRawCnt[1] = cur1;

    std::lock_guard<freertospp::Mutex> lock(mutex);
    totalCounts[0] += diff0;
    totalCounts[1] += diff1;

    // Đổi xung sang milimet
    positions[0] = (float)totalCounts[0] * model::ScalePulsesToMm;
    positions[1] = (float)totalCounts[1] * model::ScalePulsesToMm;
  }
};

}  // namespace hardware
