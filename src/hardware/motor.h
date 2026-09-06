/**
 * @file motor.h
 * @brief DRV8833 Motor Driver for STM32F411 (TIM4 Hardware PWM @ 100kHz)
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include "config/io_mapping.h"
#include "config/model.h"

namespace hardware {

class Motor {
 private:
  static constexpr float emergency_threshold = 1.3f;
  static constexpr int MOT_DUTY_MIN = 30;   // 3% minimum duty
  static constexpr int MOT_DUTY_MAX = 950;  // 95% maximum duty

 public:
  Motor() {
    init();
    free();
  }

  bool init() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uint32_t timer_clk = HAL_RCC_GetSysClockFreq(); // 100MHz trên STM32F411
    uint32_t period = (timer_clk / 25000);         // 4000 cho 25kHz PWM (Chuẩn tần số DRV8833, êm ái và mô-men lớn)
    if (period < 10) period = 4000;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->PSC = 0;
    TIM4->ARR = period - 1;

    // PWM mode 1 + Preload enable trên CH1, CH2, CH3, CH4
    TIM4->CCMR1 = 0x6868;
    TIM4->CCMR2 = 0x6868;

    // Bật Compare Output (CC1E, CC2E, CC3E, CC4E)
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;

    TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    return true;
  }

  void drive(float valueL, float valueR) {
    if (emergency) return;
    if (!std::isfinite(valueL) || !std::isfinite(valueR)) {
      free();
      return;
    }
    if (std::abs(valueL) > emergency_threshold || std::abs(valueR) > emergency_threshold) {
      emergency_stop(valueL, valueR);
      return;
    }
    set_duty_left(valueL);
    set_duty_right(valueR);
  }

  void free() {
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
  }

  void brake() {
    uint32_t full = TIM4->ARR + 1;
    TIM4->CCR1 = full;
    TIM4->CCR2 = full;
    TIM4->CCR3 = full;
    TIM4->CCR4 = full;
  }

  void emergency_stop(float vL = 0, float vR = 0) {
    emergency = true;
    LOGE(">>> MOTOR EMERGENCY STOP (L=%.2f, R=%.2f) <<<", (double)vL, (double)vR);
    free();
  }

  void emergency_release() {
    emergency = false;
    free();
  }

  bool is_emergency() const { return emergency; }

 private:
  bool emergency = false;

  void set_duty_left(float duty) {
    duty *= MOTOR_L_DIR;
    if (std::abs(duty) < 0.001f) {
      TIM4->CCR3 = 0;
      TIM4->CCR4 = 0;
      return;
    }
    float abs_d = std::clamp(std::abs(duty), (float)MOT_DUTY_MIN / 1000.0f, (float)MOT_DUTY_MAX / 1000.0f);
    uint32_t pulse_inv = (uint32_t)((TIM4->ARR + 1) * (1.0f - abs_d));
    uint32_t pulse_full = TIM4->ARR + 1;

    // Chế độ Slow Decay (Brake Mode chuẩn Kerise): Giữ 1 chân HIGH, chân còn lại băm PWM đảo
    // Giúp motor có lực kéo cực mạnh ở PWM thấp, triệt tiêu deadzone
    if (duty > 0) {
      TIM4->CCR3 = pulse_full; // PB8 = HIGH
      TIM4->CCR4 = pulse_inv;  // PB9 = Inverted PWM
    } else {
      TIM4->CCR3 = pulse_inv;  // PB8 = Inverted PWM
      TIM4->CCR4 = pulse_full; // PB9 = HIGH
    }
  }

  void set_duty_right(float duty) {
    duty *= (MOTOR_R_DIR * model::MotorTrim);
    if (std::abs(duty) < 0.001f) {
      TIM4->CCR1 = 0;
      TIM4->CCR2 = 0;
      return;
    }
    float abs_d = std::clamp(std::abs(duty), (float)MOT_DUTY_MIN / 1000.0f, (float)MOT_DUTY_MAX / 1000.0f);
    uint32_t pulse_inv = (uint32_t)((TIM4->ARR + 1) * (1.0f - abs_d));
    uint32_t pulse_full = TIM4->ARR + 1;

    // Chế độ Slow Decay (Brake Mode chuẩn Kerise): Giữ 1 chân HIGH, chân còn lại băm PWM đảo
    if (duty > 0) {
      TIM4->CCR1 = pulse_full; // PB6 = HIGH
      TIM4->CCR2 = pulse_inv;  // PB7 = Inverted PWM
    } else {
      TIM4->CCR1 = pulse_inv;  // PB6 = Inverted PWM
      TIM4->CCR2 = pulse_full; // PB7 = HIGH
    }
  }
};

}  // namespace hardware
