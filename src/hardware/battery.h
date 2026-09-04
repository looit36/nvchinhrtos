/**
 * @file battery.h
 * @brief Battery Voltage Monitor via ADC1 + DMA2 Circular Mode for STM32F411
 */
#pragma once

#include <Arduino.h>
#include "config/io_mapping.h"
#include "config/model.h"
#include "app_log.h"

namespace hardware {

class Battery {
 public:
  static constexpr int BUFFER_SIZE = 16;

  Battery() {}

  bool init() {
    // 1. Bật Clock cho GPIOB, ADC1 và DMA2
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // 2. Cấu hình chân BAT_VOL_PIN (PB0 - ADC1_IN8) ở chế độ Analog
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Cấu hình DMA2 Stream 0 Channel 0 (Ánh xạ phần cứng tới ADC1 trên STM32F411)
    // Tắt Stream trước khi cấu hình
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

    // Xóa toàn bộ cờ ngắt cũ của Stream 0
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    // Địa chỉ ngoại vi: ADC1 Data Register (DR)
    DMA2_Stream0->PAR = reinterpret_cast<uint32_t>(&(ADC1->DR));
    // Địa chỉ bộ nhớ: buffer dma_buf
    DMA2_Stream0->M0AR = reinterpret_cast<uint32_t>(dma_buf);
    // Số lượng mẫu
    DMA2_Stream0->NDTR = BUFFER_SIZE;

    // Thiết lập CR: Channel 0, 16-bit MSIZE, 16-bit PSIZE, Memory Increment, Circular Mode, Medium Priority
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos)       // Channel 0 = ADC1
                     | DMA_SxCR_PL_0                   // Medium Priority
                     | DMA_SxCR_MSIZE_0                // Memory data size: 16-bit
                     | DMA_SxCR_PSIZE_0                // Peripheral data size: 16-bit
                     | DMA_SxCR_MINC                   // Memory pointer increment
                     | DMA_SxCR_CIRC;                  // Circular mode (tự động quay vòng liên tục)

    // Bật DMA2 Stream 0
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // 4. Cấu hình ADC1
    // Prescaler cho ADC (PCLK2 / 4 = 100MHz / 2 / 4 = 12.5MHz, nằm trong khoảng 0.6MHz - 36MHz)
    ADC->CCR = ADC_CCR_ADCPRE_0;

    // CR1: Độ phân giải 12-bit (RES = 00), Scan mode = 0 (đơn kênh)
    ADC1->CR1 = 0;

    // SMPR2: Thời gian lấy mẫu cho Channel 8 (PB0) = 480 chu kỳ (tối đa ổn định, chống nhiễu)
    ADC1->SMPR2 = (ADC_SMPR2_SMP8_2 | ADC_SMPR2_SMP8_1 | ADC_SMPR2_SMP8_0);

    // SQR1: 1 chuyển đổi (L = 0)
    ADC1->SQR1 = 0;
    // SQR3: Channel 8 là chuyển đổi thứ 1
    ADC1->SQR3 = 8;

    // CR2: Bật ADC, Chế độ chuyển đổi liên tục (CONT), Kích hoạt DMA (DMA), DMA Request liên tục (DDS)
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;

    // Đợi ADC ổn định (tSTAB theo datasheet ~3 µs)
    delayMicroseconds(10);

    // Bắt đầu chuyển đổi liên tục bằng phần mềm (SWSTART)
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Đợi DMA chuyển đổi hoàn thành ít nhất 1 vòng buffer đầu tiên
    delay(2);

    LOGI("Battery ADC1 + DMA2 Circular Mode initialized. Initial Voltage: %.2fV", (double)get_voltage());
    return true;
  }

  static float get_voltage() {
    uint32_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; ++i) {
      sum += dma_buf[i];
    }
    float raw_avg = static_cast<float>(sum) / static_cast<float>(BUFFER_SIZE);
    float v = raw_avg * model::BatteryMultiplier;
    return (v > 1.0f && v < 15.0f) ? v : 7.4f;
  }

  static uint16_t get_raw() {
    uint32_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; ++i) {
      sum += dma_buf[i];
    }
    return static_cast<uint16_t>(sum / BUFFER_SIZE);
  }

 private:
  static inline volatile uint16_t dma_buf[BUFFER_SIZE] = {0};
};

}  // namespace hardware
