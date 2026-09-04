#include "MotorEncoder.h"
#include "config.h"

MotorEncoder::MotorEncoder(TIM_TypeDef *timer, bool reverse)
    : _timer(timer), _pinA(0), _pinB(0), _reverse(reverse), _totalCount(0), _lastRawCnt(0) {
}

MotorEncoder::MotorEncoder(uint8_t pinA, uint8_t pinB, bool reverse)
    : _timer(nullptr), _pinA(pinA), _pinB(pinB), _reverse(reverse), _totalCount(0), _lastRawCnt(0) {
    if ((pinA == PB4 && pinB == PB5) || (pinA == PB5 && pinB == PB4)) {
        _timer = TIM3;
    } else if ((pinA == PA15 && pinB == PB3) || (pinA == PB3 && pinB == PA15)) {
        _timer = TIM2;
    }
}

void MotorEncoder::begin() {
    if (_timer == TIM3) {
        // TIM3_CH1 (PB4), TIM3_CH2 (PB5)
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
        TIM3->SMCR = TIM_ENCODERMODE_TI12; // 0x0003: Đếm 4X trên cả 2 kênh TI1 & TI2
        TIM3->CCMR1 = 0x3131;             // Filter = 3 (lọc nhiễu motor), TI1FP1, TI2FP2
        TIM3->CCER = 0x0013;              // CC1E=1, CC1P=1 (Đảo cực tính TI1), CC2E=1: Đếm tăng khi quay tiến
        TIM3->ARR = 0xFFFF;
        TIM3->CNT = 0;
        TIM3->CR1 |= TIM_CR1_CEN;

        _lastRawCnt = TIM3->CNT;
    } else if (_timer == TIM2) {
        // TIM2_CH1 (PA15), TIM2_CH2 (PB3)
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_TIM2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
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
        TIM2->SMCR = TIM_ENCODERMODE_TI12; // 0x0003: Đếm 4X trên cả 2 kênh TI1 & TI2
        TIM2->CCMR1 = 0x3131;             // Filter = 3 (lọc nhiễu motor), TI1FP1, TI2FP2
        TIM2->CCER = 0x0013;              // CC1E=1, CC1P=1 (Đảo cực tính TI1), CC2E=1: Đếm tăng khi quay tiến
        TIM2->ARR = 0xFFFFFFFF;
        TIM2->CNT = 0;
        TIM2->CR1 |= TIM_CR1_CEN;

        _lastRawCnt = TIM2->CNT;
    }
    _totalCount = 0;
}

void MotorEncoder::update() {
    getCount();
}

long MotorEncoder::getCount() const {
    if (!_timer) return 0;

    taskENTER_CRITICAL();
    uint32_t currentRaw = _timer->CNT;
    int32_t diff = 0;

    if (_timer == TIM3) {
        // TIM3 là 16-bit timer trên STM32F411
        int16_t diff16 = (int16_t)((uint16_t)currentRaw - (uint16_t)_lastRawCnt);
        diff = diff16;
    } else {
        // TIM2 là 32-bit timer trên STM32F411
        int32_t diff32 = (int32_t)(currentRaw - _lastRawCnt);
        diff = diff32;
    }

    _lastRawCnt = currentRaw;

    if (_reverse) {
        _totalCount -= diff;
    } else {
        _totalCount += diff;
    }

    long result = _totalCount;
    taskEXIT_CRITICAL();

    return result;
}

void MotorEncoder::reset() {
    if (!_timer) return;
    taskENTER_CRITICAL();
    _timer->CNT = 0;
    _lastRawCnt = 0;
    _totalCount = 0;
    taskEXIT_CRITICAL();
}

void MotorEncoder::setReverse(bool reverse) {
    _reverse = reverse;
}


