#include "hardware/motor.h"
#include <Arduino.h>
#include "config.h"

/* ---------------------------------------------------------------
    Khởi tạo TIM4 Hardware PWM @ 100kHz trên 4 kênh PB6, PB7, PB8, PB9
--------------------------------------------------------------- */
void Motor_Initialize(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // PB6 (TIM4_CH1), PB7 (TIM4_CH2), PB8 (TIM4_CH3), PB9 (TIM4_CH4)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    uint32_t timer_clk = HAL_RCC_GetSysClockFreq(); // 100MHz trên STM32F411
    uint32_t period = (timer_clk / MOTOR_PWM_FREQ);  // 1000 cho 100kHz PWM chuẩn Zirconia
    if (period < 10) period = 1000;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->PSC = 0;
    TIM4->ARR = period - 1; // 999 cho 1000 ticks resolution

    // PWM mode 1 + Preload enable trên CH1, CH2, CH3, CH4
    TIM4->CCMR1 = 0x6868;
    TIM4->CCMR2 = 0x6868;

    // Bật Compare Output (CC1E, CC2E, CC3E, CC4E)
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;

    // Kích hoạt Timer với Auto-reload preload
    TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

/* ---------------------------------------------------------------
    Tắt toàn bộ xung PWM (Dừng motor)
--------------------------------------------------------------- */
void Motor_StopPWM(void)
{
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
}

/* ---------------------------------------------------------------
    Điều khiển Duty Motor Trái theo thang [-1000, 1000]
--------------------------------------------------------------- */
void Motor_SetDuty_Left(int16_t duty_l)
{
    duty_l = duty_l * MOTOR_L_DIR;

    if (duty_l == 0)
    {
        TIM4->CCR3 = 0;
        TIM4->CCR4 = 0;
        return;
    }

    int16_t abs_duty = abs(duty_l);
    if (abs_duty > MOT_DUTY_MAX)
    {
        abs_duty = MOT_DUTY_MAX;
    }
    else if (abs_duty < MOT_DUTY_MIN)
    {
        abs_duty = MOT_DUTY_MIN;
    }

    uint32_t pulse = (uint32_t)((TIM4->ARR + 1) * abs_duty / 1000);

    if (duty_l > 0)
    {
        TIM4->CCR3 = pulse; // PB8 (TIM4_CH3) = PWM
        TIM4->CCR4 = 0;     // PB9 (TIM4_CH4) = 0
    }
    else
    {
        TIM4->CCR3 = 0;     // PB8 (TIM4_CH3) = 0
        TIM4->CCR4 = pulse; // PB9 (TIM4_CH4) = PWM
    }
}

/* ---------------------------------------------------------------
    Điều khiển Duty Motor Phải theo thang [-1000, 1000]
--------------------------------------------------------------- */
void Motor_SetDuty_Right(int16_t duty_r)
{
    duty_r = duty_r * MOTOR_R_DIR;

    if (duty_r == 0)
    {
        TIM4->CCR1 = 0;
        TIM4->CCR2 = 0;
        return;
    }

    int16_t abs_duty = abs(duty_r);
    if (abs_duty > MOT_DUTY_MAX)
    {
        abs_duty = MOT_DUTY_MAX;
    }
    else if (abs_duty < MOT_DUTY_MIN)
    {
        abs_duty = MOT_DUTY_MIN;
    }

    uint32_t pulse = (uint32_t)((TIM4->ARR + 1) * abs_duty / 1000);

    if (duty_r > 0)
    {
        TIM4->CCR1 = pulse; // PB6 (TIM4_CH1) = PWM
        TIM4->CCR2 = 0;     // PB7 (TIM4_CH2) = 0
    }
    else
    {
        TIM4->CCR1 = 0;     // PB6 (TIM4_CH1) = 0
        TIM4->CCR2 = pulse; // PB7 (TIM4_CH2) = PWM
    }
}
