#ifndef HARDWARE_MOTOR_H
#define HARDWARE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Khởi tạo Timer phần cứng (TIM4) tạo xung PWM @ 100kHz điều khiển động cơ
 */
void Motor_Initialize(void);

/**
 * @brief Tắt toàn bộ xung PWM (Dừng khẩn cấp động cơ)
 */
void Motor_StopPWM(void);

/**
 * @brief Điều khiển Duty Motor Trái theo thang [-1000, 1000] tương ứng [-100.0%, +100.0%]
 * @param duty_l Giá trị duty (-1000 đến 1000)
 */
void Motor_SetDuty_Left(int16_t duty_l);

/**
 * @brief Điều khiển Duty Motor Phải theo thang [-1000, 1000] tương ứng [-100.0%, +100.0%]
 * @param duty_r Giá trị duty (-1000 đến 1000)
 */
void Motor_SetDuty_Right(int16_t duty_r);

#ifdef __cplusplus
}
#endif

#endif // HARDWARE_MOTOR_H

