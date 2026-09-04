#include <Arduino.h>
#include <FreeRTOS.h>
#include "config.h"
#include "MotorEncoder.h"
#include <mpu.h>

#include "supporters/speed_controller.h"
#include "agents/move_action.h"
#include "agents/commander.h"

MotorEncoder encLeft(TIM3, false);
MotorEncoder encRight(TIM2, false);

volatile float g_robot_angle_mpu = 0;

// ==========================================
// SETUP
// ==========================================
void setup()
{
    // Serial.begin(115200);
    BTSerial.begin(115200);
    // In ra thông tin Clock
    uint32_t sysclk_hz = HAL_RCC_GetSysClockFreq();
    uint32_t sysclk_source = __HAL_RCC_GET_SYSCLK_SOURCE();
    String source_str = "Unknown";

    if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSI)
    {
        source_str = "HSI";
    }
    else if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_HSE)
    {
        source_str = "HSE";
    }
    else if (sysclk_source == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
    {
        if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)
        {
            source_str = "HSE (via PLL)";
        }
        else
        {
            source_str = "HSI (via PLL)";
        }
    }

    BTSerial.println("=============================");
    BTSerial.print("System Clock Source: ");
    BTSerial.println(source_str);
    BTSerial.print("System Clock Freq: ");
    BTSerial.print(sysclk_hz);
    BTSerial.println(" Hz");
    BTSerial.println("=============================");

    // Set ADC Resolution lên 12-bit (0-4095) để khớp với thông số tính vbat trong config.h
    analogReadResolution(12);

    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);

    // Khởi tạo IMU
    setup_mpu_manual();
    reset_fused_data();

    // Khởi tạo Encoder phần cứng (TIM3 & TIM2)
    encLeft.begin();
    encRight.begin();

    // Khởi tạo các Modules (giống Kerise: hw->init(), supporters->init(), agents->init())
    speedCtrl.init();
    moveAction.init();
    commander.init();

    // Bắt đầu chạy FreeRTOS Scheduler
    vTaskStartScheduler();
}

void loop()
{
    // FreeRTOS quản lý các task, loop() để trống.
}
