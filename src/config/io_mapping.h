/**
 * @file io_mapping.h
 * @brief Pin IO Mapping for STM32F411 MicroMouse
 */
#pragma once

#include <Arduino.h>

// ==========================================
// MOTOR DRIVER (DRV8833 - Hardware TIM4 PWM)
// ==========================================
#define MOTOR_L_IN1 PB8   // TIM4_CH3
#define MOTOR_L_IN2 PB9   // TIM4_CH4
#define MOTOR_R_IN1 PB6   // TIM4_CH1
#define MOTOR_R_IN2 PB7   // TIM4_CH2

// Motor direction polarity (+1: normal, -1: reversed)
#define MOTOR_L_DIR 1
#define MOTOR_R_DIR -1

// ==========================================
// ENCODERS (MT6701 ABZ - Hardware Timer Decode 4X)
// ==========================================
#define ENCODER_L_A PB4   // TIM3_CH1
#define ENCODER_L_B PB5   // TIM3_CH2
#define ENCODER_R_A PA15  // TIM2_CH1
#define ENCODER_R_B PB3   // TIM2_CH2

// ==========================================
// IMU (BMI160 - Hardware SPI2)
// ==========================================
#define IMU_SPI_CS   PB12
#define IMU_SPI_SCK  PB13
#define IMU_SPI_MISO PB14
#define IMU_SPI_MOSI PB15

// ==========================================
// BATTERY SENSING (ADC1_IN8 via DMA)
// ==========================================
#define BAT_VOL_PIN PB0   // ADC1 Channel 8

// ==========================================
// USER INTERFACE (Button & LED)
// ==========================================
#define BUTTON_PIN PB1    // Mode INPUT (No pull-up, No pull-down)
#define LED_PIN    PC13   // On-board LED

// ==========================================
// SERIAL BLUETOOTH
// ==========================================
#define BT_SERIAL_TX PA9  // USART1 TX
#define BT_SERIAL_RX PA10 // USART1 RX
