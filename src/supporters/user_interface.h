/**
 * @file user_interface.h
 * @brief User Interface handling PB1 button & Bluetooth Serial CLI
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "hardware/hardware.h"
#include "app_log.h"

class UserInterface {
 public:
  UserInterface(hardware::Hardware* hw) : hw(hw) {}

  /**
   * @brief Chờ chọn mode (0 đến range-1) qua nút bấm PB1 hoặc gõ lệnh trên Bluetooth Serial
   */
  int waitForSelect(const int range = 16, const uint8_t init_value = 0, String* out_serial_cmd = nullptr) {
    uint8_t value = init_value;
    hw->led->set(value);

    print_menu_prompt(value);

    String serialBuffer = "";
    uint32_t press_start_ms = 0;
    bool prev_button_state = false;

    while (1) {
      vTaskDelay(pdMS_TO_TICKS(10));

      // 1. KIỂM TRA NÚT BẤM PB1 (Tích cực mức CAO / HIGH khi nhấn)
      bool btn_now = (digitalRead(hw->btn->get_pin()) == HIGH);

      if (btn_now && !prev_button_state) {
        // Cạnh lên: Bắt đầu nhấn
        press_start_ms = millis();
      } else if (!btn_now && prev_button_state) {
        // Cạnh xuống: Nhả nút
        uint32_t duration = millis() - press_start_ms;
        if (duration >= 30 && duration < 1000) {
          // Click ngắn: Chuyển sang chế độ tiếp theo
          value = (value + 1) % range;
          hw->led->set(value);
          hw->bz->play(hardware::Buzzer::SELECT);
          LOGI("Selected Mode: [%d]", value);
          print_mode_name(value);
        }
        press_start_ms = 0;
      } else if (btn_now && press_start_ms > 0) {
        // Đang giữ nút: Nếu giữ lâu > 1.2s -> Xác nhận chọn mode
        if (millis() - press_start_ms > 1200) {
          LOGI(">>> CONFIRMED MODE [%d] by Button Long-Press! <<<", value);
          hw->bz->play(hardware::Buzzer::CONFIRM);
          // Chờ người dùng nhả nút
          while (digitalRead(hw->btn->get_pin()) == HIGH) {
            vTaskDelay(pdMS_TO_TICKS(20));
          }
          return value;
        }
      }
      prev_button_state = btn_now;

      // 2. KIỂM TRA LỆNH TỪ BLUETOOTH SERIAL
      while (BTSerial.available()) {
        char c = BTSerial.read();
        if (c == '\r' || c == '\n') {
          serialBuffer.trim();
          if (serialBuffer.length() > 0) {
            // Lệnh chuỗi hoàn chỉnh (MAP:..., X..., S, M,...)
            if (serialBuffer.startsWith("MAP:") || serialBuffer.startsWith("map:")) {
              if (out_serial_cmd) *out_serial_cmd = serialBuffer;
              LOGI("Received MAP string via Bluetooth (%d chars)!", serialBuffer.length());
              hw->bz->play(hardware::Buzzer::CONFIRM);
              return 7; // Chế độ Web Map
            } else if (serialBuffer.startsWith("X") || serialBuffer.startsWith("x")) {
              if (out_serial_cmd) *out_serial_cmd = serialBuffer;
              LOGI("Received Fast Path: %s", serialBuffer.substring(1).c_str());
              hw->bz->play(hardware::Buzzer::CONFIRM);
              return 1; // Fast Run
            } else if (serialBuffer.equalsIgnoreCase("M")) {
              LOGI("Selected Mode: [2] Motor Test");
              hw->bz->play(hardware::Buzzer::CONFIRM);
              return 2;
            } else if (serialBuffer.equalsIgnoreCase("S")) {
              LOGI("Selected Mode: [0] Search Run");
              hw->bz->play(hardware::Buzzer::CONFIRM);
              return 0;
            } else if (serialBuffer.startsWith("SYSID") || serialBuffer.startsWith("sysid")) {
              if (out_serial_cmd) *out_serial_cmd = serialBuffer;
              LOGI("Received SYSID command: %s", serialBuffer.c_str());
              hw->bz->play(hardware::Buzzer::CONFIRM);
              return 8; // Chế độ SysID
            } else {
              // Nhập số chế độ (0 - 15) và ấn Enter
              int num = serialBuffer.toInt();
              if (num >= 0 && num < range) {
                LOGI("Selected Mode: [%d]", num);
                hw->bz->play(hardware::Buzzer::CONFIRM);
                return num;
              }
            }
            serialBuffer = "";
          }
        } else {
          serialBuffer += c;
          // Nếu chỉ bấm 1 ký tự số duy nhất (0 đến 8) không cần Enter
          if (serialBuffer.length() == 1 && serialBuffer[0] >= '0' && serialBuffer[0] <= '8') {
            uint8_t selected = serialBuffer[0] - '0';
            serialBuffer = "";
            LOGI("Selected Mode: [%d]", selected);
            hw->bz->play(hardware::Buzzer::CONFIRM);
            return selected;
          }
        }
      }
    }
  }

  /**
   * @brief Chờ xác nhận (nhấn nút PB1 hoặc gõ phím bất kỳ trên Serial)
   */
  bool waitForCover(bool hand_cover = true) {
    (void)hand_cover;
    LOGI("Press PB1 Button or send ENTER via Bluetooth to START...");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(20));
      if (digitalRead(hw->btn->get_pin()) == HIGH) {
        hw->bz->play(hardware::Buzzer::CONFIRM);
        while (digitalRead(hw->btn->get_pin()) == HIGH) {
          vTaskDelay(pdMS_TO_TICKS(20));
        }
        return true;
      }
      if (BTSerial.available()) {
        while (BTSerial.available()) BTSerial.read();
        hw->bz->play(hardware::Buzzer::CONFIRM);
        return true;
      }
    }
  }

  bool waitForPickup(uint32_t timeout_ms = 0) {
    (void)timeout_ms;
    return false;
  }

 private:
  hardware::Hardware* hw;

  void print_mode_name(uint8_t m) {
    switch (m) {
      case 0: LOGI("-> [0] Search Run"); break;
      case 1: LOGI("-> [1] Fast Run"); break;
      case 2: LOGI("-> [2] Open-loop Motor Test"); break;
      case 3: LOGI("-> [3] Encoder Position Test"); break;
      case 4: LOGI("-> [4] IMU Calibration Test"); break;
      case 5: LOGI("-> [5] Slalom 90 Test"); break;
      case 6: LOGI("-> [6] Spin Turn 180 Test"); break;
      case 7: LOGI("-> [7] Waiting for Web Map (MAP:...)"); break;
      case 8: LOGI("-> [8] System Identification (SysID)"); break;
      default: LOGI("-> [%d] Mode", m); break;
    }
  }

  void print_menu_prompt(uint8_t cur) {
    BTSerial.println("\r\n========================================");
    BTSerial.println(">>>       STM32 KERISE v4 MENU       <<<");
    BTSerial.println("========================================");
    BTSerial.println(" [0] Search Run (S)");
    BTSerial.println(" [1] Fast Run (F/X)");
    BTSerial.println(" [2] Motor Open-Loop Test (M)");
    BTSerial.println(" [3] Encoder Position Test");
    BTSerial.println(" [4] IMU Calibration Test");
    BTSerial.println(" [5] Slalom 90 Curve Test");
    BTSerial.println(" [6] Spin Turn 180 Test");
    BTSerial.println(" [7] Receive Web Maze Map (MAP:...)");
    BTSerial.println(" [8] System Identification (SysID)");
    BTSerial.println("----------------------------------------");
    BTSerial.printf("Current: [%d]. Tap PB1 or Send (0-8/S/M/MAP/SYSID):\r\n", cur);
    BTSerial.println("Cmd: 'SYSID <dir> <duty> [ms]' (e.g. SYSID 0 0.2)");
  }
};
