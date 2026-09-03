#include <Arduino.h>
#include <FreeRTOS.h>
#include "machine/machine.h"
#include "app_log.h"
#include "config/io_mapping.h"

machine::Machine* mach = nullptr;

extern "C" {
int _open(const char *path, int flags, ...) {
  (void)path; (void)flags;
  return -1;
}
int _unlink(const char *path) {
  (void)path;
  return -1;
}
}

void setup() {
  // 1. Khởi tạo PC13 LED & chớp 3 lần để báo hiệu MCU sống và reset thành công
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, LOW);  // LED ON (active-low)
    delay(60);
    digitalWrite(LED_PIN, HIGH); // LED OFF
    delay(60);
  }

  // 2. Khởi tạo Bluetooth Serial (USART1 PA9/PA10 @ 115200)
  BTSerial.begin(115200);
  delay(100);

  BTSerial.println("\r\n\r\n>>> STM32F411CE BOOTING <<<");

  // 3. Khởi tạo Machine Coordinator
  mach = new machine::Machine();
  mach->init();

  // 4. Bật LED báo hiệu sẵn sàng chạy FreeRTOS Scheduler
  digitalWrite(LED_PIN, LOW);

  // 5. Bắt đầu bộ lập lịch FreeRTOS
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS quản lý toàn bộ hệ thống
}