#pragma once

#include <FreeRTOS.h>
#include <task.h>

namespace freertospp {

inline void sleep_ms(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

}  // namespace freertospp
