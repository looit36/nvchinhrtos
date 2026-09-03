#pragma once

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

namespace freertospp {

class Semaphore {
 public:
  Semaphore() {
    xSemaphore = xSemaphoreCreateBinary();
  }
  ~Semaphore() {
    if (xSemaphore != NULL) {
      vSemaphoreDelete(xSemaphore);
    }
  }
  bool giveFromISR() const {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t res = xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return res == pdTRUE;
  }
  bool give() const {
    if (!xSemaphore) return false;
    return pdTRUE == xSemaphoreGive(xSemaphore);
  }
  bool take(portTickType xBlockTime = portMAX_DELAY) const {
    if (!xSemaphore) return false;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return true;
    return pdTRUE == xSemaphoreTake(xSemaphore, xBlockTime);
  }
  SemaphoreHandle_t native_handle() const {
    return xSemaphore;
  }

 private:
  SemaphoreHandle_t xSemaphore = NULL;
};

}  // namespace freertospp
