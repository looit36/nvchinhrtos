#pragma once

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

namespace freertospp {

class Mutex {
 public:
  Mutex() {
    xSemaphore = xSemaphoreCreateMutex();
  }
  ~Mutex() {
    if (xSemaphore != NULL) {
      vSemaphoreDelete(xSemaphore);
    }
  }
  void lock() {
    if (xSemaphore && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
    }
  }
  void unlock() {
    if (xSemaphore && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      xSemaphoreGive(xSemaphore);
    }
  }
  bool try_lock() {
    if (!xSemaphore) return false;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return true;
    return pdTRUE == xSemaphoreTake(xSemaphore, 0);
  }
  SemaphoreHandle_t native_handle() const {
    return xSemaphore;
  }

 private:
  SemaphoreHandle_t xSemaphore = NULL;
};

}  // namespace freertospp

using Mutex = freertospp::Mutex;
