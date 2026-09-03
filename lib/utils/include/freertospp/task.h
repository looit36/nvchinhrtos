#pragma once

#include <FreeRTOS.h>
#include <task.h>

#ifndef tskNO_AFFINITY
#define tskNO_AFFINITY 0
#endif

#ifndef PRO_CPU_NUM
#define PRO_CPU_NUM 0
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM 0
#endif

namespace freertospp {

template <typename T>
class Task {
 public:
  Task() : pxCreatedTask(NULL), obj(NULL), func(NULL) {}
  ~Task() { terminate(); }

  bool start(T* obj,
             void (T::*func)(),
             const char* const pcName,
             unsigned short usStackDepth = configMINIMAL_STACK_SIZE,
             unsigned portBASE_TYPE uxPriority = 0,
             const BaseType_t xCoreID = tskNO_AFFINITY) {
    (void)xCoreID;
    this->obj = obj;
    this->func = func;
    if (pxCreatedTask != NULL) {
      return false;
    }
    BaseType_t result =
        xTaskCreate(entry_point, pcName, usStackDepth, this,
                    uxPriority, &pxCreatedTask);
    return result == pdPASS;
  }

  void terminate() {
    if (pxCreatedTask == NULL)
      return;
    vTaskDelete(pxCreatedTask);
    pxCreatedTask = NULL;
  }

  TaskHandle_t get_handle() const {
    return pxCreatedTask;
  }

 private:
  TaskHandle_t pxCreatedTask = NULL;
  T* obj = NULL;
  void (T::*func)() = NULL;

  static void entry_point(void* arg) {
    auto task_obj = static_cast<Task*>(arg);
    (task_obj->obj->*task_obj->func)();
  }
};

}  // namespace freertospp
