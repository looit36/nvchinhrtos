#pragma once

#include <FreeRTOS.h>
#include <task.h>

class Commander {
public:
    void init();
private:
    static void serial_task_trampoline(void *pvParameters);
    static void teleplot_task_trampoline(void *pvParameters);
    void serial_task();
    void teleplot_task();
    void run_motor_direction_test();
};

extern Commander commander;

