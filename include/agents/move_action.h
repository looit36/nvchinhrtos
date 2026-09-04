#pragma once

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <string>

class MoveAction {
public:
    QueueHandle_t commandQueue;

    void init();
    void push_command(char cmd);
    void start_fast_run(const char* search_path);

private:
    std::string convert_search_to_fast(std::string search_path);
    void execute_fast_run(std::string fast_path);

    static void task_trampoline(void *pvParameters);
    void task();
};

extern MoveAction moveAction;

