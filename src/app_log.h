/**
 * @file app_log.h
 * @brief Logging utilities for STM32 Kerise
 */
#pragma once

#include <Arduino.h>
#include <cstdio>
#include <iostream>

#define BTSerial Serial1

#define APP_STRINGIFY(n) #n
#define APP_TOSTRING(n) APP_STRINGIFY(n)

#ifndef LOG_COMMON
#define LOG_COMMON(l, c, f, ...)                                           \
  do {                                                                      \
    char _log_buf[256];                                                     \
    snprintf(_log_buf, sizeof(_log_buf), c "[" l "][" __FILE__ ":" APP_TOSTRING(__LINE__) "]\x1b[0m\t" f "\r\n", ##__VA_ARGS__); \
    BTSerial.print(_log_buf);                                              \
  } while (0)
#endif

#define LOGD(fmt, ...) LOG_COMMON("D", "\x1b[34m", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_COMMON("I", "\x1b[32m", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_COMMON("W", "\x1b[33m", fmt, ##__VA_ARGS__)
#define LOGE(fmt, ...) LOG_COMMON("E", "\x1b[31m", fmt, ##__VA_ARGS__)

#ifndef app_logd
#define app_logd std::ostream(0)
#endif
#ifndef app_logi
#define app_logi std::cout
#endif
#ifndef app_logw
#define app_logw std::cout
#endif
#ifndef app_loge
#define app_loge std::cerr
#endif
