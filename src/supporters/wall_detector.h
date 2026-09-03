/**
 * @file wall_detector.h
 * @brief Wall Detector Interface
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include "hardware/hardware.h"
#include "app_log.h"

class WallDetector {
 public:
  static constexpr float Ts = 1e-3f;

  union WallValue {
    struct {
      float side[2];
      float front[2];
    };
    float value[4] = {0, 0, 0, 0};
  };

  WallValue distance;
  WallValue distance_average;
  bool is_wall[3] = {false, false, false}; // Left, Front, Right
  bool is_wall_center[2] = {false, false};

 public:
  WallDetector(hardware::Hardware* hw) : hw(hw) {}

  bool init() {
    return true;
  }

  void update() {
    // Khi chưa có cảm biến quang thực tế, giữ trạng thái an toàn
  }

  void print() {
    LOGI("Wall: L:%c F:%c R:%c",
         is_wall[0] ? 'X' : '_',
         is_wall[1] ? 'X' : '_',
         is_wall[2] ? 'X' : '_');
  }

 private:
  hardware::Hardware* hw;
};
