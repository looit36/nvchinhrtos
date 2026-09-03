/**
 * @file supporters.h
 * @brief Supporters Manager (UI, SpeedController, WallDetector)
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include "supporters/logger.h"
#include "supporters/speed_controller.h"
#include "supporters/user_interface.h"
#include "supporters/wall_detector.h"

namespace supporters {

class Supporters {
 private:
  hardware::Hardware* hw;

 public:
  UserInterface* ui;
  SpeedController* sc;
  WallDetector* wd;

 public:
  Supporters(hardware::Hardware* hw)
      : hw(hw),
        ui(new UserInterface(hw)),
        sc(new SpeedController(hw)),
        wd(new WallDetector(hw)) {}

  bool init() {
    if (!wd->init()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      LOGE("WallDetector init failed!");
      return false;
    }
    if (!sc->init()) {
      hw->bz->play(hardware::Buzzer::ERROR);
      LOGE("SpeedController init failed!");
      return false;
    }
    return true;
  }
};

}  // namespace supporters
