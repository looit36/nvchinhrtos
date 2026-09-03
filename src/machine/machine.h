/**
 * @file machine.h
 * @brief MicroMouse Machine Coordinator for STM32 Kerise v4
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include "agents/maze_robot.h"
#include "agents/move_action.h"
#include "hardware/hardware.h"
#include "supporters/supporters.h"
#include <freertospp/task.h>
#include "app_log.h"
#include <MazeLib/SearchAlgorithm.h>

namespace machine {

class Machine {
 public:
  Machine() {}

  bool init() {
    bool result = true;

    LOGI("==========================================");
    LOGI("   STARTING STM32 KERISE v4 FIRMWARE      ");
    LOGI("   CPU Frequency: %lu MHz", HAL_RCC_GetSysClockFreq() / 1000000UL);
    LOGI("   SegWidthFull:  %.1f mm", (double)field::SegWidthFull);
    LOGI("==========================================");

    // 1. Hardware
    hw = new hardware::Hardware();
    if (!hw->init()) {
      result = false;
    }

    // 2. Supporters
    sp = new supporters::Supporters(hw);
    if (!sp->init()) {
      result = false;
    }

    // 3. Agents
    ma = new MoveAction(hw, sp, model::TrajectoryTrackerGain);
    mr = new MazeRobot(hw, sp, ma);

    // 4. Logger
    lgr = new Logger();

    // 5. Start tasks
    task_drive.start(this, &Machine::drive, "Drive", 2048, 2);
    task_print.start(this, &Machine::print, "Print", 1024, 1);

    if (!result) {
      hw->bz->play(hardware::Buzzer::ERROR);
      LOGE("Machine init warning (some peripherals not ready)!");
    } else {
      LOGI("Machine initialized successfully!");
    }
    return true;
  }

  void drive() {
    while (1) {
      driveManually();
    }
  }

  void driveManually() {
    String serial_cmd = "";
    int mode = sp->ui->waitForSelect(8, 0, &serial_cmd);

    if (serial_cmd.length() > 0) {
      parse_serial_command(serial_cmd);
    } else {
      switch (mode) {
        case 0:
          search_run();
          break;
        case 1:
          fast_run();
          break;
        case 2:
          motor_direction_test();
          break;
        case 3:
          encoder_test();
          break;
        case 4:
          imu_test();
          break;
        case 5:
          slalom_test();
          break;
        case 6:
          spin_turn_test();
          break;
        case 7:
          receive_web_map_blocking();
          break;
        default:
          LOGI("Selected mode %d. Ready.", mode);
          break;
      }
    }
  }

  void print() {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz telemetry

      // Telemetry stream cho Teleplot khi robot đang chạy điều khiển
      if (sp->sc->is_enabled()) {
        BTSerial.printf(">v_ref:%.1f\n", (double)sp->sc->ref_v.tra);
        BTSerial.printf(">v_est:%.1f\n", (double)sp->sc->est_v.tra);
        BTSerial.printf(">w_ref:%.2f\n", (double)sp->sc->ref_v.rot);
        BTSerial.printf(">w_est:%.2f\n", (double)sp->sc->est_v.rot);
        BTSerial.printf(">pose_x:%.1f\n", (double)sp->sc->est_p.x);
        BTSerial.printf(">pose_y:%.1f\n", (double)sp->sc->est_p.y);
        BTSerial.printf(">pose_th:%.2f\n", (double)(sp->sc->est_p.th * 180.0f / PI));
      }
    }
  }

 private:
  freertospp::Task<Machine> task_drive;
  freertospp::Task<Machine> task_print;

  hardware::Hardware* hw = nullptr;
  supporters::Supporters* sp = nullptr;
  MoveAction* ma = nullptr;
  MazeRobot* mr = nullptr;
  Logger* lgr = nullptr;

  std::string last_web_map_path = "";

  void search_run() {
    LOGI(">>> MODE 0: SEARCH RUN <<<");
    if (!sp->ui->waitForCover()) return;
    vTaskDelay(pdMS_TO_TICKS(500));
    ma->enable(MoveAction::TaskActionSearchRun);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::START_STEP);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::ST_HALF);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::ST_FULL);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::TURN_L);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::ST_FULL);
    ma->enqueue_action(MazeLib::RobotBase::SearchAction::ST_HALF_STOP);
    ma->waitForEndAction();
    ma->disable();
    hw->bz->play(hardware::Buzzer::SUCCESSFUL);
  }

  void fast_run() {
    LOGI(">>> MODE 1: FAST RUN <<<");
    if (last_web_map_path.empty()) {
      LOGW("No fast path loaded! Setting default test path: sSs");
      last_web_map_path = "sSs";
    }
    LOGI("Path: %s", last_web_map_path.c_str());
    if (!sp->ui->waitForCover()) return;
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->mt->emergency_release();
    ma->set_fast_path(last_web_map_path);
    ma->enable(MoveAction::TaskActionFastRun);
    ma->waitForEndAction();
    ma->disable();
    hw->bz->play(hardware::Buzzer::COMPLETE);
  }

  void motor_direction_test() {
    LOGI(">>> MODE 2: OPEN-LOOP MOTOR DIRECTION TEST <<<");
    LOGW("PLEASE LIFT ROBOT OFF THE GROUND!");
    if (!sp->ui->waitForCover()) return;

    const float test_duty = 0.35f;
    const int duration = 500;

    // Left Forward
    LOGI("[1/4] Left Forward (+0.35)...");
    int32_t enc0 = hw->enc->get_total_count(0);
    hw->mt->drive(test_duty, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(duration));
    hw->mt->free();
    vTaskDelay(pdMS_TO_TICKS(200));
    LOGI("  -> EncLeft Delta: %ld", hw->enc->get_total_count(0) - enc0);

    // Left Backward
    LOGI("[2/4] Left Backward (-0.35)...");
    enc0 = hw->enc->get_total_count(0);
    hw->mt->drive(-test_duty, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(duration));
    hw->mt->free();
    vTaskDelay(pdMS_TO_TICKS(200));
    LOGI("  -> EncLeft Delta: %ld", hw->enc->get_total_count(0) - enc0);

    // Right Forward
    LOGI("[3/4] Right Forward (+0.35)...");
    int32_t enc1 = hw->enc->get_total_count(1);
    hw->mt->drive(0.0f, test_duty);
    vTaskDelay(pdMS_TO_TICKS(duration));
    hw->mt->free();
    vTaskDelay(pdMS_TO_TICKS(200));
    LOGI("  -> EncRight Delta: %ld", hw->enc->get_total_count(1) - enc1);

    // Right Backward
    LOGI("[4/4] Right Backward (-0.35)...");
    enc1 = hw->enc->get_total_count(1);
    hw->mt->drive(0.0f, -test_duty);
    vTaskDelay(pdMS_TO_TICKS(duration));
    hw->mt->free();
    vTaskDelay(pdMS_TO_TICKS(200));
    LOGI("  -> EncRight Delta: %ld", hw->enc->get_total_count(1) - enc1);

    hw->bz->play(hardware::Buzzer::CONFIRM);
  }

  void encoder_test() {
    LOGI(">>> MODE 3: ENCODER TEST (Push robot to see positions) <<<");
    LOGI("Press PB1 Button or send any character via Serial to stop.");
    hw->enc->clear_offset();
    while (digitalRead(hw->btn->get_pin()) == LOW && !BTSerial.available()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      LOGI("Enc Pos: L=%.2f mm, R=%.2f mm (Counts: L=%ld, R=%ld)",
           (double)hw->enc->get_position(0), (double)hw->enc->get_position(1),
           hw->enc->get_total_count(0), hw->enc->get_total_count(1));
    }
    while (digitalRead(hw->btn->get_pin()) == HIGH) vTaskDelay(pdMS_TO_TICKS(20));
    while (BTSerial.available()) BTSerial.read();
  }

  void imu_test() {
    LOGI(">>> MODE 4: IMU TEST <<<");
    LOGI("Press PB1 Button or send any character via Serial to stop.");
    hw->imu->calibration();
    while (digitalRead(hw->btn->get_pin()) == LOW && !BTSerial.available()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      hw->imu->print();
    }
    while (digitalRead(hw->btn->get_pin()) == HIGH) vTaskDelay(pdMS_TO_TICKS(20));
    while (BTSerial.available()) BTSerial.read();
  }

  void slalom_test() {
    LOGI(">>> MODE 5: SLALOM 90 TEST <<<");
    if (!sp->ui->waitForCover()) return;
    vTaskDelay(pdMS_TO_TICKS(500));
    ma->set_fast_path("sLs");
    ma->enable(MoveAction::TaskActionFastRun);
    ma->waitForEndAction();
    ma->disable();
  }

  void spin_turn_test() {
    LOGI(">>> MODE 6: SPIN TURN 180 TEST <<<");
    if (!sp->ui->waitForCover()) return;
    vTaskDelay(pdMS_TO_TICKS(500));
    sp->sc->enable();
    vTaskDelay(pdMS_TO_TICKS(50));
    ctrl::AccelDesigner ad(model::spin_jerk, model::spin_alpha, model::spin_omega, 0, 0, PI);
    for (float t = 0; t < ad.t_end(); t += sp->sc->Ts) {
      sp->sc->sampling_sync();
      sp->sc->set_target(0, ad.v(t), 0, ad.a(t));
    }
    sp->sc->set_target(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    sp->sc->disable();
    LOGI("Turn Finished. Final Yaw Angle: %.2f deg", (double)(hw->imu->get_angle() * 180.0f / PI));
  }

  void receive_web_map_blocking() {
    LOGI(">>> MODE 7: WAITING FOR WEB MAZE DESIGNER MAP (MAP:...) <<<");
    LOGI("Paste MAP:... string now or press PB1 to cancel.");
    String mapBuf = "";
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(15));
      if (digitalRead(hw->btn->get_pin()) == HIGH) {
        while (digitalRead(hw->btn->get_pin()) == HIGH) vTaskDelay(pdMS_TO_TICKS(20));
        return;
      }
      while (BTSerial.available()) {
        char c = BTSerial.read();
        if (c == '\r' || c == '\n') {
          mapBuf.trim();
          if (mapBuf.length() > 0) {
            parse_serial_command(mapBuf);
            return;
          }
        } else {
          mapBuf += c;
        }
      }
    }
  }

  void parse_serial_command(const String& cmdStr) {
    if (cmdStr.startsWith("MAP:") || cmdStr.startsWith("map:")) {
      String mapHex = cmdStr.substring(4);
      if (mapHex.length() >= 260) {
        int startX = (mapHex.charAt(0) >= 'A') ? (mapHex.charAt(0) - 'A' + 10) : (mapHex.charAt(0) - '0');
        int startY = (mapHex.charAt(1) >= 'A') ? (mapHex.charAt(1) - 'A' + 10) : (mapHex.charAt(1) - '0');
        int goalX  = (mapHex.charAt(2) >= 'A') ? (mapHex.charAt(2) - 'A' + 10) : (mapHex.charAt(2) - '0');
        int goalY  = (mapHex.charAt(3) >= 'A') ? (mapHex.charAt(3) - 'A' + 10) : (mapHex.charAt(3) - '0');

        MazeLib::Positions goals;
        goals.push_back(MazeLib::Position(goalX, goalY));

        MazeLib::Position currentPos(startX, startY);
        MazeLib::Maze maze(goals, currentPos);
        maze.reset();

        int idx = 4;
        for (int x = 0; x < 16; x++) {
          for (int y = 0; y < 16; y++) {
            char c = mapHex.charAt(idx++);
            uint8_t val = 0;
            if (c >= '0' && c <= '9') val = c - '0';
            else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;

            bool hasE = (val & 1);
            bool hasN = (val & 2);
            bool hasW = (val & 4);
            bool hasS = (val & 8);

            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::East, hasE, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::North, hasN, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::West, hasW, false);
            maze.updateWall(MazeLib::Position(x, y), MazeLib::Direction::South, hasS, false);

            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::East, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::North, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::West, true);
            maze.setKnown(MazeLib::Position(x, y), MazeLib::Direction::South, true);
          }
        }

        maze.updateWall(currentPos, MazeLib::Direction::East, true, true);
        maze.updateWall(currentPos, MazeLib::Direction::West, true, true);
        maze.updateWall(currentPos, MazeLib::Direction::South, true, true);

        MazeLib::SearchAlgorithm searcher(maze);
        MazeLib::Directions shortestPath;
        bool success = searcher.calcShortestDirections(shortestPath, true);

        if (success) {
          std::string keriseStr = "";
          if (!shortestPath.empty()) {
            MazeLib::Direction robotDir = shortestPath[0];
            for (size_t i = 1; i < shortestPath.size(); ++i) {
              MazeLib::Direction nextDir = shortestPath[i];
              int diff = (nextDir - robotDir + 8) % 8;
              if (diff == 0) keriseStr += "S";
              else if (diff == 6) keriseStr += "R";
              else if (diff == 2) keriseStr += "L";
              else if (diff == 4) keriseStr += "B";
              robotDir = nextDir;
            }
          }
          last_web_map_path = "s" + keriseStr + "s";
          LOGI("Shortest Path Generated: %s", last_web_map_path.c_str());
          hw->bz->play(hardware::Buzzer::SUCCESSFUL);
          // Cho phép chạy luôn fast run sau khi giải mê cung thành công
          fast_run();
        } else {
          LOGW("Search failed: No valid path to goal!");
          hw->bz->play(hardware::Buzzer::ERROR);
        }
      }
    } else if (cmdStr.startsWith("X") || cmdStr.startsWith("x")) {
      last_web_map_path = cmdStr.substring(1).c_str();
      fast_run();
    } else if (cmdStr.equalsIgnoreCase("M")) {
      motor_direction_test();
    } else if (cmdStr.equalsIgnoreCase("S")) {
      search_run();
    }
  }
};

}  // namespace machine
