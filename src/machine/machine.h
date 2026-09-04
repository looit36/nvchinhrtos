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
    int mode = sp->ui->waitForSelect(9, 0, &serial_cmd);

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
        case 8:
          run_sysid(0, 0.2f, 1000);
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
      LOGW("No fast path loaded! Setting default test path: S");
      last_web_map_path = "S";
    }
    LOGI("Path: %s", last_web_map_path.c_str());
    LOGI("READY! Place robot at start position, then press PB1 or send ENTER via Bluetooth to RUN...");
    // Xóa các ký tự \r\n thừa từ trước khi chờ xác nhận chạy
    while (BTSerial.available()) BTSerial.read();

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

  struct SysidSample {
    float enc0;
    float enc1;
    float gyro_z;
    float accel_y;
    float angular_accel;
    float u_tra;
    float u_rot;
    float vbat;
  };

  void run_sysid(int dir, float duty, int duration_ms = 1000) {
    if (duty > 1.0f) duty /= 100.0f;
    if (duty < 0.05f) duty = 0.05f;
    if (duty > 0.80f) duty = 0.80f;
    if (duration_ms < 200) duration_ms = 200;
    if (duration_ms > 3000) duration_ms = 3000;

    int sample_count = duration_ms; // 1 mẫu/ms @ 1kHz

    LOGI("==========================================");
    LOGI(">>> STARTING SYSTEM IDENTIFICATION (SysID) <<<");
    LOGI("Mode: %s | Duty: %.2f (%.0f%%) | Duration: %d ms",
         (dir == 1 ? "ROTATIONAL (SPIN)" : "TRANSLATIONAL (STRAIGHT)"),
         (double)duty, (double)(duty * 100.0f), duration_ms);
    LOGW("CAUTION: Lift wheels off table or place in clear area!");
    LOGI("==========================================");

    // Cấp phát bộ nhớ đệm mẫu trên HEAP (~32 KB)
    SysidSample* log_buf = new (std::nothrow) SysidSample[sample_count];
    if (!log_buf) {
      LOGE("Out of memory: Failed to allocate SysID buffer (%d samples)!", sample_count);
      hw->bz->play(hardware::Buzzer::ERROR);
      return;
    }

    // 1. Dừng điều khiển vòng kín & reset cảm biến
    sp->sc->disable();
    hw->mt->drive(0, 0);
    hw->mt->free();
    hw->imu->reset_angle();
    hw->enc->clear_offset();

    // 2. Tiếng còi báo hiệu chuẩn bị chạy
    hw->bz->play(hardware::Buzzer::SELECT);
    vTaskDelay(pdMS_TO_TICKS(500));
    hw->bz->play(hardware::Buzzer::CONFIRM);
    vTaskDelay(pdMS_TO_TICKS(500));

    float vbat_now = hardware::Hardware::getBatteryVoltage();

    // 3. Cấp xung PWM bước thang và thu thập dữ liệu 1kHz
    float u_l = (dir == 1) ? -duty : duty;
    float u_r = duty;
    float u_tra_val = (dir == 0) ? duty : 0.0f;
    float u_rot_val = (dir == 1) ? duty : 0.0f;

    hw->mt->drive(u_l, u_r);

    for (int i = 0; i < sample_count; i++) {
      sp->sc->sampling_sync();

      log_buf[i].enc0 = hw->enc->get_position(0); // Quãng đường mm (chuẩn Kerise)
      log_buf[i].enc1 = hw->enc->get_position(1); // Quãng đường mm (chuẩn Kerise)
      log_buf[i].gyro_z = hw->imu->get_gyro();
      log_buf[i].accel_y = hw->imu->get_accel();
      log_buf[i].angular_accel = hw->imu->get_angular_accel();
      log_buf[i].u_tra = u_tra_val;
      log_buf[i].u_rot = u_rot_val;
      log_buf[i].vbat = hw->getBatteryVoltage(); // Đọc trực tiếp từ bộ đệm ADC1 DMA (<0.1us, 1000Hz liên tục)
    }

    // 4. Ngắt motor ngay sau khi đủ số mẫu
    hw->mt->drive(0, 0);
    hw->mt->free();
    hw->bz->play(hardware::Buzzer::COMPLETE);
    vTaskDelay(pdMS_TO_TICKS(200));

    LOGI("SysID finished! Streaming %d samples in Teleplot format...", sample_count);

    // 5. In dữ liệu dạng Teleplot stream cho VS Code Teleplot Extension
    // Điều tiết tốc độ truyền phù hợp với Baudrate 115200 (~11.5 bytes/ms)
    // Mỗi mẫu ~130-140 bytes -> cần delay ~12ms để buffer UART không bị tràn / rớt ký tự
    for (int i = 0; i < sample_count; i++) {
      BTSerial.printf(">enc0:%.2f\n>enc1:%.2f\n>gyro_z:%.4f\n>accel_y:%.2f\n>angular_accel:%.2f\n>u_tra:%.3f\n>u_rot:%.3f\n>vbat:%.2f\n",
                      (double)log_buf[i].enc0,
                      (double)log_buf[i].enc1,
                      (double)log_buf[i].gyro_z,
                      (double)log_buf[i].accel_y,
                      (double)log_buf[i].angular_accel,
                      (double)log_buf[i].u_tra,
                      (double)log_buf[i].u_rot,
                      (double)log_buf[i].vbat);

      // In thêm vận tốc ước lượng để hiển thị trực tiếp đồ thị bứt tốc trên Teleplot
      if (i > 0) {
        float d_tra = ((log_buf[i].enc0 - log_buf[i - 1].enc0) + (log_buf[i].enc1 - log_buf[i - 1].enc1)) * 0.5f;
        float d_rot = ((log_buf[i].enc1 - log_buf[i - 1].enc1) - (log_buf[i].enc0 - log_buf[i - 1].enc0)) * 0.5f;
        float v_tra = d_tra / 0.001f;
        float w_rot = d_rot / 0.001f / model::RotationRadius;
        BTSerial.printf(">v_tra:%.1f\n>w_rot:%.2f\n", (double)v_tra, (double)w_rot);
      }

      // Delay 12ms mỗi mẫu đảm bảo Bluetooth 115200 truyền hết sạch 100% không mất 1 byte nào
      vTaskDelay(pdMS_TO_TICKS(12));
    }

    LOGI(">>> SysID Teleplot Stream Finished <<<");

    delete[] log_buf;
  }

  void receive_web_map_blocking() {
    LOGI(">>> MODE 7: WAITING FOR WEB MAZE DESIGNER MAP (MAP:...) <<<");
    LOGI("Paste MAP:... string now or press PB1 to cancel.");

    // Dọn sạch các ký tự \r, \n hoặc khoảng trắng dư thừa từ trước
    vTaskDelay(pdMS_TO_TICKS(20));
    while (BTSerial.available()) {
      char peekChar = BTSerial.peek();
      if (peekChar == '\r' || peekChar == '\n' || peekChar == ' ') {
        BTSerial.read();
      } else {
        break;
      }
    }

    String mapBuf = "";
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(5));

      if (digitalRead(hw->btn->get_pin()) == HIGH) {
        LOGI("Mode 7 cancelled by button.");
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
      mapHex.trim();
      LOGI("Received MAP data length: %d chars", mapHex.length());

      if (mapHex.length() < 260) {
        LOGE("MAP hex string too short! Expected >= 260, got: %d", mapHex.length());
        LOGE("Please re-send MAP string from Web Maze Designer.");
        hw->bz->play(hardware::Buzzer::ERROR);
        return;
      }

      int startX = (mapHex.charAt(0) >= 'A') ? (mapHex.charAt(0) - 'A' + 10) : (mapHex.charAt(0) - '0');
      int startY = (mapHex.charAt(1) >= 'A') ? (mapHex.charAt(1) - 'A' + 10) : (mapHex.charAt(1) - '0');
      int goalX  = (mapHex.charAt(2) >= 'A') ? (mapHex.charAt(2) - 'A' + 10) : (mapHex.charAt(2) - '0');
      int goalY  = (mapHex.charAt(3) >= 'A') ? (mapHex.charAt(3) - 'A' + 10) : (mapHex.charAt(3) - '0');
      LOGI("Maze Config: Start(%d, %d) -> Goal(%d, %d)", startX, startY, goalX, goalY);

      MazeLib::Positions goals;
      goals.push_back(MazeLib::Position(goalX, goalY));

      MazeLib::Position currentPos(startX, startY);

      // Cấp phát Maze và SearchAlgorithm trên HEAP (tránh tràn stack 8KB của FreeRTOS task!)
      MazeLib::Maze* maze = new (std::nothrow) MazeLib::Maze(goals, currentPos);
      if (!maze) {
        LOGE("Out of memory: Failed to allocate Maze!");
        hw->bz->play(hardware::Buzzer::ERROR);
        return;
      }
      maze->reset();

      int idx = 4;
      for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
          char c = mapHex.charAt(idx++);
          uint8_t val = 0;
          if (c >= '0' && c <= '9') val = c - '0';
          else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
          else if (c >= 'a' && c <= 'f') val = c - 'a' + 10;

          bool hasE = (val & 1);
          bool hasN = (val & 2);
          bool hasW = (val & 4);
          bool hasS = (val & 8);

          maze->updateWall(MazeLib::Position(x, y), MazeLib::Direction::East, hasE, false);
          maze->updateWall(MazeLib::Position(x, y), MazeLib::Direction::North, hasN, false);
          maze->updateWall(MazeLib::Position(x, y), MazeLib::Direction::West, hasW, false);
          maze->updateWall(MazeLib::Position(x, y), MazeLib::Direction::South, hasS, false);

          maze->setKnown(MazeLib::Position(x, y), MazeLib::Direction::East, true);
          maze->setKnown(MazeLib::Position(x, y), MazeLib::Direction::North, true);
          maze->setKnown(MazeLib::Position(x, y), MazeLib::Direction::West, true);
          maze->setKnown(MazeLib::Position(x, y), MazeLib::Direction::South, true);
        }
      }

      maze->updateWall(currentPos, MazeLib::Direction::East, true, true);
      maze->updateWall(currentPos, MazeLib::Direction::West, true, true);
      maze->updateWall(currentPos, MazeLib::Direction::South, true, true);

      MazeLib::SearchAlgorithm* searcher = new (std::nothrow) MazeLib::SearchAlgorithm(*maze);
      if (!searcher) {
        LOGE("Out of memory: Failed to allocate SearchAlgorithm!");
        delete maze;
        hw->bz->play(hardware::Buzzer::ERROR);
        return;
      }

      MazeLib::Directions shortestPath;
      bool success = searcher->calcShortestDirections(shortestPath, true);

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
        last_web_map_path = keriseStr;
        LOGI("Shortest Path Generated: %s (Steps: %d)", last_web_map_path.c_str(), (int)shortestPath.size());
        hw->bz->play(hardware::Buzzer::SUCCESSFUL);

        // Giải phóng bộ nhớ heap sau khi tính toán xong
        delete searcher;
        delete maze;

        // Cho phép chạy luôn fast run sau khi giải mê cung thành công
        fast_run();
      } else {
        LOGW("Search failed: No valid path to goal!");
        hw->bz->play(hardware::Buzzer::ERROR);
        delete searcher;
        delete maze;
      }
    } else if (cmdStr.startsWith("X") || cmdStr.startsWith("x")) {
      last_web_map_path = cmdStr.substring(1).c_str();
      fast_run();
    } else if (cmdStr.equalsIgnoreCase("M")) {
      motor_direction_test();
    } else if (cmdStr.equalsIgnoreCase("S")) {
      search_run();
    } else if (cmdStr.startsWith("SYSID") || cmdStr.startsWith("sysid")) {
      int dir = 0;
      float duty = 0.2f;
      int duration = 1000;

      int firstSpace = cmdStr.indexOf(' ');
      if (firstSpace > 0) {
        String remain = cmdStr.substring(firstSpace + 1);
        remain.trim();
        int secondSpace = remain.indexOf(' ');
        if (secondSpace > 0) {
          dir = remain.substring(0, secondSpace).toInt();
          String remain2 = remain.substring(secondSpace + 1);
          remain2.trim();
          int thirdSpace = remain2.indexOf(' ');
          if (thirdSpace > 0) {
            duty = remain2.substring(0, thirdSpace).toFloat();
            duration = remain2.substring(thirdSpace + 1).toInt();
          } else {
            duty = remain2.toFloat();
          }
        } else {
          dir = remain.toInt();
        }
      }
      run_sysid(dir, duty, duration);
    }
  }
};

}  // namespace machine
