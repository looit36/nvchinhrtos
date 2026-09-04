#include "agents/commander.h"
#include "agents/move_action.h"
#include "supporters/speed_controller.h"
#include "config.h"
#include "MazeLib/Maze.h"
#include "MazeLib/SearchAlgorithm.h"
#include "MotorEncoder.h"

extern MotorEncoder encLeft;
extern MotorEncoder encRight;

Commander commander;

void Commander::init() {
    xTaskCreate(serial_task_trampoline, "Serial", 512, this, 2, NULL);
    xTaskCreate(teleplot_task_trampoline, "Teleplot", 512, this, 1, NULL);
}

void Commander::serial_task_trampoline(void *pvParameters) {
    static_cast<Commander*>(pvParameters)->serial_task();
}

void Commander::teleplot_task_trampoline(void *pvParameters) {
    static_cast<Commander*>(pvParameters)->teleplot_task();
}

void Commander::serial_task() {
    String inputBuffer = "";
    for (;;) {
        while (BTSerial.available()) {
            char c = BTSerial.read();
            if (c == '\n' || c == '\r') {
                if (inputBuffer.length() > 0) {
                    if (inputBuffer.charAt(0) == 'X') {
                        // Gọi chế độ FastRun, bỏ chữ X đầu
                        String path = inputBuffer.substring(1);
                        moveAction.start_fast_run(path.c_str());
                    } else if (inputBuffer.startsWith("MAP:")) {
                        // Nhận bản đồ HEX (4 ký tự tọa độ + 256 ký tự tường cho 16x16)
                        String mapHex = inputBuffer.substring(4);
                        if (mapHex.length() >= 260) {
                            int startX = (mapHex.charAt(0) >= 'A') ? (mapHex.charAt(0) - 'A' + 10) : (mapHex.charAt(0) - '0');
                            int startY = (mapHex.charAt(1) >= 'A') ? (mapHex.charAt(1) - 'A' + 10) : (mapHex.charAt(1) - '0');
                            int goalX  = (mapHex.charAt(2) >= 'A') ? (mapHex.charAt(2) - 'A' + 10) : (mapHex.charAt(2) - '0');
                            int goalY  = (mapHex.charAt(3) >= 'A') ? (mapHex.charAt(3) - 'A' + 10) : (mapHex.charAt(3) - '0');
                            
                            MazeLib::Positions goals;
                            goals.push_back(MazeLib::Position(goalX, goalY));
                            
                            MazeLib::Position currentPos(startX, startY);
                            MazeLib::Maze* maze = new (std::nothrow) MazeLib::Maze(goals, currentPos);
                            
                            if (!maze) {
                                BTSerial.println(">Err:OOM Maze|xy");
                            } else {
                                maze->reset();
                                
                                int idx = 4;
                                for (int x = 0; x < 16; x++) {
                                    for (int y = 0; y < 16; y++) {
                                        char c = mapHex[idx++];
                                        uint8_t val = 0;
                                        if (c >= '0' && c <= '9') val = c - '0';
                                        else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
                                        
                                        // Web: N=2(Top), S=8(Bottom), E=1(Right), W=4(Left)
                                        // Trục Y: y=0 là Bottom-Left, KHỚP HOÀN TOÀN với Kerise
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
                                
                                // FORCE chuẩn Micromouse: Ô xuất phát chỉ được phép đi thẳng (North)
                                maze->updateWall(currentPos, MazeLib::Direction::East, true, true);
                                maze->updateWall(currentPos, MazeLib::Direction::West, true, true);
                                maze->updateWall(currentPos, MazeLib::Direction::South, true, true);
                                
                                // Gọi thuật toán tìm đường ngắn nhất
                                MazeLib::SearchAlgorithm* searcher = new (std::nothrow) MazeLib::SearchAlgorithm(*maze);
                                if (!searcher) {
                                    BTSerial.println(">Err:OOM Search|xy");
                                } else {
                                    MazeLib::Directions shortestPath;
                                    bool success = searcher->calcShortestDirections(shortestPath, true);
                            
                                    if (success) {
                                        String keriseStr = "";
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
                                        // Gói chuỗi Search lại bằng 2 chữ 's'
                                        keriseStr = "s" + keriseStr + "s";
                                        
                                        BTSerial.print("Kerise Path: ");
                                        BTSerial.println(keriseStr);
                                        
                                        // Gọi FastRun
                                        moveAction.start_fast_run(keriseStr.c_str());
                                    } else {
                                        BTSerial.println("Search failed! No path to goal.");
                                    }
                                    delete searcher;
                                }
                                delete maze;
                            }
                        } else {
                            BTSerial.print("Error: MAP string too short! Expected >= 260, got: ");
                            BTSerial.println(mapHex.length());
                        }
                    } else {
                        // Chạy từng lệnh bình thường theo định dạng Kerise
                        for (size_t i = 0; i < inputBuffer.length(); i++) {
                            char cmd = inputBuffer.charAt(i);
                            if (cmd == 'M' || cmd == 'm') {
                                run_motor_direction_test();
                            } else if (cmd == 'S' || cmd == 'F' || cmd == 'R' || cmd == 'L' || cmd == 'B' || cmd == 'T' || cmd == 't') {
                                if (cmd == 'T') BTSerial.println(">>> EXEC SPIN TEST (180 DEG) <<<");
                                if (cmd == 't') BTSerial.println(">>> EXEC SPIN TEST (90 DEG) <<<");
                                moveAction.push_command(cmd);
                            }
                        }
                    }
                    inputBuffer = "";
                }
            } else {
                inputBuffer += c;
            }
        }
        vTaskDelay(1);
    }
}

void Commander::run_motor_direction_test() {
    BTSerial.println("\r\n============================================");
    BTSerial.println(">>> START OPEN-LOOP MOTOR DIRECTION TEST <<<");
    BTSerial.println(">> CHU Y: HAY NANG BANH XE KHOI MAT BAN <<");
    BTSerial.println("============================================");

    speedCtrl.set_manual_override(true);
    speedCtrl.set_manual_pwm(0, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    const int test_pwm = 400;     // ~40% duty (thang 1000 theo chuẩn Zirconia), đủ lực bứt ma sát tĩnh mà không giật xe
    const int duration_ms = 500;  // 500ms mỗi chiều

    // --- TEST 1: MOTOR LEFT FORWARD ---
    BTSerial.println("\n[1/4] Testing LEFT Motor FORWARD (Duty +400)...");
    long encL_before = encLeft.getCount();
    speedCtrl.set_manual_pwm(test_pwm, 0);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    speedCtrl.set_manual_pwm(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    long encL_fwd_delta = encLeft.getCount() - encL_before;
    BTSerial.print("  -> EncLeft Delta: ");
    BTSerial.println(encL_fwd_delta);

    // --- TEST 2: MOTOR LEFT BACKWARD ---
    BTSerial.println("[2/4] Testing LEFT Motor BACKWARD (Duty -400)...");
    encL_before = encLeft.getCount();
    speedCtrl.set_manual_pwm(-test_pwm, 0);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    speedCtrl.set_manual_pwm(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    long encL_bwd_delta = encLeft.getCount() - encL_before;
    BTSerial.print("  -> EncLeft Delta: ");
    BTSerial.println(encL_bwd_delta);

    // --- TEST 3: MOTOR RIGHT FORWARD ---
    BTSerial.println("\n[3/4] Testing RIGHT Motor FORWARD (Duty +400)...");
    long encR_before = encRight.getCount();
    speedCtrl.set_manual_pwm(0, test_pwm);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    speedCtrl.set_manual_pwm(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    long encR_fwd_delta = encRight.getCount() - encR_before;
    BTSerial.print("  -> EncRight Delta: ");
    BTSerial.println(encR_fwd_delta);

    // --- TEST 4: MOTOR RIGHT BACKWARD ---
    BTSerial.println("[4/4] Testing RIGHT Motor BACKWARD (PWM -100)...");
    encR_before = encRight.getCount();
    speedCtrl.set_manual_pwm(0, -test_pwm);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    speedCtrl.set_manual_pwm(0, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    long encR_bwd_delta = encRight.getCount() - encR_before;
    BTSerial.print("  -> EncRight Delta: ");
    BTSerial.println(encR_bwd_delta);

    // Tắt chế độ manual override & reset PID
    speedCtrl.set_manual_override(false);
    speedCtrl.reset();

    BTSerial.println("\n============================================");
    BTSerial.println(">>> KET QUA CHAN DOAN CHIEU MOTOR <<<");
    BTSerial.println("============================================");

    bool left_ok = (encL_fwd_delta > 10) && (encL_bwd_delta < -10);
    bool right_ok = (encR_fwd_delta > 10) && (encR_bwd_delta < -10);

    if (abs(encL_fwd_delta) < 10 && abs(encL_bwd_delta) < 10) {
        BTSerial.println("(!) CANH BAO: Motor Trai khong quay hoac Encoder Trai khong doc duoc!");
    } else if (left_ok) {
        BTSerial.println("[PASS] Motor Trai: CHIEU DUNG (Forward -> Enc tang, Backward -> Enc giam)");
    } else {
        BTSerial.println("[FAIL] Motor Trai: BI NGUOC! -> Can doi MOTOR_L_DIR = -1 trong config.h");
    }

    if (abs(encR_fwd_delta) < 10 && abs(encR_bwd_delta) < 10) {
        BTSerial.println("(!) CANH BAO: Motor Phai khong quay hoac Encoder Phai khong doc duoc!");
    } else if (right_ok) {
        BTSerial.println("[PASS] Motor Phai: CHIEU DUNG (Forward -> Enc tang, Backward -> Enc giam)");
    } else {
        BTSerial.println("[FAIL] Motor Phai: BI NGUOC! -> Can doi MOTOR_R_DIR = -1 trong config.h");
    }
    BTSerial.println("============================================\n");
}

void Commander::teleplot_task() {
    long last_encL = encLeft.getCount();
    long last_encR = encRight.getCount();
    TickType_t last_time = xTaskGetTickCount();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz Teleplot streaming (tối ưu cho 115200 baud)

        long cur_encL = encLeft.getCount();
        long cur_encR = encRight.getCount();
        TickType_t cur_time = xTaskGetTickCount();

        float dt = (cur_time - last_time) * (float)portTICK_PERIOD_MS / 1000.0f;
        if (dt <= 0.0001f) dt = 0.02f;

        // 1. Tính toán vận tốc (mm/s & RPM) và quãng đường (mm) từng bánh
        float speed_L = ((cur_encL - last_encL) * MM_PER_COUNT) / dt;
        float speed_R = ((cur_encR - last_encR) * MM_PER_COUNT) / dt;
        float rpm_L   = ((float)(cur_encL - last_encL) / (COUNTS_PER_REV * GEAR_RATIO)) / (dt / 60.0f);
        float rpm_R   = ((float)(cur_encR - last_encR) / (COUNTS_PER_REV * GEAR_RATIO)) / (dt / 60.0f);
        float dist_L  = (float)cur_encL * MM_PER_COUNT;
        float dist_R  = (float)cur_encR * MM_PER_COUNT;

        last_encL = cur_encL;
        last_encR = cur_encR;
        last_time = cur_time;

        // === Teleplot: 2-Wheel Encoder Metrics (50Hz) ===
        BTSerial.print(">EncL_Raw:");  BTSerial.println(cur_encL);
        BTSerial.print(">EncR_Raw:");  BTSerial.println(cur_encR);
        BTSerial.print(">SpdL_mms:");  BTSerial.println(speed_L, 1);
        BTSerial.print(">SpdR_mms:");  BTSerial.println(speed_R, 1);
        BTSerial.print(">RpmL:");      BTSerial.println(rpm_L, 1);
        BTSerial.print(">RpmR:");      BTSerial.println(rpm_R, 1);
        BTSerial.print(">DistL_mm:");  BTSerial.println(dist_L, 1);
        BTSerial.print(">DistR_mm:");  BTSerial.println(dist_R, 1);

        // === Teleplot: Rotational PID Metrics ===
        float ref_w_deg = speedCtrl.ref_w * 180.0f / PI;
        float est_w_deg = speedCtrl.est_w * 180.0f / PI;
        float err_w_deg = (speedCtrl.ref_w - speedCtrl.est_w) * 180.0f / PI;
        float th_deg    = speedCtrl.est_p.th * 180.0f / PI;

        BTSerial.print(">RefW:"); BTSerial.println(ref_w_deg, 1);
        BTSerial.print(">EstW:"); BTSerial.println(est_w_deg, 1);
        BTSerial.print(">ErrW:"); BTSerial.println(err_w_deg, 1);
        BTSerial.print(">Heading:"); BTSerial.println(th_deg, 1);
        
        BTSerial.print(">FfRot:"); BTSerial.println(speedCtrl.telemetry_ff_rot, 2);
        BTSerial.print(">FbRot:"); BTSerial.println(speedCtrl.telemetry_fb_rot, 2);
        
        BTSerial.print(">PwmL:"); BTSerial.println(speedCtrl.telemetry_pwm_L);
        BTSerial.print(">PwmR:"); BTSerial.println(speedCtrl.telemetry_pwm_R);
        BTSerial.print(">VBat:"); BTSerial.println(speedCtrl.battery_voltage, 2);

        // === Teleplot: 2D Position ===
        BTSerial.print(">EstXY:"); 
        BTSerial.print(speedCtrl.est_p.x); 
        BTSerial.print(":"); 
        BTSerial.print(speedCtrl.est_p.y);
        BTSerial.println("|xy");
    }
}

