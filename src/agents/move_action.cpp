#include "agents/move_action.h"
#include "config.h"
#include "ctrl/accel_designer.h"
#include "supporters/speed_controller.h"


MoveAction moveAction;

void MoveAction::init() {
  commandQueue = xQueueCreate(50, sizeof(char));
  xTaskCreate(task_trampoline, "Action", 2048, this, 3, NULL);
}

void MoveAction::push_command(char cmd) { xQueueSend(commandQueue, &cmd, 0); }

void MoveAction::task_trampoline(void *pvParameters) {
  static_cast<MoveAction *>(pvParameters)->task();
}

void MoveAction::task() {
  for (;;) {
    char cmd;
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdPASS) {
      float Ts = 0.001f;

      if (cmd == 'S' || cmd == 'F') { // ST_FULL: Đi thẳng 1 ô (300mm)
        float dist = FULL_CELL;
        ctrl::AccelDesigner ad(METRIC_JERK * 1000.0f, METRIC_ACCEL * 1000.0f,
                               METRIC_SEARCH_SPEED * 1000.0f, 0, 0, dist);
        for (float t = 0; t < ad.t_end(); t += Ts) {
          speedCtrl.set_target(ad.v(t), 0, ad.a(t), 0);
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        speedCtrl.set_target(0, 0, 0, 0);

        ctrl::Pose new_p = speedCtrl.est_p;
        new_p.x -= dist;
        speedCtrl.update_pose(new_p);
      } else if (cmd == 'R' || cmd == 'L' || cmd == 'B') {
        // TURN: Tiến nửa ô vào tâm → Xoay → Tiến nửa ô ra mép

        // 1. Tiến nửa ô (150mm)
        ctrl::AccelDesigner ad_in(METRIC_JERK * 1000.0f, METRIC_ACCEL * 1000.0f,
                                  METRIC_SEARCH_SPEED * 1000.0f, 0, 0,
                                  HALF_CELL);
        for (float t = 0; t < ad_in.t_end(); t += Ts) {
          speedCtrl.set_target(ad_in.v(t), 0, ad_in.a(t), 0);
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        speedCtrl.set_target(0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100)); // Dừng nhẹ

        ctrl::Pose new_p_in = speedCtrl.est_p;
        new_p_in.x -= HALF_CELL;
        speedCtrl.update_pose(new_p_in);

        // 2. Xoay tại chỗ — dùng rad-based dynamics + back_gain (Kerise-style)
        float angle = 0;
        if (cmd == 'R')
          angle = -PI / 2.0f; // rad
        if (cmd == 'L')
          angle = PI / 2.0f; // rad
        if (cmd == 'B')
          angle = PI; // rad

        const float back_gain = 10.0f; // Kerise: model::turn_back_gain = 10.0

        ctrl::AccelDesigner ad_rot(SPIN_TURN_JERK, SPIN_TURN_ALPHA,
                                   SPIN_TURN_OMEGA, 0, 0, angle);
        for (float t = 0; t < ad_rot.t_end(); t += Ts) {
          // back_gain: kéo robot về gốc khi xoay (tránh trôi)
          float delta = speedCtrl.est_p.x * cos(-speedCtrl.est_p.th) -
                        speedCtrl.est_p.y * sin(-speedCtrl.est_p.th);
          speedCtrl.set_target(-delta * back_gain, ad_rot.v(t), 0, ad_rot.a(t));
          vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Settling PI loop — bám chính xác góc mục tiêu (Kerise-style)
        float int_error = 0;
        for (int i = 0; i < 2000; i++) // Max 2s timeout
        {
          float delta = speedCtrl.est_p.x * cos(-speedCtrl.est_p.th) -
                        speedCtrl.est_p.y * sin(-speedCtrl.est_p.th);
          const float Kp_settle = 10.0f;
          const float Ki_settle = 10.0f;
          float error = angle - speedCtrl.est_p.th;
          int_error += error * Ts;
          speedCtrl.set_target(-delta * back_gain,
                               Kp_settle * error + Ki_settle * int_error);
          vTaskDelay(pdMS_TO_TICKS(1));
          if (fabsf(Kp_settle * error) + fabsf(Ki_settle * int_error) <
              0.1f * PI)
            break;
        }
        speedCtrl.set_target(0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100)); // Dừng nhẹ

        ctrl::Pose new_p_rot = speedCtrl.est_p;
        new_p_rot.th -= angle;
        speedCtrl.update_pose(new_p_rot);

        // 3. Tiến nửa ô (150mm)
        ctrl::AccelDesigner ad_out(
            METRIC_JERK * 1000.0f, METRIC_ACCEL * 1000.0f,
            METRIC_SEARCH_SPEED * 1000.0f, 0, 0, HALF_CELL);
        for (float t = 0; t < ad_out.t_end(); t += Ts) {
          speedCtrl.set_target(ad_out.v(t), 0, ad_out.a(t), 0);
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        speedCtrl.set_target(0, 0, 0, 0);

        ctrl::Pose new_p_out = speedCtrl.est_p;
        new_p_out.x -= HALF_CELL;
        speedCtrl.update_pose(new_p_out);
      }
      vTaskDelay(pdMS_TO_TICKS(100)); // Nghỉ 1 chút sau mỗi lệnh
    }
  }
}

#include <ctrl/slalom/trajectory.h>
#include <ctrl/straight/trajectory.h>
#include <ctrl/trajectory_tracker.h>


void MoveAction::start_fast_run(const char *search_path) {
  std::string fast_path = convert_search_to_fast(search_path);
  execute_fast_run(fast_path);
}

// Hàm replace chuỗi
static int replace(std::string &src, const std::string &from,
                   const std::string &to) {
  if (from.empty())
    return 0;
  auto pos = src.find(from);
  auto toLen = to.size();
  int i = 0;
  while ((pos = src.find(from, pos)) != std::string::npos) {
    src.replace(pos, from.size(), to);
    pos += toLen;
    i++;
  }
  return i;
}

// Hàm nén chuỗi tìm kiếm thành chuỗi FastRun Diagonal (Giống y hệt MazeLib)
std::string MoveAction::convert_search_to_fast(std::string src) {
  // 0. Chuyển đổi định dạng Chuỗi của User sang định dạng chuẩn của Kerise
  // User format: "F R F L F" (R chỉ quay tại chỗ, F là tiến 1 ô)
  // Kerise format: "S R L" (R là tiến nửa ô, quay 90, tiến nửa ô = tổng 1 ô)
  replace(src, "RF", "R");
  replace(src, "LF", "L");
  replace(src, "BF", "B");
  replace(src, "F", "S");

  // 1. Mở rộng (Expand)
  replace(src, "S", "ss");
  replace(src, "L", "ll");
  replace(src, "R", "rr");

  // 2. Nén chéo (Diagonal)
  replace(src, "rllllr", "rlplr"); // FV90_L (p)
  replace(src, "lrrrrl", "lrPrl"); // FV90_R (P)
  replace(src, "sllr", "zlr");     // F45_L (z)
  replace(src, "srrl", "crl");     // F45_R (c)
  replace(src, "rlls", "rlZ");     // F45_LP (Z)
  replace(src, "lrrs", "lrC");     // F45_RP (C)
  replace(src, "sllllr", "alr");   // F135_L (a)
  replace(src, "srrrrl", "drl");   // F135_R (d)
  replace(src, "rlllls", "rlA");   // F135_LP (A)
  replace(src, "lrrrrs", "lrD");   // F135_RP (D)
  replace(src, "slllls", "u");     // F180_L (u)
  replace(src, "srrrrs", "U");     // F180_R (U)
  replace(src, "rllr", "rlwlr");   // ST_DIAG (w)
  replace(src, "lrrl", "lrwrl");   // ST_DIAG (w)
  replace(src, "slls", "q");       // F90_L (q)
  replace(src, "srrs", "Q");       // F90_R (Q)

  // 3. Dọn dẹp padding
  replace(src, "rl", "");
  replace(src, "lr", "");

  // 4. Gom các đoạn thẳng
  replace(src, "ss", "S"); // ST_FULL (S)
  replace(src, "ll", "L"); // FS90_L (L)
  replace(src, "rr", "R"); // FS90_R (R)
  replace(src, "s", "h");  // ST_HALF (h)

  return src;
}

void MoveAction::execute_fast_run(std::string fast_path) {
  float Ts = 0.001f;
  float v_run = METRIC_FAST_SPEED * 1000.0f;  // mm/s
  float a_run = METRIC_FAST_ACCEL * 1000.0f;  // mm/s^2
  float j_run = METRIC_JERK * 1000.0f;        // mm/s^3
  float v_turn = METRIC_TURN_SPEED * 1000.0f; // mm/s

  // Hệ số Scale cho lưới 300mm (Chuẩn Kerise halfsize là 90mm)
  float K = FULL_CELL / 90.0f;
  float cell = FULL_CELL;
  float half = HALF_CELL;
  float sqrt_2 = sqrt(2.0f);

  // Giới hạn động học góc PHẢI được scale xuống để giữ nguyên vận tốc tuyến
  // tính! Nếu không scale, robot sẽ chạy cua nhanh gấp 3.33 lần Kerise (lên tới
  // >2m/s) gây văng xe!
  float w_max = ctrl::slalom::dth_max_default / K;
  float aw_max = ctrl::slalom::ddth_max_default / (K * K);
  float jw_max = ctrl::slalom::dddth_max_default / (K * K * K);

  // Slalom shapes — Scale hình học từ Kerise 90mm lên 300mm
  // Constructor #1: Shape(total, y_curve_end, x_adv, dddth_max, ddth_max,
  // dth_max) y_curve_end = curve Y displacement (auto-generates curve,
  // straight_prev/post, v_ref)
  ctrl::slalom::Shape shapeF45(ctrl::Pose(90.0f * K, 45.0f * K, PI / 4.0f),
                               30.0f * K, 0, jw_max, aw_max, w_max);
  ctrl::slalom::Shape shapeF90(ctrl::Pose(90.0f * K, 90.0f * K, PI / 2.0f),
                               70.0f * K, 0, jw_max, aw_max, w_max);
  ctrl::slalom::Shape shapeF135(
      ctrl::Pose(45.0f * K, 90.0f * K, 3.0f * PI / 4.0f), 80.0f * K, 0, jw_max,
      aw_max, w_max);
  ctrl::slalom::Shape shapeF180(ctrl::Pose(0.0f, 90.0f * K, PI), 90.0f * K,
                                24.0f * K, jw_max, aw_max, w_max);
  ctrl::slalom::Shape shapeFV90(
      ctrl::Pose(45.0f * sqrt_2 * K, 45.0f * sqrt_2 * K, PI / 2.0f), 48.0f * K,
      0, jw_max, aw_max, w_max);
  ctrl::slalom::Shape shapeFS90(ctrl::Pose(45.0f * K, 45.0f * K, PI / 2.0f),
                                44.0f * K, 0.0f, jw_max, aw_max, w_max);

  const float TAIL_OFFSET =
      48.0f; // khoảng cách từ đuôi (áp tường) tới tâm robot, mm
  float straight_acc = HALF_CELL - TAIL_OFFSET;
  float current_v = 0.0f;
  ctrl::TrajectoryTracker::Gain tt_gain;
  // omega_n điều chỉnh độ mạnh của feedback tracking.
  // omega_n thấp (1.5): tracking chậm, giảm tốc mạnh khi lệch → an toàn nhưng
  // chậm omega_n cao (5-10): tracking nhanh, cho phép vận tốc cao hơn → nhanh
  // nhưng có thể trượt Với cell 300mm và v_run=5m/s, cần omega_n cao hơn để
  // không bị giới hạn vận tốc
  tt_gain.omega_n = 1.5f;

  // Hàm nội suy đi thẳng dùng TrajectoryTracker
  // Tự động xác định v_max dựa trên quãng đường và v_end:
  // - Nếu đoạn đủ dài để tăng tốc, dùng v_run
  // - Nếu đoạn ngắn hoặc phải giảm tốc về v_end thấp, giới hạn v_max
  auto run_straight = [&](float dist, float v_end) {
    if (dist <= 0)
      return;

    // Tính v_max tối ưu: cho phép tăng tốc tới v_run nếu đoạn đủ dài
    float v_max_allowed = v_run;

    ctrl::straight::Trajectory traj;
    traj.reset(j_run, a_run, v_max_allowed, current_v, v_end, dist);

    ctrl::TrajectoryTracker tt(tt_gain);
    tt.reset(current_v);

    ctrl::State s;
    for (float t = 0; t < traj.t_end(); t += Ts) {
      traj.update(s, t);
      const auto ref = tt.update(
          speedCtrl.est_p, ctrl::Polar(speedCtrl.est_v, speedCtrl.est_w),
          ctrl::Polar(speedCtrl.est_dv, speedCtrl.est_dw), s);
      speedCtrl.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    current_v = v_end;

    // Cập nhật lại hệ tọa độ cục bộ (đẩy trục X lên 1 đoạn = dist)
    const auto net = ctrl::Pose(dist, 0, 0);
    speedCtrl.update_pose((speedCtrl.est_p - net).rotate(-net.th));
  };

  // Hàm nội suy Slalom dùng TrajectoryTracker (Kerise SlalomProcess style)
  // reverse=false: straight_prev trước, straight_post sau (normal)
  // reverse=true:  straight_post trước, straight_prev sau (P-variant)
  // NOTE: Không gọi run_straight ở đây nữa - đã được gọi trước trong switch với
  // v_end thích hợp
  auto run_slalom = [&](ctrl::slalom::Shape &shape, bool is_right,
                        bool reverse = false) {
    ctrl::slalom::Trajectory traj(shape, is_right);
    const float v_slalom = v_turn;
    traj.reset(v_slalom);

    // Chạy Curve (straight_prev đã được xử lý bên ngoài)
    ctrl::TrajectoryTracker tt(tt_gain);
    tt.reset(v_slalom);

    ctrl::State s;
    for (float t = 0; t < traj.getTimeCurve(); t += Ts) {
      traj.update(s, t, Ts);
      const auto ref = tt.update(
          speedCtrl.est_p, ctrl::Polar(speedCtrl.est_v, speedCtrl.est_w),
          ctrl::Polar(speedCtrl.est_dv, speedCtrl.est_dw), s);
      speedCtrl.set_target(ref.v, ref.w, ref.dv, ref.dw);
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    current_v = v_slalom;

    // Cập nhật lại hệ tọa độ cục bộ (reset vị trí, xoay heading theo góc rẽ)
    const auto &net = traj.getShape().curve;
    speedCtrl.update_pose((speedCtrl.est_p - net).rotate(-net.th));

    // Kerise: straight += reverse ? straight_prev : straight_post
    straight_acc +=
        reverse ? traj.getShape().straight_prev : traj.getShape().straight_post;
  };

  // Helper: kiểm tra ký tự có là straight không (S, h, w)
  auto is_straight = [](char c) { return c == 'S' || c == 'h' || c == 'w'; };

  // Helper: chạy straight với straight_prev của slalom, sau đó chạy slalom
  auto run_straight_then_slalom = [&](ctrl::slalom::Shape &shape, bool is_right,
                                      bool reverse = false) {
    // Tạo trajectory để lấy straight_prev
    ctrl::slalom::Trajectory traj(shape, is_right);
    traj.reset(v_turn);

    // Cộng straight_prev (hoặc straight_post nếu reverse)
    straight_acc += !reverse ? traj.getShape().straight_prev
                             : traj.getShape().straight_post;

    // Chạy đoạn thẳng và giảm tốc xuống v_turn
    run_straight(straight_acc, v_turn);
    straight_acc = 0.0f;

    // Chạy slalom curve
    run_slalom(shape, is_right, reverse);
  };

  for (size_t i = 0; i < fast_path.size(); i++) {
    char c = fast_path[i];
    // Look-ahead: nếu move tiếp theo là straight hoặc hết path, v_end = v_run
    // Nếu move tiếp theo là turn, v_end = v_slalom
    bool next_is_straight =
        (i + 1 < fast_path.size()) && is_straight(fast_path[i + 1]);
    bool is_last_move = (i + 1 >= fast_path.size());
    float v_end_after_straight =
        (next_is_straight || is_last_move) ? v_run : v_turn;

    switch (c) {
    case 'S':
      straight_acc += cell;
      break;
    case 'h':
      straight_acc += half;
      break;
    case 'w':
      straight_acc += half * sqrt_2;
      break; // ST_DIAG
    case 'z':
      run_straight_then_slalom(shapeF45, false, false);
      break; // F45_L
    case 'c':
      run_straight_then_slalom(shapeF45, true, false);
      break; // F45_R
    case 'Z':
      run_straight_then_slalom(shapeF45, false, true);
      break; // F45_LP (same shape, reverse=true)
    case 'C':
      run_straight_then_slalom(shapeF45, true, true);
      break; // F45_RP (same shape, reverse=true)
    case 'q':
      run_straight_then_slalom(shapeF90, false);
      break; // F90_L
    case 'Q':
      run_straight_then_slalom(shapeF90, true);
      break; // F90_R
    case 'p':
      run_straight_then_slalom(shapeFV90, false);
      break; // FV90_L
    case 'P':
      run_straight_then_slalom(shapeFV90, true);
      break; // FV90_R
    case 'L':
      run_straight_then_slalom(shapeFS90, false);
      break; // FS90_L
    case 'R':
      run_straight_then_slalom(shapeFS90, true);
      break; // FS90_R
    case 'a':
      run_straight_then_slalom(shapeF135, false, false);
      break; // F135_L
    case 'd':
      run_straight_then_slalom(shapeF135, true, false);
      break; // F135_R
    case 'A':
      run_straight_then_slalom(shapeF135, false, true);
      break; // F135_LP (same shape, reverse=true)
    case 'D':
      run_straight_then_slalom(shapeF135, true, true);
      break; // F135_RP (same shape, reverse=true)
    case 'u':
      run_straight_then_slalom(shapeF180, false);
      break; // F180_L
    case 'U':
      run_straight_then_slalom(shapeF180, true);
      break; // F180_R
    }
  }

  // Cuối cùng: Chạy nốt quãng đường thẳng còn lại và phanh
  run_straight(straight_acc, 0.0f);
  speedCtrl.set_target(0, 0, 0, 0);
}
