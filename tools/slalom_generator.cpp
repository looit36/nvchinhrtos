/**
 * @file slalom_generator.cpp
 * @brief Generator for Slalom Shapes (Kerise v4 compatible)
 * @author Antigravity pair programming with User
 */

#include <ctrl/slalom/trajectory.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ctrl;

static const float pi = M_PI;
static const float sqrt_2 = std::sqrt(2.0f);

struct ShapeConfig {
  std::string name;
  Pose total;
  float y_curve_end;
  float x_adv;
};

void run_generator(float W, bool scale_dynamics_by_friction, const std::string& title) {
  std::cout << "\n========================================================================\n";
  std::cout << " " << title << " (W = " << W << " mm)\n";
  std::cout << "========================================================================\n";

  // Hệ số tỷ lệ kích thước so với ô 90mm chuẩn của Kerise
  const float K = W / 90.0f;

  float dddth_max = slalom::dddth_max_default; // 1200 * pi
  float ddth_max = slalom::ddth_max_default;   // 36 * pi
  float dth_max = slalom::dth_max_default;     // 3 * pi

  if (scale_dynamics_by_friction) {
    // Giữ gia tốc hướng tâm a_lat = v^2 / R = const -> v ~ sqrt(K)
    const float v_scale = std::sqrt(K);
    dth_max /= v_scale;
    ddth_max /= K;
    dddth_max /= (K * v_scale);
  }

  std::vector<ShapeConfig> configs = {
      {"SS_S90 ", Pose(45 * K, 45 * K, pi / 2), 44 * K, 0},
      {"SS_F45 ", Pose(90 * K, 45 * K, pi / 4), 30 * K, 0},
      {"SS_F90 ", Pose(90 * K, 90 * K, pi / 2), 70 * K, 0},
      {"SS_F135", Pose(45 * K, 90 * K, pi * 3 / 4), 80 * K, 0},
      {"SS_F180", Pose(0, 90 * K, pi), 90 * K, 24 * K},
      {"SS_FV90", Pose(45 * sqrt_2 * K, 45 * sqrt_2 * K, pi / 2), 48 * K, 0},
      {"SS_FK90", Pose(90 * sqrt_2 * K, 90 * sqrt_2 * K, pi / 2), 125 * K, 0},
      {"SS_FS90", Pose(45 * K, 45 * K, pi / 2), 44 * K, 0},
  };

  std::cout << std::fixed << std::setprecision(5);
  std::cout << "/*\n";
  std::cout << "  Angular Limits: dddth_max = " << dddth_max << " rad/s^3, "
            << "ddth_max = " << ddth_max << " rad/s^2, "
            << "dth_max = " << dth_max << " rad/s\n";
  std::cout << "*/\n";

  std::cout << "static const std::array<ctrl::slalom::Shape, ShapeIndexMax> shapes = {{\n";
  for (const auto& cfg : configs) {
    slalom::Shape s(cfg.total, cfg.y_curve_end, cfg.x_adv, dddth_max, ddth_max, dth_max);
    const AccelDesigner ad(s.dddth_max, s.ddth_max, s.dth_max, 0, 0, s.total.th);
    const float t_total = ad.t_end() + (s.straight_prev + s.straight_post) / s.v_ref;

    std::cout << "/* " << cfg.name << " T:" << std::setprecision(6) << t_total << " */\n";
    std::cout << "ctrl::slalom::Shape(\n"
              << "    ctrl::Pose(" << std::setw(10) << s.total.x << "f, "
                                 << std::setw(10) << s.total.y << "f, "
                                 << std::setw(10) << s.total.th << "f),\n"
              << "    ctrl::Pose(" << std::setw(10) << s.curve.x << "f, "
                                 << std::setw(10) << s.curve.y << "f, "
                                 << std::setw(10) << s.curve.th << "f),\n"
              << "    " << std::setw(10) << s.straight_prev << "f, "
              << std::setw(10) << s.straight_post << "f, "
              << std::setw(10) << s.v_ref << "f,\n"
              << "    " << std::setw(10) << s.dddth_max << "f, "
              << std::setw(10) << s.ddth_max << "f, "
              << std::setw(10) << s.dth_max << "f),\n";
  }
  std::cout << "}};\n";
}

int main(int argc, char** argv) {
  float W = 180.0f; // Mặc định full-size 180 mm (18 cm)
  if (argc > 1) {
    W = std::stof(argv[1]);
  }

  // Chạy cả 2 chế độ để quan sát và so sánh:
  run_generator(W, true, "CHẾ ĐỘ 1: BẢO TOÀN GIA TỐC HƯỚNG TÂM AN TOÀN (v_ref ~ sqrt(K), Khuyên dùng)");
  run_generator(W, false, "CHẾ ĐỘ 2: GIỮ NGUYÊN GIỚI HẠN GÓC GỐC (v_ref ~ K, Tốc độ cao)");

  return 0;
}
