/**
 * @file slalom_shapes.h
 * @brief Slalom Shape Definitions for Full-size Maze (18cm x 18cm)
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <ctrl/slalom/slalom.h>
#include <ctrl/slalom/trajectory.h>
#include <array>
#include <cmath>

namespace field {

enum ShapeIndex {
  S90,
  F45,
  F90,
  F135,
  F180,
  FV90,
  FK90,
  FS90,
  ShapeIndexMax,
};

/**
 * @brief Bảng thông số Slalom chuẩn cho sa bàn Full-size 180mm (18cm x 18cm).
 * Được sinh tự động bởi công cụ tools/slalom_generator.exe với bảo toàn
 * gia tốc hướng tâm an toàn a_lat = v^2 / R (v_ref ~ sqrt(K)).
 */
static const std::array<ctrl::slalom::Shape, ShapeIndexMax> shapes = {{
/* SS_S90  T:0.406622 */
ctrl::slalom::Shape(ctrl::Pose(      90,       90,    1.5708), ctrl::Pose(      88,       88,    1.5708),  1.99973,        2,  375.827, 1332.86, 56.5487, 6.66432),
/* SS_F45  T:0.363638 */
ctrl::slalom::Shape(ctrl::Pose(     180,       90,  0.785398), ctrl::Pose( 144.853,       60,  0.785398),  5.14722,  42.4264,  582.142, 1332.86, 56.5487, 6.66432),
/* SS_F90  T:0.529779 */
ctrl::slalom::Shape(ctrl::Pose(     180,      180,    1.5708), ctrl::Pose(     140,      140,    1.5708),  39.9995,       40,  597.907, 1332.86, 56.5487, 6.66432),
/* SS_F135 T:0.657827 */
ctrl::slalom::Shape(ctrl::Pose(      90,      180,   2.35619), ctrl::Pose( 66.2748,      160,   2.35619),  43.7253,  28.2843,  500.081, 1332.86, 56.5487, 6.66432),
/* SS_F180 T:0.796353 */
ctrl::slalom::Shape(ctrl::Pose(       0,      180,   3.14159), ctrl::Pose(       0,      180,   3.14159),       48,       48,  582.979, 1332.86, 56.5487, 6.66432),
/* SS_FV90 T:0.548563 */
ctrl::slalom::Shape(ctrl::Pose( 127.279,  127.279,    1.5708), ctrl::Pose(      96,       96,    1.5708),  31.2789,  31.2792,  409.993, 1332.86, 56.5487, 6.66432),
/* SS_FK90 T:0.404518 */
ctrl::slalom::Shape(ctrl::Pose( 254.558,  254.558,    1.5708), ctrl::Pose(     250,      250,    1.5708),  4.55756,  4.55846,  1067.69, 1332.86, 56.5487, 6.66432),
/* SS_FS90 T:0.406622 */
ctrl::slalom::Shape(ctrl::Pose(      90,       90,    1.5708), ctrl::Pose(      88,       88,    1.5708),  1.99973,        2,  375.827, 1332.86, 56.5487, 6.66432),
}};

}  // namespace field
