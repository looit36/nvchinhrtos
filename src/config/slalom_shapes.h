/**
 * @file slalom_shapes.h
 * @brief Slalom Shape Definitions dynamically scaled by field::SegWidthFull
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <ctrl/slalom.h>
#include <array>
#include <cmath>
#include "config/model.h"

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

inline ctrl::slalom::Shape get_shape(ShapeIndex idx) {
  const float K = field::SegWidthFull / 90.0f;
  const float v_scale = std::sqrt(K); // Giữ gia tốc hướng tâm an toàn: a_lat = v^2 / R

  switch (idx) {
    case S90:
      return ctrl::slalom::Shape(
          ctrl::Pose(45.0f * K, 45.0f * K, 1.5708f),
          ctrl::Pose(44.0f * K, 44.0f * K, 1.5708f),
          1.00004f * K, 1.0f * K,
          265.749f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case F45:
      return ctrl::slalom::Shape(
          ctrl::Pose(90.0f * K, 45.0f * K, 0.785398f),
          ctrl::Pose(72.4263f * K, 30.0f * K, 0.785398f),
          2.57365f * K, 21.2132f * K,
          411.636f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case F90:
      return ctrl::slalom::Shape(
          ctrl::Pose(90.0f * K, 90.0f * K, 1.5708f),
          ctrl::Pose(70.0f * K, 70.0f * K, 1.5708f),
          20.0f * K, 20.0f * K,
          422.783f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case F135:
      return ctrl::slalom::Shape(
          ctrl::Pose(45.0f * K, 90.0f * K, 2.35619f),
          ctrl::Pose(33.1373f * K, 80.0f * K, 2.35619f),
          21.8627f * K, 14.1421f * K,
          353.609f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case F180:
      return ctrl::slalom::Shape(
          ctrl::Pose(0.0f * K, 90.0f * K, 3.14159f),
          ctrl::Pose(0.0f * K, 90.0f * K, 3.14159f),
          24.0f * K, 24.0f * K,
          412.228f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case FV90:
      return ctrl::slalom::Shape(
          ctrl::Pose(63.6396f * K, 63.6396f * K, 1.5708f),
          ctrl::Pose(48.0f * K, 48.0f * K, 1.5708f),
          15.6396f * K, 15.6396f * K,
          289.908f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case FK90:
      return ctrl::slalom::Shape(
          ctrl::Pose(127.279f * K, 127.279f * K, 1.5708f),
          ctrl::Pose(125.0f * K, 125.0f * K, 1.5708f),
          2.27925f * K, 2.27922f * K,
          754.969f * v_scale, 3769.91f, 113.097f, 9.42478f);
    case FS90:
    default:
      return ctrl::slalom::Shape(
          ctrl::Pose(45.0f * K, 45.0f * K, 1.5708f),
          ctrl::Pose(44.0f * K, 44.0f * K, 1.5708f),
          1.00004f * K, 1.0f * K,
          265.749f * v_scale, 3769.91f, 113.097f, 9.42478f);
  }
}

struct ShapesAccessor {
  ctrl::slalom::Shape operator[](size_t idx) const {
    return get_shape(static_cast<ShapeIndex>(idx));
  }
  ctrl::slalom::Shape operator[](ShapeIndex idx) const {
    return get_shape(idx);
  }
  size_t size() const { return ShapeIndexMax; }
};

static const ShapesAccessor shapes;

}  // namespace field
