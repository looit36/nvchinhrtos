/**
 * @file math_utils.hpp
 * @brief Math Utilities
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 */
#pragma once

#include <cmath>
#include <algorithm>

namespace math_utils {

static inline float round2(float value, float div) {
  return std::floor((value + div / 2.0f) / div) * div;
}

static inline float saturate(float src, float sat) {
  return std::max(std::min(src, sat), -sat);
}

static inline float sum_of_square(float v1, float v2) {
  return v1 * v1 + v2 * v2;
}

}  // namespace math_utils
