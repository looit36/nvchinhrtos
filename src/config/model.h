/**
 * @file model.h
 * @brief Robot Physical Model & Control Parameters
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <ctrl/feedback_controller.h>
#include <ctrl/polar.h>
#include <ctrl/trajectory_tracker.h>

/* Math Constants */
#ifndef PI
static constexpr float PI = 3.14159265358979323846f;
#endif

namespace field {

/**
 * @brief Kích thước ô mê cung (mm).
 * Full-size = 180.0f (18cm x 18cm)
 */
extern float SegWidthFull;

inline float getSegWidthFull() { return SegWidthFull; }
inline float getSegWidthHalf() { return SegWidthFull / 2.0f; }
inline float getSegWidthDiag() { return SegWidthFull * 1.414213562373f; }
static constexpr float SegWidthDiag = 254.5584412f; // SegWidthFull * sqrt(2)
static constexpr float WallThickness = 12.0f;

} // namespace field

namespace model {

/* Physical Robot Dimensions */
static constexpr float RotationRadius =
    34.0f; // mm (khoảng cách tâm xe đến bánh)
static constexpr float WheelDiameter = 26.0f; // mm (đường kính bánh xe thực tế)
static constexpr float MotorTrim =
    0.920f; // Tỉ lệ cân bằng 2 động cơ (Right / Left, từ SysID: 0.962)
static constexpr float TailLength = 15.0f;   // mm (đuôi xe)
static constexpr float CenterOffsetY = 0.0f; // mm
static constexpr float turn_back_gain = 10.0f;
static constexpr float front_wall_attach_gain = 30.0f;
static constexpr float front_wall_attach_end = 0.4f;
static constexpr float wall_fix_offset = -5.0f;
static constexpr float wall_avoid_alpha = 0.05f;
static constexpr float wall_fix_theta_gain = 1e-8f;
static constexpr float EncoderPPR = 1024.0f;   // MT6701 PPR
static constexpr float CountsPerRev = 4096.0f; // 1024 * 4X hardware decode
static constexpr float GearRatio = 1.0f;       // Tỷ số truyền
static constexpr float ScalePulsesToMm =
    (PI * WheelDiameter) / (CountsPerRev * GearRatio); // ~0.01994 mm/count

/* Battery ADC Divider (PB0 / ADC1_IN8) */
static constexpr float BatteryR1 = 47000.0f; // 47k to Vbat
static constexpr float BatteryR2 = 10000.0f; // 10k to GND
static constexpr float BatteryDividerRatio =
    BatteryR2 / (BatteryR1 + BatteryR2);
static constexpr float AdcFSR = 4095.0f;
static constexpr float AdcRefVolts = 3.30f;
static constexpr float BatteryMultiplier =
    (AdcRefVolts / AdcFSR) / BatteryDividerRatio;
static constexpr float BatteryMinVoltage =
    6.4f; // Ngưỡng pin 2S cạn (3.2V/cell)
static constexpr float BatteryNominalVoltage = 7.4f;

/* Motor Models & System Identification (mm/s/V & rad/s/V) */
static constexpr ctrl::FeedbackController<ctrl::Polar>::Model
    SpeedControllerModel = {
        .K1 = ctrl::Polar(1180.0f, 7.54f),
        .T1 = ctrl::Polar(0.256f, 0.130f),
};

static constexpr ctrl::FeedbackController<ctrl::Polar>::Gain
    SpeedControllerGain = {
        .Kp = ctrl::Polar(0.0078f, 0.550f),
        .Ki = ctrl::Polar(0.136f, 10.0f),
        .Kd = ctrl::Polar(0.0f, 0.0f),
};

/* Velocity Estimation Filter (alpha = 1.0: 100% encoder & gyro, bypass noisy
 * accelerometer) */
static constexpr ctrl::Polar velocity_filter_alpha = ctrl::Polar(0.9f, 1.0f);

/* Trajectory Tracking Gains */
static constexpr ctrl::TrajectoryTracker::Gain TrajectoryTrackerGain = {
    .zeta = 0.8f,
    .omega_n = 8.0f,
    .low_zeta = 0.5f,
    .low_b = 1e-3f,
};

/* Default Dynamics Limits */
static constexpr float v_search = 330.0f;    // mm/s
static constexpr float a_search = 3600.0f;   // mm/s^2
static constexpr float j_search = 240000.0f; // mm/s^3

static constexpr float v_fast = 1200.0f; // mm/s
static constexpr float a_fast = 2000.0f; // mm/s^2
static constexpr float j_fast = 240.0f;  // mm/s^3

/* Spin Turn Parameters (rad-based) */
static constexpr float spin_omega = 2.0f * PI;  // rad/s (~360 deg/s)
static constexpr float spin_alpha = 20.0f * PI; // rad/s^2
static constexpr float spin_jerk = 400.0f * PI; // rad/s^3

} // namespace model
