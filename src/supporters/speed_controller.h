/**
 * @file speed_controller.h
 * @brief Speed Controller using FeedbackController & Complementary Filter
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "config/model.h"
#include "config/wheel_parameter.h"
#include "hardware/hardware.h"

#include <ctrl/accumulator.h>
#include <ctrl/feedback_controller.h>
#include <ctrl/polar.h>
#include <ctrl/pose.h>
#include <freertospp/mutex.h>
#include <freertospp/semphr.h>
#include <mutex>

class SpeedController {
 public:
  static constexpr const float Ts = 1e-3f;
  static constexpr int acc_num = 4;

 public:
  ctrl::Polar ref_v;
  ctrl::Polar ref_a;
  ctrl::Polar est_v;
  ctrl::Polar est_a;
  ctrl::Pose est_p;
  WheelParameter enc_v;
  ctrl::Accumulator<float, acc_num> wheel_position[2];
  ctrl::Accumulator<ctrl::Polar, acc_num> accel;
  ctrl::FeedbackController<ctrl::Polar> fbc;

 private:
  hardware::Hardware* hw;

 public:
  SpeedController(hardware::Hardware* hw)
      : fbc(model::SpeedControllerModel, model::SpeedControllerGain), hw(hw) {
    reset();
  }

  bool init() {
    xTaskCreate(
        [](void* arg) { static_cast<SpeedController*>(arg)->task(); },
        "SpeedCtrl", 1024, this, 5, NULL);
    return true;
  }

  void reset() {
    {
      std::lock_guard<freertospp::Mutex> lock(mutex);
      ref_v.clear();
      ref_a.clear();
      est_v.clear();
      est_a.clear();
      est_p.clear();
      enc_v.clear();
      for (int i = 0; i < 2; i++)
        wheel_position[i].clear(hw->enc->get_position(i));
      accel.clear({hw->imu->get_accel(), hw->imu->get_angular_accel()});
      fbc.reset();
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      delay(50);
    }
  }

  void enable() {
    reset();
    drive_enabled = true;
  }

  void disable() {
    drive_enabled = false;
    sampling_sync();
    sampling_sync();
    hw->mt->free();
    hw->fan->drive(0);
  }

  void set_target(float v_tra, float v_rot, float a_tra = 0, float a_rot = 0) {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    ref_v.tra = v_tra;
    ref_v.rot = v_rot;
    ref_a.tra = a_tra;
    ref_a.rot = a_rot;
    if (drive_enabled)
      drive();
  }

  void fix_pose(ctrl::Pose fix, bool force = true) {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    if (!force) {
      const float max_fix = 1.0f;
      fix.x = std::max(std::min(fix.x, max_fix), -max_fix);
      fix.y = std::max(std::min(fix.y, max_fix), -max_fix);
    }
    est_p += fix;
  }

  void update_pose(const ctrl::Pose& new_pose) {
    std::lock_guard<freertospp::Mutex> lock(mutex);
    est_p = new_pose;
  }

  bool sampling_sync(portTickType xBlockTime = pdMS_TO_TICKS(50)) const {
    return data_ready_semaphore.take(xBlockTime);
  }

  bool is_enabled() const { return drive_enabled; }

 private:
  volatile bool drive_enabled = false;
  mutable freertospp::Semaphore data_ready_semaphore;
  freertospp::Mutex mutex;

  void task() {
    while (1) {
      /* sampling sync from sensors */
      hw->imu->sampling_sync();
      hw->enc->sampling_sync();

      /* update state estimation */
      {
        std::lock_guard<freertospp::Mutex> lock(mutex);
        update_samples();
        update_estimator();
        update_odometry();
      }

      /* notify trajectory tracker and controller */
      data_ready_semaphore.give();

      /* update motor control */
      if (drive_enabled)
        drive();
    }
  }

  void update_samples() {
    for (int i = 0; i < 2; i++)
      wheel_position[i].push(hw->enc->get_position(i));
    accel.push({hw->imu->get_accel(), hw->imu->get_angular_accel()});
  }

  void update_estimator() {
    for (int i = 0; i < 2; i++)
      enc_v.wheel[i] = (wheel_position[i][0] - wheel_position[i][1]) / Ts;
    enc_v.wheel2pole();

    const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, hw->imu->get_gyro());
    const ctrl::Polar v_high = est_v + accel[0] * float(Ts);
    const ctrl::Polar alpha = model::velocity_filter_alpha;
    est_v = alpha * v_low + (ctrl::Polar(1.0f, 1.0f) - alpha) * v_high;
    est_a = accel[0];
  }

  void update_odometry() {
    const float k = 0.0f;
    const float slip_angle = k * ref_v.tra * ref_v.rot / 1000.0f;
    est_p.th += hw->imu->get_gyro() * Ts;
    est_p.x += enc_v.tra * std::cos(est_p.th + slip_angle) * Ts;
    est_p.y += enc_v.tra * std::sin(est_p.th + slip_angle) * Ts;
  }

  void drive() {
    // FeedbackController tính ra điện áp điều khiển (Volts)
    const auto u = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
    float vbat = hw->getBatteryVoltage();
    if (vbat < 5.0f || vbat > 12.0f) vbat = 7.4f;

    // Chuẩn hóa từ Volt sang Duty [-1.0, 1.0] và bão hòa an toàn
    float duty_L = (u.tra - u.rot / 2.0f) / vbat;
    float duty_R = (u.tra + u.rot / 2.0f) / vbat;

    duty_L = std::clamp(duty_L, -0.95f, 0.95f);
    duty_R = std::clamp(duty_R, -0.95f, 0.95f);

    hw->mt->drive(duty_L, duty_R);
  }
};
