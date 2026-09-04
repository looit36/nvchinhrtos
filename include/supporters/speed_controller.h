#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ctrl/pose.h>

class SpeedController {
public:
    // === State Variables (read from other tasks) ===
    volatile float ref_v = 0;   // Reference velocity (mm/s)
    volatile float ref_w = 0;   // Reference angular velocity (rad/s)
    volatile float ref_dv = 0;  // Reference acceleration (mm/s^2)
    volatile float ref_dw = 0;  // Reference angular acceleration (rad/s^2)

    volatile float est_v = 0;   // Estimated velocity (mm/s)
    volatile float est_w = 0;   // Estimated angular velocity (rad/s)
    volatile float est_dv = 0;  // Estimated acceleration (mm/s^2)
    volatile float est_dw = 0;  // Estimated angular acceleration (rad/s^2)
    
    ctrl::Pose est_p;           // Estimated pose (x, y, th in mm, mm, rad)

    volatile float err_v_i = 0; // Integral of velocity error
    volatile float err_w_i = 0; // Integral of angular velocity error
    
    volatile float ref_th_local = 0; // Local reference heading for exact Kerise failsafe

    volatile float battery_voltage = 8.0f; // Measured battery voltage (V)

    // Manual open-loop override (cho motor testing & diagnostics)
    volatile bool manual_override = false;
    volatile int manual_pwm_L = 0;
    volatile int manual_pwm_R = 0;

    void set_manual_override(bool enable) {
        taskENTER_CRITICAL();
        manual_override = enable;
        if (!enable) {
            manual_pwm_L = 0;
            manual_pwm_R = 0;
        }
        taskEXIT_CRITICAL();
    }

    void set_manual_pwm(int pwmL, int pwmR) {
        taskENTER_CRITICAL();
        manual_pwm_L = pwmL;
        manual_pwm_R = pwmR;
        taskEXIT_CRITICAL();
    }

    // Telemetry output variables for Teleplot
    volatile float telemetry_ff_rot = 0;
    volatile float telemetry_fb_rot = 0;
    volatile int telemetry_pwm_L = 0;
    volatile int telemetry_pwm_R = 0;

    void init();

    // Thread-safe setters using FreeRTOS critical sections
    void set_target(float v, float w, float dv = 0, float dw = 0) {
        taskENTER_CRITICAL();
        // BUG FATAL: RefV bị đẩy lên 17000 mm/s do TrajectoryTracker đòi bù sai số.
        // Clamp tối đa 1500 mm/s để luôn chừa lại áp dư cho motor bẻ lái!
        ref_v = constrain(v, -1500.0f, 1500.0f); 
        ref_w = constrain(w, -15.0f, 15.0f);
        ref_dv = dv; 
        ref_dw = dw;
        taskEXIT_CRITICAL();
    }

    void update_pose(const ctrl::Pose& p) {
        taskENTER_CRITICAL();
        float delta_th = p.th - est_p.th; 
        est_p = p;
        ref_th_local += delta_th; // Rotate the failsafe reference by the exact same amount to maintain invariant error!
        taskEXIT_CRITICAL();
    }
    void reset();

private:
    static void task_trampoline(void *pvParameters);
    void task();
    float read_battery_voltage();
};

void init_battery_dma();

extern SpeedController speedCtrl;

