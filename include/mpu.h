#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

void setup_mpu_manual();
void calibrate_imu();     // Runtime calibration (call at startup, robot stationary)
void read_mpu_data(float dt);
float get_yaw_angle();
float get_yaw_rate();
float get_linear_accel();
float get_sideways_accel();
float get_angular_accel();
void reset_angle();
void reset_fused_data();

extern float RateRoll, RatePitch, RateYaw;
extern float AccelX, AccelY, AccelZ;

#endif
