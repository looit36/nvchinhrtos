#ifndef CONFIG_H
#define CONFIG_H

#define PI 3.1415926535897932384626433832795
#define BTSerial Serial1

// ==========================================
// BATTERY MONITORING
// ==========================================
#define VBAT PB0 // ADC1_IN8

const float BATTERY_R1 = 47000.0f; // resistor to battery + (47k)
const float BATTERY_R2 = 10000.0f; // resistor to Gnd (10k)
const float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
const float ADC_FSR = 4095.0f;     // The maximum reading for the 12-bit ADC
const float ADC_REF_VOLTS = 3.30f; // Reference voltage of ADC (3.3V)
const float BATTERY_MULTIPLIER =
    (ADC_REF_VOLTS / ADC_FSR / BATTERY_DIVIDER_RATIO);
const float MAX_MOTOR_VOLTS = 7.7f;

// ==========================================
// MOTOR PINS (DRV8833 - All on TIM4 Hardware PWM)
// Left Motor:  IN1=PB8 (TIM4_CH3), IN2=PB9 (TIM4_CH4)
// Right Motor: IN1=PB6 (TIM4_CH1), IN2=PB7 (TIM4_CH2)
// ==========================================
#define MOTOR_L_IN1 PB8
#define MOTOR_L_IN2 PB9
#define MOTOR_R_IN1 PB6
#define MOTOR_R_IN2 PB7

// Đảo chiều motor nếu đấu ngược dây động cơ (+1: thuận, -1: nghịch)
#define MOTOR_L_DIR 1
#define MOTOR_R_DIR -1

// ==========================================
// MOTOR PWM FREQUENCY & DUTY LIMITS (Zirconia Standard)
// ==========================================
#define MOTOR_PWM_FREQ 100000 // 100 kHz PWM (Siêu êm, giảm dòng gợn motor)
#define MOT_DUTY_MIN 30       // Duty tối thiểu 3.0% (trên thang 1000)
#define MOT_DUTY_MAX 950      // Duty tối đa 95.0% (trên thang 1000)

// ==========================================
// ENCODER PINS (STM32 Hardware Timer Encoder)
// Left:  TIM3_CH1 (PB4), TIM3_CH2 (PB5)
// Right: TIM2_CH1 (PA15), TIM2_CH2 (PB3)
// ==========================================
#define ENCAL PB4
#define ENCBL PB5
#define ENCAR PA15
#define ENCBR PB3

// ==========================================
// PHYSICAL PARAMETERS (MT6701 - 1024 PPR 4X Geared Drive: 11T / 42T, D=26mm)
// ==========================================
const float WHEEL_DIAMETER = 26.0f; // mm (Đường kính bánh xe thực tế)
const float ENCODER_PPR =
    1024.0f; // MT6701 PPR (Pulses Per Revolution trên trục motor)
const float COUNTS_PER_REV =
    4096.0f; // 1024 PPR * 4X Hardware Timer Decode = 4096 ticks/vòng motor
const float GEAR_RATIO =
    1; // Tỷ số truyền: 1 vòng motor = 1 vòng bánh xe (encoder gắn trên bánh xe)
const float MOUSE_RADIUS = 34.0f; // mm (half of wheel track)

const float MM_PER_COUNT =
    (PI * WHEEL_DIAMETER) / (COUNTS_PER_REV * GEAR_RATIO);
const float DEG_PER_MM_DIFFERENCE = (180.0f / (2.0f * MOUSE_RADIUS * PI));

// ==========================================
// MAZE / CELL DIMENSIONS
// Chỉnh 1 dòng này khi đổi mê cung:
//   Halfsize = 90.0f | Fullsize = 180.0f | Custom = tùy ý (vd 300.0f)
// ==========================================
const float FULL_CELL = 300.0f;                     // mm
const float METRIC_FULL_CELL = FULL_CELL / 1000.0f; // m
const float HALF_CELL = FULL_CELL / 2.0;
const float METRIC_HALF_CELL = METRIC_FULL_CELL / 2.0f;

// ==========================================
// CONTROL LOOP TIMING
// ==========================================
const float LOOP_FREQUENCY = 1000.0;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

// ==========================================
// MOTOR MODEL (System Identification)
// ==========================================
// Translation
const float FWD_KM = 1205.0f; // mm/s/Volt (motor gain)
const float FWD_TM = 0.240f;  // s (motor time constant)
// Rotation (Cập nhật từ Teleplot Log thực tế trên mặt sàn)
// const float ROT_KM_DEG = 105.0f; // deg/s/Volt (Hệ số tải thực tế trên mặt
// sàn)
const float ROT_KM = 7.54f;  // rad/s/Volt (~1.83 rad/s/Volt)
const float ROT_TM = 0.130f; // s

// ==========================================
// FEEDFORWARD GAINS
// ==========================================
const float SPEED_FF = (1.0 / FWD_KM);      // Volt / (mm/s)
const float BIAS_FF = (373.41286 / FWD_KM); // Static friction voltage
const float ACC_FF = (FWD_TM / FWD_KM);     // Acceleration feedforward

// ==========================================
// TRANSLATION PI (Pole-Zero Cancellation)
// ==========================================
// Bandwidth ~20 rad/s. Ki = 20 / FWD_KM = 0.045. Kp = FWD_TM * Ki = 0.011
const float FWD_KP = 0.0135f; // Proportional on velocity error
const float FWD_KI = 0.403f;  // Integral on velocity error

// ==========================================
// ROTATION PI (Pole-Zero Cancellation)
// ==========================================
// Bandwidth ~25 rad/s. Ki = 25 / ROT_KM = 13.6. Kp = ROT_TM * Ki = 3.97
const float ROT_KP = 0.971f; // Proportional on angular velocity error
const float ROT_KI = 27.6f;  // Integral on angular velocity error

// ==========================================
// SPIN TURN DYNAMICS (rad-based)
// ==========================================
const float SPIN_TURN_OMEGA = 2.0f * PI;  // rad/s - max angular velocity
const float SPIN_TURN_ALPHA = 20.0f * PI; // rad/s^2 - max angular acceleration
const float SPIN_TURN_JERK = 400.0f * PI; // rad/s^3 - angular jerk

// ==========================================
// VELOCITY ESTIMATION FILTER
// Complementary filter alpha:
//   est_v = alpha * raw + (1-alpha) * est_v_prev
//   alpha = 1.0 → pure sensor (no filter)
//   alpha = 0.2 → heavy smoothing
// ==========================================
const float VELOCITY_FILTER_ALPHA_TRA = 0.95f;
const float VELOCITY_FILTER_ALPHA_ROT = 1.0f;

// ==========================================
// SEARCH / FAST RUN SPEEDS — VẬN TỐC THỰC (m/s)
// Đặt trực tiếp vận tốc mong muốn, KHÔNG cần tính scale.
// Code slalom tự điều chỉnh giới hạn góc theo FULL_CELL và V_SLALOM.
//
// Gợi ý an toàn khi tăng V_SLALOM: gia tốc hướng tâm khi cua
//   a_lat ≈ 2.7 * V_SLALOM^2 / (FULL_CELL/1000)  [m/s^2]
// giữ a_lat < ~3 m/s^2 nếu không có quạt hút.
// (FULL_CELL=300: 0.60 m/s → 3.2 m/s^2; 0.50 m/s → 2.2 m/s^2)
// ==========================================
const float METRIC_SEARCH_SPEED = 0.33f; // m/s (search velocity)
const float METRIC_FAST_SPEED = 5.0f;    // m/s (fast run, đoạn thẳng)
const float METRIC_SLALOM_SPEED = 1.3f; // m/s (vận tốc THỰC khi ôm cua) - tăng
                                        // từ 1.3 để cho phép vận tốc cao hơn
const float METRIC_TURN_SPEED =
    METRIC_SLALOM_SPEED;               // m/s (slalom turn velocity)
const float METRIC_ACCEL = 3.6f;       // m/s^2 (search acceleration)
const float METRIC_FAST_ACCEL = 10.0f; // m/s^2 (fast run acceleration)
const float METRIC_JERK = 240.0f;     // m/s^3 (jerk)

// ==========================================
// IMU (BMI160 via SPI2)
// ==========================================
#define MPU_ADDR 0x68
const float GYRO_SCALE_FACTOR = 1.013f;
const int IMU_CALIBRATION_SAMPLES =
    500; // Number of samples for runtime calibration

extern volatile float g_robot_angle_mpu;

#endif
