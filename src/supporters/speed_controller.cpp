#include "supporters/speed_controller.h"
#include "hardware/motor.h"
#include "MotorEncoder.h"
#include <mpu.h>
#include <config.h>

SpeedController speedCtrl;

// Khai báo ngoài
extern MotorEncoder encLeft;
extern MotorEncoder encRight;
extern volatile float g_robot_angle_mpu;


#define VBAT_DMA_SAMPLES 16
static volatile uint16_t s_vbat_dma_buffer[VBAT_DMA_SAMPLES] = {0};

void init_battery_dma()
{
    // 1. Kích hoạt Clock ngoại vi: GPIOB, ADC1, DMA2
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // 2. Cấu hình chân PB0 (ADC1_IN8) chế độ Analog
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. Cấu hình DMA2 Stream 0 Channel 0 (Kênh DMA chuẩn của ADC1 trên STM32F411)
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN)
    {
        // Chờ DMA stream tắt hoàn toàn
    }

    // Xóa cờ ngắt cũ của Stream 0
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

    DMA2_Stream0->PAR = (uint32_t)&(ADC1->DR);
    DMA2_Stream0->M0AR = (uint32_t)s_vbat_dma_buffer;
    DMA2_Stream0->NDTR = VBAT_DMA_SAMPLES;

    // Cấu hình: Channel 0, Ưu tiên Medium, 16-bit Memory, 16-bit Peripheral, Tự tăng địa chỉ RAM (MINC), Vòng tròn (CIRC)
    DMA2_Stream0->CR = (0 << DMA_SxCR_CHSEL_Pos) | 
                       (1 << DMA_SxCR_PL_Pos)    | 
                       (1 << DMA_SxCR_MSIZE_Pos) | 
                       (1 << DMA_SxCR_PSIZE_Pos) | 
                       DMA_SxCR_MINC             | 
                       DMA_SxCR_CIRC;

    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // 4. Cấu hình ADC1
    // Prescaler ADC clock = PCLK2 / 4 (100MHz / 4 = 25MHz <= 36MHz max)
    ADC->CCR = (1 << ADC_CCR_ADCPRE_Pos);

    ADC1->CR1 = 0; // 12-bit resolution

    // Thời gian lấy mẫu Channel 8 (PB0) = 480 cycles (SMP8 = 0b111)
    // Đảm bảo tụ lấy mẫu nạp đầy qua trở kháng cầu phân áp 47k/10k
    ADC1->SMPR2 = (7 << (8 * 3));

    // Regular sequence: 1 conversion, chọn Channel 8
    ADC1->SQR1 = (0 << ADC_SQR1_L_Pos);
    ADC1->SQR3 = (8 << ADC_SQR3_SQ1_Pos);

    // Kích hoạt ADC1: ADON, Continuous mode (CONT), DMA enable (DMA), DMA continuous request (DDS)
    ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;

    delayMicroseconds(100);

    // Bắt đầu chu trình chuyển đổi tự hành
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void SpeedController::init()
{
    // Khởi tạo TIM4 Hardware PWM (Zirconia driver module)
    Motor_Initialize();

    // Khởi tạo ADC1 + DMA2 Continuous Battery Monitoring
    init_battery_dma();

    // Chờ 2ms để DMA nạp đầy 16 mẫu đầu tiên
    delay(2);
    battery_voltage = read_battery_voltage();
    if (battery_voltage < 3.0f || battery_voltage > 13.0f)
    {
        battery_voltage = 8.0f; // Fallback an toàn
    }

    xTaskCreate(task_trampoline, "ControlLoop", 1024, this, 4, NULL);
}

void SpeedController::reset()
{
    taskENTER_CRITICAL();
    est_v = 0;
    est_w = 0;
    est_dv = 0;
    est_dw = 0;
    ref_v = 0;
    ref_w = 0;
    ref_dv = 0;
    ref_dw = 0;
    err_v_i = 0;
    err_w_i = 0;
    est_p.x = 0;
    est_p.y = 0;
    est_p.th = 0;
    ref_th_local = 0;
    taskEXIT_CRITICAL();
}

float SpeedController::read_battery_voltage()
{
    uint32_t sum = 0;
    for (int i = 0; i < VBAT_DMA_SAMPLES; i++)
    {
        sum += s_vbat_dma_buffer[i];
    }
    float raw_avg = (float)sum / (float)VBAT_DMA_SAMPLES;
    return raw_avg * BATTERY_MULTIPLIER;
}

void SpeedController::task_trampoline(void *pvParameters)
{
    static_cast<SpeedController *>(pvParameters)->task();
}

void SpeedController::task()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    long prev_encL = encLeft.getCount();
    long prev_encR = encRight.getCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // === 1. ĐỌC CẢM BIẾN ===
        read_mpu_data(0.001f);

        long current_encL = encLeft.getCount();
        long current_encR = encRight.getCount();

        float dL = (current_encL - prev_encL) * MM_PER_COUNT;
        float dR = (current_encR - prev_encR) * MM_PER_COUNT;
        prev_encL = current_encL;
        prev_encR = current_encR;

        // === 2. ƯỚC LƯỢNG TRẠNG THÁI ===
        float dTra = (dR + dL) / 2.0f;
        float raw_v = dTra / 0.001f;

        float raw_w = get_yaw_rate(); // rad/s trực tiếp từ gyro

        // Tính tọa độ Pose (Kerise-style Local Odometry)
        // Lưu ý: Không dùng absolute get_yaw_angle() vì hệ toạ độ local sẽ bị reset sau mỗi lệnh!
        est_p.th += raw_w * 0.001f;
        est_p.x += dTra * cos(est_p.th);
        est_p.y += dTra * sin(est_p.th);

        // Complementary filter (Kerise style)
        // v_low = sensor reading (encoder for tra, gyro for rot)
        // v_high = previous estimate + accelerometer * Ts (prediction)
        // est = alpha * v_low + (1-alpha) * v_high
        float v_high_tra = est_v + get_linear_accel() * 0.001f;  // Predict from accel
        float v_high_rot = est_w + get_angular_accel() * 0.001f; // Predict from angular accel
        est_v = VELOCITY_FILTER_ALPHA_TRA * raw_v + (1.0f - VELOCITY_FILTER_ALPHA_TRA) * v_high_tra;
        est_w = VELOCITY_FILTER_ALPHA_ROT * raw_w + (1.0f - VELOCITY_FILTER_ALPHA_ROT) * v_high_rot;

        // Estimated acceleration from IMU (Kerise: est_a = accel[0])
        est_dv = get_linear_accel();  // mm/s^2
        est_dw = get_angular_accel(); // rad/s^2

        g_robot_angle_mpu = get_yaw_angle();

        // === 2.1 MANUAL OPEN-LOOP OVERRIDE (Dùng cho Diagnostics / Motor Test) ===
        if (manual_override)
        {
            Motor_SetDuty_Left(manual_pwm_L);
            Motor_SetDuty_Right(manual_pwm_R);
            telemetry_pwm_L = manual_pwm_L;
            telemetry_pwm_R = manual_pwm_R;
            continue;
        }

        // === 3. ĐỌC REFERENCE TỪ TRAJECTORY TRACKER ===
        // Đọc ref an toàn (sử dụng critical section ở phía setter)
        float local_ref_v = ref_v;
        float local_ref_w = ref_w;
        float local_ref_dv = ref_dv;
        float local_ref_dw = ref_dw;

        // Reset tích phân khi robot đứng yên (tránh trôi tích phân do nhiễu Gyro khi idle)
        if (fabsf(local_ref_v) < 1.0f && fabsf(local_ref_w) < 0.01f)
        {
            err_v_i = 0.0f;
            err_w_i = 0.0f;
        }

        // === 4. FEEDFORWARD + PI (TRANSLATION) — Kerise-style ===
        float err_v = local_ref_v - est_v;
        err_v_i += err_v * 0.001f;                   // Tích phân sai số vận tốc
        err_v_i = constrain(err_v_i, -50.0f, 50.0f); // Anti-windup

        // Feedforward: (T1 * ref_accel + ref_velocity) / K1
        float ff_tra = (FWD_TM * local_ref_dv + local_ref_v) * SPEED_FF;
        if (local_ref_v > 1.0f)
            ff_tra += BIAS_FF;
        else if (local_ref_v < -1.0f)
            ff_tra -= BIAS_FF;

        // Feedback: Kp * velocity_error + Ki * integral(velocity_error)
        float u_tra = FWD_KP * err_v + FWD_KI * err_v_i + ff_tra;

        // === 5. FEEDFORWARD + PI (ROTATION) — Kerise-style ===
        float err_w = local_ref_w - est_w;
        err_w_i += err_w * 0.001f;                   // Tích phân sai số vận tốc góc
        err_w_i = constrain(err_w_i, -10.0f, 10.0f); // Anti-windup (Tránh bão hòa khi ôm cua gắt)

        // Feedforward: (T1 * ref_angular_accel + ref_angular_velocity) / K1
        float ff_rot = (ROT_TM * local_ref_dw + local_ref_w) * (1.0f / ROT_KM);

        // Feedback: Kp * angular_velocity_error + Ki * integral(angular_velocity_error)
        float fb_rot = ROT_KP * err_w + ROT_KI * err_w_i;

        if (fabsf(local_ref_v) < 1.0f && fabsf(local_ref_w) < 0.01f)
        {
            u_tra = 0.0f;
            ff_rot = 0.0f;
            fb_rot = 0.0f;
        }

        // === 6. MOTOR OUTPUT ===
        // Cả Feedforward và Feedback đều phải chia 2 để bảo toàn V_diff = V_R - V_L = ff_rot + fb_rot
        float vL = u_tra - (ff_rot + fb_rot) / 2.0f;
        float vR = u_tra + (ff_rot + fb_rot) / 2.0f;

        // Lưu telemetry cho Teleplot
        telemetry_ff_rot = ff_rot;
        telemetry_fb_rot = fb_rot;

        // Cập nhật góc tham chiếu nội bộ (sử dụng cho Failsafe chuẩn Kerise)
        ref_th_local += local_ref_w * 0.001f;

        // === 6.1 ĐO & BÙ ĐIỆN ÁP PIN TỰ HÀNH (ADC1 + DMA2 @ 1kHz) ===
        float instant_vbat = read_battery_voltage();
        if (instant_vbat > 3.0f && instant_vbat < 13.0f)
        {
            // Bộ lọc thông thấp số IIR (EMA) alpha = 0.05 (~8Hz cutoff @ 1kHz)
            // Bám sát sụt áp tải động cơ (voltage sag ~20ms) và triệt tiêu xung nhiễu PWM 20kHz
            battery_voltage = 0.05f * instant_vbat + 0.95f * battery_voltage;
        }

        float v_bat = battery_voltage;
        if (v_bat < 3.0f) v_bat = 8.0f; // Fallback an toàn

        // Quy đổi điện áp sang Duty [-1000, 1000] theo chuẩn Zirconia
        int duty_L = constrain((vL / v_bat) * 1000.0f, -1000.0f, 1000.0f);
        int duty_R = constrain((vR / v_bat) * 1000.0f, -1000.0f, 1000.0f);

        telemetry_pwm_L = duty_L;
        telemetry_pwm_R = duty_R;

        // === 7. FAILSAFE / CRASH DETECTION (Chuẩn Kerise V4) ===
        static int failsafe_counter = 0;
        bool is_failsafe = false;

        // 1. Lỗi kẹt bánh (Stuck) - Y hệt Kerise
        if (abs(local_ref_v) > 100.0f && abs(est_v) < 50.0f)
        {
            failsafe_counter++;
        }
        else
        {
            failsafe_counter = 0;
        }

        if (failsafe_counter > 500)
        { // 500ms
            is_failsafe = true;
        }

        // 2. Lỗi mất lái góc (Out of control) - Y hệt Kerise
        // Công thức chuẩn Kerise V4: abs(est_p.th - ref_p.th) > 3pi/4.
        // Bằng cách track ref_th_local đồng bộ với est_p.th, ta có được con số chính xác hoàn đối!
        if (abs(est_p.th - ref_th_local) > 3.0f * PI / 4.0f)
        {
            is_failsafe = true;
        }

        if (is_failsafe)
        {
            Motor_StopPWM();

            // Khóa chết hệ thống, nháy LED cảnh báo
            pinMode(PC13, OUTPUT);
            while (1)
            {
                digitalWrite(PC13, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(PC13, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        // === 8. DRIVE MOTORS (DRV8833 TIM4 PWM @ 100kHz) ===
        Motor_SetDuty_Left(duty_L);
        Motor_SetDuty_Right(duty_R);
    }
}

