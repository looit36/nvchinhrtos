#include "mpu.h"
#include <Arduino.h>
#include <SPI.h>
#include "config.h"

// Define SPI2 Chip Select Pin
#define MPU_CS PB12

float AccelX, AccelY, AccelZ;
float RateRoll, RatePitch, RateYaw;
double yaw_angle = 0.0;
static float s_current_yaw_rate = 0.0;
static float s_current_linear_accel = 0.0;
static float s_current_sideways_accel = 0.0;
static float s_current_angular_accel = 0.0;
static float s_prev_yaw_rate = 0.0;

// Calibration offsets (will be set by runtime calibration)
static float gyro_z_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float accel_z_offset = 0.0f;
static float accel_y_offset = 0.0f;

static double pitch_angle = 0.0;

// Instantiate SPIClass for Hardware SPI2
SPIClass hspi2(PB15, PB14, PB13); // MOSI, MISO, SCK

// Set SPI Settings for BMI160: 1 MHz for signal stability over jumper wires, Mode 0 (CPOL=0, CPHA=0)
SPISettings bmi160Settings(1000000, MSBFIRST, SPI_MODE0);

// SPI Helper Functions for BMI160
void writeRegisterSPI(uint8_t reg, uint8_t data)
{
  hspi2.beginTransaction(bmi160Settings);
  digitalWrite(MPU_CS, LOW);
  hspi2.transfer(reg);  // Write register address (Bit 7 is 0)
  hspi2.transfer(data); // Send data
  digitalWrite(MPU_CS, HIGH);
  hspi2.endTransaction();
}

uint8_t readRegisterSPI(uint8_t reg)
{
  hspi2.beginTransaction(bmi160Settings);
  digitalWrite(MPU_CS, LOW);
  hspi2.transfer(reg | 0x80);         // Read register address (Bit 7 is 1)
  uint8_t val = hspi2.transfer(0x00); // Read actual data (1st byte after address is the register value)
  digitalWrite(MPU_CS, HIGH);
  hspi2.endTransaction();
  return val;
}

void readRegistersSPI(uint8_t reg, uint8_t *buffer, uint8_t length)
{
  hspi2.beginTransaction(bmi160Settings);
  digitalWrite(MPU_CS, LOW);
  hspi2.transfer(reg | 0x80); // Read start register address (Bit 7 is 1)
  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = hspi2.transfer(0x00); // Read sequential registers
  }
  digitalWrite(MPU_CS, HIGH);
  hspi2.endTransaction();
}

void mpu_signals(void)
{
  uint8_t data[12];

  // BMI160 Data registers from 0x0C to 0x17:
  // 0x0C - 0x11: Gyro X, Y, Z LSB/MSB
  // 0x12 - 0x17: Accel X, Y, Z LSB/MSB
  readRegistersSPI(0x0C, data, 12);

  // BMI160 stores multi-byte values in Little-Endian format (LSB first)
  int16_t rawGX = (int16_t)((data[1] << 8) | data[0]);
  int16_t rawGY = (int16_t)((data[3] << 8) | data[2]);
  int16_t rawGZ = (int16_t)((data[5] << 8) | data[4]);
  int16_t rawAX = (int16_t)((data[7] << 8) | data[6]);
  int16_t rawAY = (int16_t)((data[9] << 8) | data[8]);
  int16_t rawAZ = (int16_t)((data[11] << 8) | data[10]);

  // Scale Accel for ±2g (16384 LSB/g) -> mm/s^2 (1g = 9807 mm/s^2)
  // New Horizontal Mounting: Z=Vertical (Up), X=Forward
  // (We map physical rawAX to AccelZ so that AccelZ continues to represent Forward acceleration in the code)
  AccelX = (float)rawAZ / 16384.0f * 9807.0f; // Physical Vertical axis (Z)
  AccelY = (float)rawAY / 16384.0f * 9807.0f; // Physical Sideways axis (Y)
  AccelZ = (float)rawAX / 16384.0f * 9807.0f; // Physical Forward axis (X)

  // Hóa ra DẤU TRỪ LÀ SAI!
  // Robot bị mất lái (saturate PWM) nên đâm tường và trượt bánh quay mòng mòng, chứ không phải do Gyro bị ngược!
  // Đã trả lại dấu dương chuẩn (CCW = Dương).
  RateYaw = (float)rawGZ / 16.4f;   
  RatePitch = (float)rawGY / 16.4f; // Physical Sideways axis (Y) for Pitch
}

bool mpu_found = false;

void setup_mpu_manual()
{
  delay(1000);

  // Configure Chip Select Pin
  pinMode(MPU_CS, OUTPUT);
  digitalWrite(MPU_CS, HIGH); // Start deselected

  // Configure SPI2 Pins explicitly to avoid core default conflicts
  hspi2.setMOSI(PB15);
  hspi2.setMISO(PB14);
  hspi2.setSCLK(PB13);

  // Initialize SPI2 Hardware Bus
  hspi2.begin();

  // Step 1: Perform a dummy read to force BMI160 into SPI mode at startup
  hspi2.beginTransaction(bmi160Settings);
  digitalWrite(MPU_CS, LOW);
  hspi2.transfer(0x7F | 0x80); // Read dummy register
  hspi2.transfer(0x00);        // Read value
  digitalWrite(MPU_CS, HIGH);
  hspi2.endTransaction();
  delay(10);

  // Step 2: Soft Reset the BMI160
  writeRegisterSPI(0x7E, 0xB6); // Write B6 to CMD (0x7E)
  delay(100);                   // Wait for the reset process to finish (the chip reboots and defaults back to I2C)

  // Step 3: MUST perform another dummy read after soft-reset to re-force SPI mode!
  // (Because soft reset defaults the chip back to I2C mode, the first SPI read will be ignored as a dummy read)
  readRegisterSPI(0x7F);
  delay(10); // Wait for interface to settle

  // Step 4: Now read CHIPID to verify connection
  uint8_t chip_id = readRegisterSPI(0x00); // CHIPID is at 0x00

  if (chip_id != 0xD1) // 0xD1 is the default Chip ID for BMI160
  {
    BTSerial.print("ERROR: BMI160 NOT FOUND AT SPI2! CHIP ID: 0x");
    BTSerial.println(chip_id, HEX);
    return;
  }
  mpu_found = true;
  BTSerial.println("BMI160 FOUND ON SPI2!");

  // Step 5: Wake up Accelerometer and Gyroscope from Suspend Mode
  writeRegisterSPI(0x7E, 0x11); // Power Up Accel to Normal Mode
  delay(10);                    // Startup delay for Accel (>3.8ms)

  writeRegisterSPI(0x7E, 0x15); // Power Up Gyro to Normal Mode
  delay(100);                   // Startup delay for Gyro (>80ms)

  // Step 6: Configure Accelerometer Range to ±2g (default 16384 LSB/g)
  writeRegisterSPI(0x41, 0x03); // Write 0x03 to ACC_RANGE
  delay(10);

  // Step 7: Configure Gyroscope Range to ±2000 deg/s (default 16.4 LSB/dps)
  writeRegisterSPI(0x43, 0x00); // Write 0x00 to GYR_RANGE
  delay(10);

  // Step 8: Configure Output Data Rate (ODR) and Filters
  writeRegisterSPI(0x40, 0x28); // ACC_CONF: Normal mode filter, 100Hz ODR
  delay(10);
  writeRegisterSPI(0x42, 0x1B); // GYR_CONF: OSR2 filter mode, 1000Hz (1kHz) ODR
  delay(10);

  // Step 9: Runtime Calibration — ROBOT PHẢI ĐỨNG YÊN HOÀN TOÀN!
  calibrate_imu();
}

void calibrate_imu()
{
  if (!mpu_found)
    return;

  BTSerial.println("Calibrating IMU... Keep robot stationary!");

  float gz_sum = 0, gy_sum = 0;
  float az_sum = 0, ay_sum = 0;
  int n = IMU_CALIBRATION_SAMPLES;

  // Bỏ 100 sample đầu (transient settling)
  for (int i = 0; i < 100; i++)
  {
    mpu_signals();
    delay(1);
  }

  // Lấy trung bình N sample
  for (int i = 0; i < n; i++)
  {
    mpu_signals();
    gz_sum += RateYaw;
    gy_sum += RatePitch;
    az_sum += AccelZ;
    ay_sum += AccelY;
    delay(1);
  }

  gyro_z_offset = gz_sum / n;
  gyro_y_offset = gy_sum / n;
  accel_z_offset = az_sum / n;
  accel_y_offset = ay_sum / n;

  BTSerial.print("BMI160 Calibrated. GyroZ Offset: ");
  BTSerial.print(gyro_z_offset);
  BTSerial.print(" | GyroY Offset: ");
  BTSerial.print(gyro_y_offset);
  BTSerial.print(" | AccelZ Offset: ");
  BTSerial.print(accel_z_offset);
  BTSerial.print(" | AccelY Offset: ");
  BTSerial.println(accel_y_offset);
}

void read_mpu_data(float dt)
{
  if (!mpu_found)
    return;
  mpu_signals();

  // Apply calibration (static offset)
  float currentRate = RateYaw - gyro_z_offset;
  float currentRatePitch = RatePitch - gyro_y_offset;

  // Convert yaw rate to rad/s
  float currentRateRad = currentRate * PI / 180.0f;
  float currentRatePitchRad = currentRatePitch * PI / 180.0f;

  // Calculate centripetal acceleration compensation for 10.0mm (1cm) forward offset
  // a_center = a_sensor + r * omega^2
  float centripetal_compensation = 10.0f * (currentRateRad * currentRateRad);
  
  // Simple integration for yaw angle
  yaw_angle += (double)currentRateRad * (double)dt;
  
  // BUG: Trên chuột chạy tốc độ cao, gia tốc tịnh tiến rất lớn (lên tới 1G).
  // Việc dùng atan2(-AccelZ, AccelX) sẽ nhầm lẫn gia tốc tịnh tiến thành góc nghiêng (Pitch), 
  // làm pitch_angle bị sai lệch nghiêm trọng (lên tới 45 độ), kéo theo gravity_compensation bị sai lệch tới 0.7G!
  // SỬA: Robot luôn chạy trên mặt phẳng phẳng, góc Pitch thực tế luôn = 0.
  pitch_angle = 0.0f; 
  float gravity_compensation = 0.0f;

  // Compute forward acceleration exactly at the center of rotation!
  float currentAccelForward = (AccelZ - accel_z_offset) + centripetal_compensation + gravity_compensation;

  // Output
  s_current_yaw_rate = currentRateRad;
  s_current_linear_accel = currentAccelForward;
  s_current_angular_accel = (currentRateRad - s_prev_yaw_rate) / dt;
  s_current_sideways_accel = (AccelY - accel_y_offset) - 10.0f * s_current_angular_accel;
  s_prev_yaw_rate = currentRateRad;
}

void reset_angle()
{
  yaw_angle = 0;
  pitch_angle = 0;
}

void reset_fused_data()
{
  yaw_angle = 0;
  pitch_angle = 0;
  s_current_linear_accel = 0;
  s_current_sideways_accel = 0;
  s_current_yaw_rate = 0;
  s_current_angular_accel = 0;
  s_prev_yaw_rate = 0;
}

float get_yaw_angle()
{
  return (float)yaw_angle;
}

float get_yaw_rate()
{
  return s_current_yaw_rate;
}

float get_linear_accel()
{
  return s_current_linear_accel;
}

float get_sideways_accel()
{
  return s_current_sideways_accel;
}

float get_angular_accel()
{
  return s_current_angular_accel;
}

