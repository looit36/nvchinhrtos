# 📖 Hướng Dẫn Sử Dụng API — Micromouse STM32 RTOS

## Mục Lục

1. [Kiến Trúc Tổng Quan](#1-kiến-trúc-tổng-quan)
2. [SpeedController — Điều Khiển Tốc Độ](#2-speedcontroller)
3. [MoveAction — Hành Động Di Chuyển](#3-moveaction)
4. [Commander — Giao Tiếp Serial](#4-commander)
5. [IMU (BMI160)](#5-imu-bmi160)
6. [Encoder](#6-encoder)
7. [Config — Hằng Số Quan Trọng](#7-config)
8. [Lưu Ý Quan Trọng](#8-lưu-ý-quan-trọng)

---

## 1. Kiến Trúc Tổng Quan

```
┌─────────────────────────────────────────────────┐
│                  FreeRTOS Tasks                  │
├──────────┬──────────┬──────────┬─────────────────┤
│ Control  │ Action   │ Serial   │ Teleplot        │
│ Loop     │ Task     │ Task     │ Task            │
│ @1kHz    │          │          │ @100Hz          │
│ Prio: 4  │ Prio: 3  │ Prio: 2  │ Prio: 1         │
├──────────┴──────────┴──────────┴─────────────────┤
│ SpeedCtrl  MoveAction  Commander                 │
├──────────────────────────────────────────────────┤
│ Hardware: Motor, Encoder, IMU, Battery ADC       │
└──────────────────────────────────────────────────┘
```

**Luồng dữ liệu:**
1. `Commander` nhận lệnh từ Bluetooth Serial
2. `MoveAction` tạo trajectory profile (AccelDesigner)
3. `SpeedController` đọc sensor + PID → drive motor @1kHz

---

## 2. SpeedController

### Biến Trạng Thái (volatile, đọc được từ mọi task)

| Biến | Đơn vị | Mô tả |
|------|--------|-------|
| `ref_x` | mm | Vị trí mục tiêu (tích lũy) |
| `ref_v` | mm/s | Vận tốc mục tiêu |
| `ref_th` | degree | Góc mục tiêu |
| `ref_w` | deg/s | Vận tốc góc mục tiêu |
| `est_x` | mm | Vị trí ước lượng (encoder) |
| `est_v` | mm/s | Vận tốc ước lượng |
| `est_th` | degree | Góc ước lượng (gyro) |
| `est_w` | deg/s | Vận tốc góc ước lượng |
| `est_global_x` | mm | Tọa độ X toàn cục |
| `est_global_y` | mm | Tọa độ Y toàn cục |
| `battery_voltage` | V | Điện áp pin đo thực |

### Hàm API

#### `speedCtrl.init()`
- **Khi nào gọi:** 1 lần duy nhất trong `setup()`, sau khi đã init encoder và IMU
- **Tác dụng:** Tạo FreeRTOS task chạy PID loop @1kHz

#### `speedCtrl.set_target_tra(float x, float v)`
- **Thread-safe:** ✅ (có critical section)
- **Đơn vị:** x = mm (vị trí tuyệt đối), v = mm/s
- **Ví dụ:**
```cpp
// Đặt mục tiêu: đi đến vị trí 300mm với vận tốc 330mm/s
speedCtrl.set_target_tra(300.0f, 330.0f);

// Dừng tại vị trí hiện tại
speedCtrl.set_target_tra(speedCtrl.ref_x, 0);
```

#### `speedCtrl.set_target_rot(float th, float w)`
- **Thread-safe:** ✅ (có critical section)
- **Đơn vị:** th = degree (góc tuyệt đối), w = deg/s
- **Ví dụ:**
```cpp
// Xoay đến góc 90°, vận tốc 180°/s
speedCtrl.set_target_rot(90.0f, 180.0f);

// Giữ góc hiện tại, dừng xoay
speedCtrl.set_target_rot(speedCtrl.ref_th, 0);
```

#### `speedCtrl.reset()`
- **Tác dụng:** Reset toàn bộ trạng thái (ref + est) về 0
- **Khi nào dùng:** Trước khi bắt đầu một chuyến chạy mới
- **Lưu ý:** KHÔNG reset góc IMU (gyro vẫn tích lũy). Gọi `reset_fused_data()` nếu cần reset góc.

---

## 3. MoveAction

### Hàm API

#### `moveAction.init()`
- **Khi nào gọi:** 1 lần duy nhất trong `setup()`
- **Tác dụng:** Tạo command queue (50 slots) và FreeRTOS task

#### `moveAction.push_command(char cmd)`
- **Thread-safe:** ✅ (FreeRTOS Queue)
- **Ký tự hợp lệ:**

| Ký tự | Hành động |
|-------|-----------|
| `S` hoặc `F` | Đi thẳng 1 ô (300mm) |
| `R` | Rẽ phải 90° (nửa ô → xoay → nửa ô) |
| `L` | Rẽ trái 90° (nửa ô → xoay → nửa ô) |
| `B` | Quay 180° (nửa ô → xoay → nửa ô) |

- **Ví dụ:**
```cpp
// Đi thẳng 1 ô, rẽ phải, đi thẳng 1 ô
moveAction.push_command('S');
moveAction.push_command('R');
moveAction.push_command('S');
```

- **Lưu ý:** Lệnh được xếp hàng (queue) và thực thi tuần tự. Có delay 100ms giữa các lệnh.

#### `moveAction.start_fast_run(const char* search_path)`
- **Tác dụng:** Chuyển đổi chuỗi search → fast path, rồi thực thi
- **Input format:** Chuỗi Kerise (đã bao gồm `s` đầu/cuối)
- **Ví dụ:**
```cpp
// Chuỗi search: đi thẳng, rẽ trái, đi thẳng, rẽ phải, đi thẳng
moveAction.start_fast_run("sSLSRSs");
```

### Bảng Ký Tự Fast Path (sau convert)

| Ký tự | Hành động | Quãng đường |
|-------|-----------|-------------|
| `S` | ST_FULL — Thẳng 1 ô | 300mm |
| `h` | ST_HALF — Thẳng nửa ô | 150mm |
| `w` | ST_DIAG — Thẳng chéo | 150√2 mm |
| `L` | FS90_L — Cua mềm trái 90° | Slalom |
| `R` | FS90_R — Cua mềm phải 90° | Slalom |
| `q` | F90_L — Cua chéo trái 90° | Slalom |
| `Q` | F90_R — Cua chéo phải 90° | Slalom |
| `z` | F45_L — Cua chéo trái 45° | Slalom |
| `c` | F45_R — Cua chéo phải 45° | Slalom |
| `Z` | F45_LP — Cua chéo trái 45° (post) | Slalom |
| `C` | F45_RP — Cua chéo phải 45° (post) | Slalom |
| `a` | F135_L — Cua chéo trái 135° | Slalom |
| `d` | F135_R — Cua chéo phải 135° | Slalom |
| `A` | F135_LP — Cua chéo trái 135° (post) | Slalom |
| `D` | F135_RP — Cua chéo phải 135° (post) | Slalom |
| `u` | F180_L — U-turn trái | Slalom |
| `U` | F180_R — U-turn phải | Slalom |
| `p` | FV90_L — V-turn trái 90° | Slalom |
| `P` | FV90_R — V-turn phải 90° | Slalom |

---

## 4. Commander

### Giao tiếp qua Bluetooth Serial (115200 baud)

#### Gửi lệnh di chuyển (từng bước)
```
SRL\n        → Đi thẳng, Rẽ phải, Rẽ trái (queue từng lệnh)
```

#### Gửi Fast Run
```
XsSLSRSs\n  → Chữ X đầu + chuỗi Kerise search path
```

#### Gửi bản đồ + tự tìm đường
```
MAP:00770F0B...\n   → 4 hex coords + 256 hex wall values
```
**Format:** `MAP:` + startX(1 hex) + startY(1 hex) + goalX(1 hex) + goalY(1 hex) + 256 ký tự wall (mỗi cell = 1 hex, E=1, N=2, W=4, S=8)

### Teleplot Output (đọc tự động @100Hz)

| Kênh | Ý nghĩa |
|------|---------|
| `>RefX` | Vị trí mục tiêu (mm) |
| `>EstX` | Vị trí ước lượng (mm) |
| `>EstTh` | Góc ước lượng (degree) |
| `>RefV` | Vận tốc mục tiêu (mm/s) |
| `>EstV` | Vận tốc ước lượng (mm/s) |
| `>EstXY:x:y\|xy` | Tọa độ XY 2D (mm) |
| `>VBat` | Điện áp pin (V) |
| `>EncLeft` | Encoder trái (counts) |
| `>EncRight` | Encoder phải (counts) |

---

## 5. IMU (BMI160)

### Hàm API

#### `setup_mpu_manual()`
- Khởi tạo BMI160 qua SPI2 + **runtime calibration**
- Robot **PHẢI ĐỨNG YÊN** khi gọi hàm này!
- Gọi 1 lần trong `setup()`

#### `calibrate_imu()`
- Chạy calibration riêng (500 samples, ~0.6s)
- Tự động được gọi trong `setup_mpu_manual()`
- Có thể gọi lại bất cứ lúc nào (robot phải đứng yên)

#### `read_mpu_data(float dt)`
- Đọc data + xử lý (offset, integration, compensation)
- `dt` = thời gian giữa 2 lần gọi (thường = 0.001f)
- Được gọi tự động bởi SpeedController @1kHz

#### Getter Functions

| Hàm | Đơn vị | Mô tả |
|-----|--------|-------|
| `get_yaw_angle()` | degree | Góc yaw tích lũy |
| `get_yaw_rate()` | deg/s | Vận tốc góc yaw (đã bù offset) |
| `get_linear_accel()` | mm/s² | Gia tốc tiến (đã bù gravity + centripetal) |
| `get_sideways_accel()` | mm/s² | Gia tốc ngang |
| `get_angular_accel()` | deg/s² | Gia tốc góc (finite difference) |

#### `reset_fused_data()`
- Reset tất cả trạng thái IMU (góc, gia tốc) về 0
- Gọi trước khi bắt đầu chuyến chạy mới

---

## 6. Encoder

### API

```cpp
MotorEncoder enc(pinA, pinB);
enc.begin();                    // Khởi tạo GPIO + đọc trạng thái đầu
enc.update();                   // Gọi trong ISR (cả CHANGE trên cả 2 kênh)
long count = enc.getCount();    // Thread-safe (disable interrupts)
enc.reset();                    // Reset count về 0
```

### Cấu hình ISR (trong main.cpp)
```cpp
attachInterrupt(digitalPinToInterrupt(ENCAL), isrLeft, CHANGE);
attachInterrupt(digitalPinToInterrupt(ENCBL), isrLeft, CHANGE);
```
**Quan trọng:** Attach **CẢ 2 kênh** (A và B) trên **CHANGE** để có độ phân giải 4X.

### Đơn vị chuyển đổi
```
MM_PER_COUNT = π × 40mm / (28 × 10) ≈ 0.449 mm/count
```

---

## 7. Config — Hằng Số Quan Trọng

### Cần Tune Khi Đổi Phần Cứng

| Hằng số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| `WHEEL_DIAMETER` | 40mm | Đường kính bánh xe |
| `ENCODER_PULSES` | 28 | PPR gốc của encoder |
| `GEAR_RATIO` | 10 | Tỷ số truyền |
| `MOUSE_RADIUS` | 38.76mm | Nửa khoảng cách 2 bánh |
| `FULL_CELL` | 300mm | Kích thước 1 ô mê cung |

### Cần Tune Khi Đổi Motor

| Hằng số | Ý nghĩa |
|---------|---------|
| `FWD_KM` | Motor gain (mm/s/V) — đo bằng system ID |
| `FWD_TM` | Motor time constant (s) — đo bằng system ID |
| `ROT_KM` | Rotational motor gain (deg/s/V) |
| `ROT_TM` | Rotational time constant (s) |

### Tốc Độ Chạy

| Hằng số | Giá trị | Khi nào dùng |
|---------|---------|--------------|
| `METRIC_SEARCH_SPEED` | 0.33 m/s | Search run (thận trọng) |
| `METRIC_FAST_SPEED` | 0.72 m/s | Fast run max |
| `METRIC_ACCEL` | 3.6 m/s² | Gia tốc |
| `METRIC_JERK` | 240 m/s³ | Jerk (độ mượt) |

---

## 8. Lưu Ý Quan Trọng

### ⚠️ Đơn Vị
- **Translation:** tất cả đều dùng **mm** và **mm/s**
- **Rotation:** tất cả đều dùng **degree** và **deg/s**
- **Ngoại lệ:** Slalom shapes (`ctrl::slalom`) dùng **radian** nội bộ. Khi set ref_rot từ slalom output, phải nhân `180/PI`

### ⚠️ Thread Safety
- `set_target_tra()` và `set_target_rot()` đã có **critical section** — an toàn gọi từ bất kỳ task nào
- Các biến `est_*` và `ref_*` là `volatile float` — an toàn ĐỌC từ bất kỳ task nào (ARM 32-bit atomic)
- **KHÔNG** gọi `reset()` từ task khác khi SpeedController đang chạy PID

### ⚠️ Khởi Động
1. Robot **PHẢI ĐỨNG YÊN** trong ~1 giây đầu tiên (IMU calibration)
2. Nếu gyro drift nhiều, gọi `calibrate_imu()` lại
3. Luôn gọi `reset_fused_data()` trước mỗi chuyến chạy mới

### ⚠️ Battery
- Điện áp pin được đo tự động qua chân PA7 mỗi 1 giây
- PWM output tự động bù theo pin thực — không cần lo pin yếu
- Nếu ADC đọc < 3V → fallback về 8V (tránh chia cho 0)

### ⚠️ Giới Hạn Hiện Tại
1. **Không có TrajectoryTracker 2D** — robot chỉ bám 1 chiều (tiến/lùi), không sửa được lệch ngang
2. **Không có cảm biến vách** — không thể tự khám phá mê cung, chỉ chạy theo bản đồ cho sẵn
3. **Slalom shapes** được scale tuyến tính từ Kerise 90mm — chưa tối ưu cho ô 300mm, cần tune lại nếu sai lệch quỹ đạo

### ⚠️ Tần Số PWM
- Đã set 20kHz (siêu âm) bằng `analogWriteFrequency(20000)` trong `setup()`
- Giảm tiếng rít động cơ. Nếu cần thay đổi, sửa trong `main.cpp`
