# 🔬 Phân Tích So Sánh: Project Hiện Tại vs. Kerise v4

## Tổng Quan

| Tiêu chí | Project hiện tại (STM32) | Kerise v4 (ESP32) |
|---|---|---|
| MCU | STM32F411 | ESP32 |
| RTOS | FreeRTOS (Arduino) | FreeRTOS (ESP-IDF) |
| IMU | BMI160 (SPI) | ICM20602 (SPI, x2) |
| Encoder | Software Quadrature (GPIO ISR) | AS5048A / MA730 (SPI, absolute) |
| Motor Driver | TB6612 (library) | MCPWM (native ESP32) |
| Cell Size | **300mm** | **90mm** (half-size) |
| Cảm biến vách | ❌ Không có | ✅ ToF + Reflector + Wall Detector |

---

## 1. Speed Controller — Vấn đề nghiêm trọng nhất

### 1.1 Kiến trúc PID hoàn toàn khác Kerise

> [!CAUTION]
> Project hiện tại dùng **PD controller thủ công** (vị trí + vận tốc), trong khi Kerise dùng **`FeedbackController` với mô hình vật lý** (Kp, Ki, Kd cho cả translation lẫn rotation) cùng **Complementary Filter** cho ước lượng vận tốc. Hai kiến trúc này khác nhau hoàn toàn.

**Kerise v4** (`speed_controller.h`):
```cpp
// Dùng FeedbackController chuyên biệt với model vật lý
ctrl::FeedbackController<ctrl::Polar> fbc;

// Velocity estimation = complementary filter giữa encoder (low-freq) và IMU (high-freq)
const ctrl::Polar v_low = ctrl::Polar(enc_v.tra, hw->imu->get_gyro());
const ctrl::Polar v_high = est_v + accel[0] * float(Ts);
est_v = alpha * v_low + (1 - alpha) * v_high;

// Drive output từ FeedbackController
const auto pwm_value = fbc.update(ref_v, est_v, ref_a, est_a, Ts);
```

**Project hiện tại** (`speed_controller.cpp`):
```cpp
// PD thủ công, KHÔNG CÓ TÍCH PHÂN (Ki)
float u_tra = FWD_KP * err_x + true_FWD_KD * err_v + ff_tra;
float u_rot = ROT_KP * err_th + true_ROT_KD * err_w + ff_rot;

// Ép PWM giá trị cứng thay vì dùng điện áp pin thực
const float V_BAT = 8.0f;
int pwm_L = constrain((vL / V_BAT) * 255.0f, -255.0f, 255.0f);
```

**Vấn đề cụ thể:**

| # | Vấn đề | Mức độ |
|---|---|---|
| 1 | **Thiếu thành phần Ki** (integral) cho translation → sai số tĩnh tích lũy | 🔴 Nghiêm trọng |
| 2 | `V_BAT = 8.0f` hardcoded — Kerise đo pin thực → PWM sai khi pin yếu | 🟡 Trung bình |
| 3 | **Không có thread safety** — `ref_x`, `ref_v` được ghi từ task Action (priority 3) nhưng đọc từ task ControlLoop (priority 4) mà KHÔNG CÓ mutex | 🔴 Nghiêm trọng |
| 4 | Low-pass filter alpha = 0.2 cho cả `est_v` lẫn `est_w` — Kerise dùng `alpha_rot = 1.0` (100% gyro, không filter) | 🟡 Trung bình |

### 1.2 Đơn vị (Units) — Khác biệt lớn

> [!WARNING]
> Kerise hoạt động hoàn toàn trong hệ **SI (mm, mm/s, rad, rad/s)**. Project hiện tại **trộn lẫn degree và radian** khắp nơi.

**Kerise:**
- `est_p.th`: **radian**
- `gyro.z`: **rad/s**
- `set_target(v_tra, v_rot)`: mm/s, **rad/s**

**Project hiện tại:**
- `est_th`: **degree** (từ `get_yaw_angle()`)
- `est_w`: **deg/s** (từ `get_yaw_rate()`)
- `set_target_rot(th, w)`: **degree, deg/s**
- `ROT_KP * err_th` → nhân degree × gain, kết quả = **Volt** → Không chuẩn hóa

Điều này có nghĩa tất cả hệ số PID cho rotation (`ROT_KP`, `ROT_KD`) cần phải được tính lại hoàn toàn nếu muốn chuyển sang radian. Hiện tại code **hoạt động được** nhưng rất khó debug và tune vì các gain phụ thuộc vào đơn vị hỗn hợp.

### 1.3 Odometry: Cách tính khác nhau

**Kerise:**
```cpp
// Odometry dùng encoder velocity + gyro angle (chính xác hơn)
est_p.th += hw->imu->get_gyro() * Ts;                    // radian
est_p.x += enc_v.tra * std::cos(est_p.th + slip) * Ts;   // mm
est_p.y += enc_v.tra * std::sin(est_p.th + slip) * Ts;   // mm
```

**Project hiện tại:**
```cpp
// Tích lũy từ encoder delta (đúng logic)
est_x += dTra;
est_th = get_yaw_angle();  // gyro integration, degree

// Tọa độ global — chuyển degree→radian tại đây
est_global_x += dTra * cos(est_th * PI / 180.0f);
est_global_y += dTra * sin(est_th * PI / 180.0f);
```

→ **Logic cơ bản đúng**, nhưng `est_x` chỉ là khoảng cách tổng (1D), không phải vị trí theo phương tiến (x trong hệ local). Kerise dùng `est_p` là `Pose(x, y, th)` — 2D position tracking. Điều này quan trọng cho trajectory tracking.

---

## 2. Move Action — Search Run

### 2.1 Logic Search Run cơ bản

**Project hiện tại** — dùng **AccelDesigner trực tiếp** để tạo profile:
```cpp
if (cmd == 'S') {
    ctrl::AccelDesigner ad(METRIC_JERK*1000, METRIC_ACCEL*1000, 
                           METRIC_SEARCH_SPEED*1000, 0, 0, 300.0f);
    for (float t = 0; t < ad.t_end(); t += Ts) {
        speedCtrl.set_target_tra(start_x + ad.x(t), ad.v(t));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

**Kerise** — dùng **TrajectoryTracker** (trajectory tracking controller):
```cpp
ctrl::TrajectoryTracker tt{tt_gain};
ctrl::straight::Trajectory trajectory;
trajectory.reset(j_max, a_max, v_max, v_start, v_end, distance);
tt.reset(v_start);
for (float t = 0; true; t += Ts) {
    trajectory.update(ref_s, t);
    const auto ref = tt.update(est_p, est_v, est_a, ref_s);
    sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
}
```

> [!IMPORTANT]
> **Kerise dùng TrajectoryTracker** — một bộ điều khiển 2D bám quỹ đạo (giống như pure pursuit + PID), có khả năng sửa lỗi cross-track (y) và heading (θ) trong khi chạy. Project hiện tại **chỉ bám theo 1 chiều x** → không thể sửa lỗi lệch ngang.

### 2.2 Xoay tại chỗ (Spin Turn) — Vấn đề logic

**Project hiện tại:**
```cpp
// Lệnh xoay: Dừng → Xoay → Dừng → Tiến tiếp
// AccelDesigner cho rotation dùng hệ số: METRIC_JERK * 50, METRIC_ACCEL * 50
ctrl::AccelDesigner ad_rot(METRIC_JERK * 50.0f, METRIC_ACCEL * 50.0f, 
                           METRIC_SEARCH_SPEED * 50.0f, 0, 0, angle);
```

> [!WARNING]
> `METRIC_JERK * 50 = 240 * 50 = 12000 deg/s³` — giá trị này được truyền vào AccelDesigner vốn hoạt động trong hệ radian. Vì `angle = ±90.0f` (degree), AccelDesigner sẽ hiểu đây là **90 radian** (≈ 5156 degree!). Đây là **BUG** rất nghiêm trọng.

**Kerise** dùng `turn()` với radian:
```cpp
turn(PI / 2);    // 90° Left
turn(-PI / 2);   // 90° Right
turn(PI);        // 180°
```

### 2.3 Search Turn Flow

| Bước | Project hiện tại | Kerise |
|---|---|---|
| 1 | Tiến nửa ô (150mm) | `straight_x(SegWidthFull/2, ...)` |
| 2 | Dừng 100ms | Tùy trường hợp: dùng slalom hoặc dừng + front_wall_attach |
| 3 | Spin turn tại chỗ | `turn(±PI/2)` |
| 4 | Dừng 100ms | — |
| 5 | Tiến nửa ô (150mm) | `straight_x(SegWidthFull/2, ...)` |

→ Logic tổng thể **giống nhau** cho trường hợp spin turn. Nhưng Kerise có thêm logic **slalom turn** (cua mềm không dừng) cho Search Run khi điều kiện thuận lợi, giúp tiết kiệm thời gian.

---

## 3. Fast Run — Phân tích chi tiết

### 3.1 Chuyển đổi chuỗi Search → Fast

**Project hiện tại** — tự viết lại logic `convert_search_to_fast()`:
```cpp
replace(src, "S", "ss");       // Expand
replace(src, "L", "ll");
replace(src, "R", "rr");
// ... Diagonal compression patterns ...
replace(src, "ss", "S");       // Compact
replace(src, "s", "h");        // Half
```

**Kerise** — dùng `MazeLib::RobotBase::pathConvertSearchToFast()` (thư viện chuẩn):
```cpp
const auto path = MazeLib::RobotBase::pathConvertSearchToFast(
    search_actions, rp.diag_enabled);
```

→ Logic chuyển đổi của project hiện tại **giống y hệt** MazeLib (các pattern replace giống nhau). ✅ **Đúng logic**.

### 3.2 Slalom Shapes — Sai lệch nghiêm trọng khi scale

> [!CAUTION]
> Đây là vấn đề **nghiêm trọng nhất** trong toàn bộ project.

**Kerise (ô 90mm)** — Slalom shapes được tính toán chính xác bằng optimization:
```cpp
// F90: Pose(90, 90, PI/2), curve_end=(70, 70), straight_prev=20, straight_post=20
ctrl::slalom::Shape(Pose(90, 90, 1.5708), Pose(70, 70, 1.5708), 20, 20, ...);
```

**Project hiện tại (ô 300mm)** — Scale thủ công bằng `K = 300/90`:
```cpp
float K = 300.0f / 90.0f;
// F90: Pose(90*K, 90*K, PI/2) = Pose(300, 300, PI/2)
ctrl::slalom::Shape shapeF90(ctrl::Pose(90.0f*K, 90.0f*K, PI/2), 70.0f*K, 0, ...);
```

**Vấn đề:**

| # | Vấn đề | Chi tiết |
|---|---|---|
| 1 | **`y_curve_end` bị bỏ qua** | Kerise Shape constructor nhận cả `Pose curve` VÀ `Pose curve_end`. Project hiện tại chỉ truyền `curve` và 1 giá trị float duy nhất cho `y_curve_end`, nhưng **constructor của Shape cần nhiều tham số hơn** |
| 2 | **`straight_prev` và `straight_post` = 0 ở hầu hết shapes** | Kerise có `straight_prev=20, straight_post=20` cho F90 → đoạn thẳng trước/sau cua giúp robot ổn định. Project hiện tại set = 0 |
| 3 | **Các hệ số jw_max, aw_max, w_max không tương thích** | Kerise: `jw=3769.91, aw=113.097, w=9.42478` (đơn vị rad). Project: `jw=400, aw=40, w=10` — **thấp hơn ~10 lần** |
| 4 | **F135P, F45P dùng tọa độ hardcoded** | `Pose(106.066, 318.198, ...)` và `Pose(318.198, 106.066, ...)` — đây là giá trị tính tay, dễ sai nếu thay đổi cell size |

### 3.3 Thực thi Slalom — Logic `run_slalom` vs Kerise `trace`

**Project hiện tại:**
```cpp
auto run_slalom = [&](Shape &shape, bool is_right) {
    traj.reset(v_run);
    // Chạy straight_prev
    run_straight(straight_acc, v_run);
    // Chạy curve — CHỈ SET ref_tra theo v*t (1D) + ref_rot từ trajectory
    for (float t = 0; t < traj.getTimeCurve(); t += Ts) {
        traj.update(state, t, Ts);
        speedCtrl.set_target_tra(current_x + v_run * t, v_run);
        speedCtrl.set_target_rot(start_th + state.q.th * 180/PI, state.dq.th * 180/PI);
    }
    current_x += v_run * traj.getTimeCurve();
};
```

**Kerise:**
```cpp
void trace(slalom::Trajectory& traj, const RunParameter& rp) {
    TrajectoryTracker tt(tt_gain);
    tt.reset(velocity);
    traj.reset(velocity);
    for (float t = 0; t < traj.getTimeCurve(); t += Ts) {
        traj.update(s, t, Ts);
        // TrajectoryTracker xử lý TOÀN BỘ: position, velocity, acceleration
        const auto ref = tt.update(est_p, est_v, est_a, s);
        sc->set_target(ref.v, ref.w, ref.dv, ref.dw);
    }
    // CẬP NHẬT VỊ TRÍ sau slalom: est_p = (est_p - net).rotate(-net.th)
    sc->update_pose((sc->est_p - net).rotate(-net.th));
    offset += net.rotate(offset.th);
}
```

> [!CAUTION]
> **Sai lệch quan trọng #1:** Project hiện tại xấp xỉ forward position trong slalom bằng `v_run * t` (tuyến tính). Thực tế trong cua, quãng đường theo x cục bộ **KHÔNG** bằng `v * t` vì robot đang quay — nó phụ thuộc `cos(θ)`. Sai số tích lũy qua nhiều cua.
>
> **Sai lệch quan trọng #2:** Project hiện tại **KHÔNG cập nhật hệ tọa độ local** sau mỗi slalom. Kerise thực hiện `est_p = (est_p - net).rotate(-net.th)` để "reset" hệ tọa độ local, đảm bảo `est_p.x` luôn là khoảng cách theo phương tiến. Project hiện tại chỉ cộng `current_x += v_run * t` → sai số cộng dồn ngày càng lớn.
>
> **Sai lệch quan trọng #3:** Project hiện tại KHÔNG truyền `ref_a` (acceleration feedforward) cho speed controller → phản hồi chậm hơn, đặc biệt khi thay đổi tốc độ đột ngột.

---

## 4. IMU (BMI160 vs ICM20602)

### 4.1 Calibration

**Kerise** — Runtime calibration (đo 200 sample, lấy trung bình):
```cpp
void task_calibration(TickType_t& xLastWakeTime) {
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < ave_count; i++) {
            update(); 
            gyro_sum += gyro;
        }
        gyro_offset += gyro_sum / ave_count;
    }
}
```

**Project hiện tại** — **Hardcoded offset**:
```cpp
gyro_z_offset = 0.25f;
accel_z_offset = -134.42f;
```

> [!WARNING]
> Gyro bias drift theo nhiệt độ (~0.015 deg/s/°C cho BMI160). Hardcoded offset sẽ **sai lệch** khi nhiệt độ thay đổi. Nên calibrate mỗi lần khởi động như Kerise.

### 4.2 Angular Acceleration

**Kerise (v4, 2 IMU):**
```cpp
// Dùng accelerometer trên cánh tay đòn 10mm
angular_accel = (icm[0].accel.y + icm[1].accel.y) / 2 / IMU_ROTATION_RADIUS;
```

**Project hiện tại:**
```cpp
// Dùng finite difference (ồn hơn)
s_current_angular_accel = (currentRate - s_prev_yaw_rate) / dt;
```

→ Project hiện tại dùng **sai phân bậc 1** — ồn hơn nhiều so với dùng accelerometer. Tuy nhiên, `angular_accel` chỉ dùng cho complementary filter ở Kerise, project hiện tại không dùng nên **chưa gây vấn đề thực tế**.

### 4.3 Centripetal Compensation — ✅ Đúng

```cpp
// Project hiện tại — bù gia tốc hướng tâm cho IMU offset 10mm
float centripetal_compensation = 10.0f * (omega * omega);
```
→ Logic đúng theo công thức vật lý `a_center = r * ω²`.

---

## 5. Encoder — Khác biệt cơ bản

| Tiêu chí | Project hiện tại | Kerise |
|---|---|---|
| Loại | Quadrature AB (GPIO ISR) | AS5048A absolute (SPI) |
| Độ phân giải | 7 PPR × 4 × 10 = **280 counts/vòng** | **16384 counts/vòng** |
| Precision | ~0.448 mm/count | ~0.0024 mm/count |
| Sampling | GPIO interrupt → ISR update | SPI polling @1kHz trong task |
| Thread safety | `noInterrupts()` guard ✅ | `std::mutex` ✅ |

> [!NOTE]
> Encoder hiện tại có **độ phân giải thấp hơn ~15 lần** so với Kerise. Ở tốc độ 800mm/s, mỗi tick 1ms chỉ đọc được ~7 counts. Đủ dùng cho tốc độ thấp nhưng sẽ gây **nhiễu vận tốc đáng kể** ở tốc độ cao.

---

## 6. Motor Driver

**Kerise** — dùng trực tiếp duty cycle [-1.0, 1.0]:
```cpp
hw->mt->drive(pwm_value_L, pwm_value_R);  // float [-1, 1]
```

**Project hiện tại** — TB6612 library với PWM [0, 255]:
```cpp
int pwm_L = constrain((vL / V_BAT) * 255.0f, -255.0f, 255.0f);
motorLeft.drive(pwm_L);
```

→ Logic tương đương. Tuy nhiên, **V_BAT hardcoded** ở 8.0V là vấn đề. Nên đo pin thực bằng ADC qua VBAT (PA7) đã được config.

---

## 7. Code bị vô hiệu hóa trong Speed Controller

> [!CAUTION]
> **BUG NGHIÊM TRỌNG:** Code đang ở trạng thái test!

```cpp
// Lines 115-119 trong speed_controller.cpp:
// TEST MT6701: Ép bánh trái quay chậm, vô hiệu hóa bánh phải
pwm_L = 40;
motorLeft.drive(pwm_L);
// motorRight.drive(pwm_R); // BỎ GỌI HÀM NÀY
```

Toàn bộ logic PID đã bị **ghi đè** bởi `pwm_L = 40` → Motor trái chạy cố định ở 15% duty, motor phải không chạy. Robot **KHÔNG THỂ DI CHUYỂN ĐÚNG** với code này.

---

## 8. Tổng Kết Các Vấn Đề Theo Mức Độ

### 🔴 Nghiêm trọng (Phải sửa ngay)

| # | Vấn đề | File |
|---|---|---|
| 1 | **Code test MT6701 ghi đè PID output** | [speed_controller.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/supporters/speed_controller.cpp#L115-L119) |
| 2 | **Spin turn dùng degree thay vì radian** cho AccelDesigner | [move_action.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/agents/move_action.cpp#L61-L75) |
| 3 | **Không cập nhật hệ tọa độ local sau slalom** → sai số cộng dồn | [move_action.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/agents/move_action.cpp#L226-L250) |
| 4 | **Không có thread safety** cho shared variables giữa tasks | [speed_controller.h](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/include/supporters/speed_controller.h) |

### 🟡 Trung bình (Nên sửa)

| # | Vấn đề | File |
|---|---|---|
| 5 | **Thiếu Ki** cho translation PID → steady-state error | [speed_controller.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/supporters/speed_controller.cpp#L69-L77) |
| 6 | **V_BAT hardcoded** thay vì đo ADC | [speed_controller.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/supporters/speed_controller.cpp#L89) |
| 7 | **Gyro offset hardcoded** thay vì runtime calibration | [mpu.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/mpu.cpp#L166-L169) |
| 8 | **Slalom shapes scale đơn giản** — thiếu `straight_prev/post` chính xác | [move_action.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/agents/move_action.cpp#L187-L204) |
| 9 | **Low-pass filter alpha_rot = 0.2** — nên dùng 1.0 (100% gyro) như Kerise | [speed_controller.cpp](file:///c:/Users/Admin/Documents/PlatformIO/Projects/rtosnvchinh/src/supporters/speed_controller.cpp#L63) |

### 🟢 Nhẹ / Cải thiện

| # | Vấn đề | File |
|---|---|---|
| 10 | **Không có TrajectoryTracker** → không sửa được lệch ngang | Toàn bộ move_action.cpp |
| 11 | Thiếu emergency stop / motor overcurrent protection | speed_controller.cpp |
| 12 | Teleplot task chạy ở 100Hz có thể chiếm bandwidth UART | commander.cpp |

---

## 9. Những Phần Đúng Logic ✅

| Phần | Đánh giá |
|---|---|
| **Chuỗi convert_search_to_fast** | ✅ Giống y hệt MazeLib |
| **Encoder quadrature 4X** | ✅ State machine đúng chuẩn |
| **BMI160 SPI init** | ✅ Tuân thủ đúng datasheet (dummy read, soft reset, re-init SPI) |
| **Centripetal compensation** | ✅ Đúng vật lý |
| **FreeRTOS task architecture** | ✅ Phân chia hợp lý (Control @1kHz, Action, Serial, Teleplot) |
| **Maze pathfinding** (Commander + MazeLib) | ✅ Dùng đúng API MazeLib |
| **Failsafe crash detection** | ✅ Logic detect kẹt tường hợp lý |
| **AccelDesigner cho straight** | ✅ Dùng đúng jerk-limited profile |
| **Gravity compensation** cho IMU | ✅ Bù nghiêng bằng pitch angle |

---

## 10. Khuyến Nghị Thứ Tự Sửa

1. **Bỏ code test MT6701** → khôi phục PID output cho cả 2 motor
2. **Sửa spin turn** → chuyển `angle` sang radian trước khi truyền vào AccelDesigner
3. **Thêm cập nhật hệ tọa độ local** sau mỗi slalom (theo pattern Kerise `est_p = (est_p - net).rotate(-net.th)`)
4. **Thêm mutex** cho biến chia sẻ giữa speed controller và move action
5. **Runtime gyro calibration** thay cho hardcoded offset
6. **Đo V_BAT thực** qua ADC chân PA7
7. **Cân nhắc thêm TrajectoryTracker** cho fast run (cải thiện đáng kể độ chính xác)
