# MicroMouse STM32 FreeRTOS

Dự án firmware điều khiển robot giải mê cung (**MicroMouse**) tốc độ cao, xây dựng trên vi điều khiển **STM32F411CEU6**, sử dụng **FreeRTOS** và kiến trúc điều khiển chuyển động tiên tiến (quỹ đạo mượt mà, rẽ Slalom, thuật toán tìm đường tối ưu).

---

## Tính Năng Nổi Bật

- **Đa nhiệm thời gian thực (FreeRTOS):**
  - **Vòng lặp điều khiển 1 kHz (Prio 4):** Đọc cảm biến, ước lượng trạng thái (Odometry + Gyro fusion), bộ điều khiển vị trí/vận tốc góc PID + Feedforward.
  - **MoveAction Task:** Sinh quỹ đạo chuyển động mượt mà (AccelDesigner) cho các thao tác đi thẳng, quay tại chỗ (Spin Turn) và ôm cua mượt không dừng (Slalom Turn).
  - **Drive Task (Prio 2):** Điều phối trạng thái máy (Machine State Machine), khám phá mê cung và chạy tối ưu.
  - **Telemetry Task (100 Hz, Prio 1):** Xuất dữ liệu thời gian thực ra công cụ **Teleplot** qua Bluetooth.
- **Thuật toán mê cung (MazeLib):**
  - Khám phá mê cung với thuật toán **Flood-Fill / Adachi**.
  - Tối ưu đường chạy tốc độ cao (**Fast Run**) với các đoạn cua Slalom 90°, 180° và đường chéo (Diagonal).
- **Giao diện & Tiện ích:**
  - Chọn chế độ linh hoạt bằng 1 nút bấm (Button) và đèn LED chỉ thị.
  - Tích hợp công cụ trực quan [maze_designer.html](maze_designer.html) thiết kế mê cung trên trình duyệt và nạp bản đồ trực tiếp qua Bluetooth UART.
  - Tích hợp System Identification (SysID) để xác định tham số mô hình động cơ.

---

## Cấu Hình Phần Cứng

| Thành phần | Chi tiết | Giao tiếp / Pin STM32F411 |
| :--- | :--- | :--- |
| **MCU** | STM32F411CEU6 ("BlackPill", ARM Cortex-M4F @ 100MHz) | — |
| **Động cơ & Driver** | Động cơ DC Coreless + Driver DRV8833 | `PB6`, `PB7`, `PB8`, `PB9` (Hardware TIM4 PWM) |
| **Encoder** | 2x MT6701 Magnetic Encoder (Chế độ ABZ 4X) | Left: `PB4`, `PB5` (TIM3) / Right: `PA15`, `PB3` (TIM2) |
| **IMU** | BMI160 (Gyroscope + Accelerometer 6-DOF) | `PB12` (CS), `PB13` (SCK), `PB14` (MISO), `PB15` (MOSI) via SPI2 |
| **Cảm biến tường** | Cảm biến hồng ngoại / ToF phản xạ thành | Cấu hình trong `hardware/` |
| **Giao tiếp Bluetooth** | Module UART không dây (115200 baud) | `PA9` (TX), `PA10` (RX) via USART1 |
| **Đo điện áp pin** | Đo áp Pin LiPo qua cầu phân áp | `PB0` (ADC1 Channel 8 via DMA) |
| **UI** | 1 Phím bấm đa năng & LED báo trạng thái | Nút: `PB1`, LED: `PC13`, Còi: `PB10` |

---

## Cấu Trúc Thư Mục

```text
├── src/
│   ├── main.cpp              # Khởi tạo phần cứng, FreeRTOS Scheduler
│   ├── machine/              # Machine Coordinator & State Machine
│   ├── agents/               # Điều khiển chuyển động (MoveAction, MazeRobot)
│   ├── supporters/           # Bộ điều khiển tốc độ (SpeedController), UI, Logger
│   ├── hardware/             # Trừu tượng hóa phần cứng (Motor, Encoder, IMU, Pin...)
│   └── config/               # Pin IO mapping, hằng số vật lý, Slalom shape
├── lib/
│   ├── ctrl/                 # Thư viện sinh quỹ đạo (Trajectory/AccelDesigner, PID)
│   ├── maze/                 # Thư viện giải mê cung (MazeLib, Wall/Cell Representation)
│   └── utils/                # C++ FreeRTOS wrapper & các tiện ích toán học
├── docs/                     # Tài liệu hướng dẫn API và phân tích kiến trúc
├── tools/                    # Tool nhận diện hệ thống (SysID)
├── maze_designer.html        # Công cụ web vẽ bản đồ và sinh chuỗi hex nạp vào robot
└── platformio.ini            # Cấu hình nạp mã nguồn, thư viện & cờ biên dịch
```

---

## Danh Sách Chế Độ Hoạt Động (Operating Modes)

Khi khởi động, robot ở trạng thái chờ chọn chế độ (nhấn giữ nút để tăng mode, nhả để xác nhận):

- **Mode 0:** `Search Run` — Chạy khám phá và lưu bản đồ mê cung.
- **Mode 1:** `Fast Run` — Chạy đường đua ngắn nhất với tốc độ cao.
- **Mode 2:** `Motor Test` — Kiểm tra chiều quay và đáp ứng động cơ.
- **Mode 3:** `Encoder Test` — Kiểm tra số xung đọc được từ 2 encoder MT6701.
- **Mode 4:** `IMU Test` — Kiểm tra dữ liệu góc quay và hiệu chuẩn Gyro.
- **Mode 5:** `Slalom Test` — Chạy thử nghiệm quỹ đạo cua Slalom.
- **Mode 6:** `Spin Turn Test` — Chạy thử nghiệm quay tại chỗ 90° / 180°.
- **Mode 7:** `Receive Web Map` — Nhận bản đồ mê cung dạng hex string truyền qua Bluetooth.
- **Mode 8:** `SysID` — Thu thập dữ liệu đáp ứng bước phục vụ tinh chỉnh bộ điều khiển.

---

## Hướng Dẫn Biên Dịch & Nạp Code

### Yêu Cầu
- [VS Code](https://code.visualstudio.com/) + Tiện ích mở rộng [PlatformIO IDE](https://platformio.org/).
- Mạch nạp **ST-Link V2** (kết nối chân SWD: SWDIO `PA13`, SWCLK `PA14`, GND, 3.3V).

### Các Lệnh Thường Dùng
```powershell
# Biên dịch dự án
pio run

# Nạp firmware vào vi điều khiển qua ST-Link
pio run -t upload

# Mở cổng Serial Monitor giám sát (115200 baud)
pio device monitor

# Cập nhật compilation database cho Clangd
pio run -t compiledb
```
