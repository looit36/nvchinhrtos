# MicroMouse STM32 FreeRTOS

Dự án firmware điều khiển robot giải mê cung (MicroMouse) tốc độ cao, xây dựng trên vi điều khiển STM32F411CEU6, sử dụng FreeRTOS.

---

## Hướng Dẫn Biên Dịch & Nạp Code

### Yêu Cầu
- [VS Code](https://code.visualstudio.com/) + Tiện ích mở rộng [PlatformIO IDE](https://platformio.org/).
- Mạch nạp **ST-Link V2** (kết nối chân SWD: SWDIO `PA13`, SWCLK `PA14`, GND, 3.3V).

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
