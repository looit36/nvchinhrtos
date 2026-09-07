#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
  STM32 KERISE v4 - SYSTEM IDENTIFICATION & PID TUNING TOOL
  Author: Antigravity Pair-Programming Assistant
  Reference: Ryotaro Onuki <kerikun11+github@gmail.com> (Kerise v4)
=============================================================================
  Mục đích:
    - Thu thập dữ liệu đáp ứng bước (Step Response) qua cổng Serial / Bluetooth
      hoặc đọc từ file log .tab.
    - Nhận dạng hàm truyền bậc 1 của động cơ: P(s) = K1 / (T1 * s + 1)
    - Tính toán chính xác các tham số K1, T1, Feedforward (C1, C2) và Feedback PID
    - Xuất mã nguồn C++ sẵn sàng dán vào src/config/model.h
    - Vẽ đồ thị so sánh giữa dữ liệu thực tế và mô hình lý thuyết.
=============================================================================
"""

import os
import sys
import time
import argparse
import datetime
import numpy as np

# Đảm bảo in UTF-8 không bị lỗi charmap trên console Windows
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="backslashreplace")

# Thử import matplotlib nếu có
try:
    import matplotlib.pyplot as plt
    from matplotlib.ticker import ScalarFormatter
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Thử import pyserial nếu có
try:
    import serial
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

# ==========================================
# THÔNG SỐ VẬT LÝ ROBOT (từ model.h)
# ==========================================
WHEEL_DIAMETER_MM = 26.0
ROTATION_RADIUS_MM = 34
ENCODER_PPR = 1024.0
COUNTS_PER_REV = 4096.0  # 4X decode
GEAR_RATIO = 1.0
SCALE_MM_PER_COUNT = (np.pi * WHEEL_DIAMETER_MM) / (COUNTS_PER_REV * GEAR_RATIO)  # ~0.01994 mm/count
DT = 0.001  # 1kHz sampling (1ms)


def capture_from_serial(port, baudrate, dir_type, duty, duration_ms, save_path):
    """Gửi lệnh SYSID xuống robot qua Serial và ghi log kết quả."""
    if not HAS_PYSERIAL:
        print("[ERROR] Thư viện 'pyserial' chưa được cài đặt!")
        print("Vui lòng chạy lệnh: pip install pyserial")
        sys.exit(1)

    print(f"\n[INFO] Đang kết nối tới cổng {port} (Baudrate: {baudrate})...")
    try:
        ser = serial.Serial(port, baudrate, timeout=5)
    except Exception as e:
        print(f"[ERROR] Không thể mở cổng {port}: {e}")
        sys.exit(1)

    time.sleep(1.0)
    ser.reset_input_buffer()

    # Gửi lệnh SYSID
    cmd = f"SYSID {dir_type} {duty} {duration_ms}\r\n"
    print(f"[TX] Gửi lệnh: {cmd.strip()}")
    ser.write(cmd.encode("utf-8"))

    print("[RX] Đang chờ robot chạy và truyền dữ liệu...")
    lines = []
    recording = False
    start_time = time.time()
    timeout_sec = (duration_ms / 1000.0) + 10.0

    while True:
        if time.time() - start_time > timeout_sec:
            print("[WARN] Hết thời gian chờ dữ liệu từ robot!")
            break

        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        print(f"  {line}")

        if "# SYSID BEGIN" in line:
            recording = True
            lines.append(line + "\n")
            continue

        if "# SYSID END" in line:
            lines.append(line + "\n")
            recording = False
            break

        if recording:
            lines.append(line + "\n")

    ser.close()

    if not lines:
        print("[ERROR] Không nhận được khối dữ liệu '# SYSID BEGIN'!")
        sys.exit(1)

    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
    with open(save_path, "w", encoding="utf-8") as f:
        f.writelines(lines)

    print(f"[SUCCESS] Đã lưu dữ liệu vào: {save_path}")
    return save_path


def load_sysid_data(filepath):
    """Đọc dữ liệu file log .tab hoặc .txt và trích xuất các cột."""
    meta = {"dir": 0, "duty": 0.2, "dt": 0.001, "vbat": 7.4}

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("# SYSID BEGIN"):
                parts = line.strip().split()
                for p in parts[3:]:
                    if "=" in p:
                        k, v = p.split("=", 1)
                        try:
                            meta[k] = float(v) if "." in v else int(v)
                        except ValueError:
                            meta[k] = v
                break

    # Đọc số liệu bằng numpy
    raw = np.loadtxt(filepath, delimiter="\t", comments="#")
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)

    # Cột: enc0, enc1, gyro.z, accel.y, angular_accel, u.tra, u.rot, vbat
    enc0 = raw[:, 0]
    enc1 = raw[:, 1]
    gyro_z = raw[:, 2]      # rad/s
    accel_y = raw[:, 3]     # mm/s^2
    ang_acc = raw[:, 4]     # rad/s^2
    u_tra = raw[:, 5]
    u_rot = raw[:, 6]
    vbat = raw[:, 7] if raw.shape[1] > 7 else np.full(len(raw), meta["vbat"])

    n = len(enc0)
    t = np.arange(n) * DT

    # Kiểm tra đơn vị encoder: nếu dữ liệu có phần thập phân hoặc giá trị thực tế nhỏ -> đã là mm (chuẩn Kerise v4)
    has_fraction = np.any(np.abs(enc0 - np.round(enc0)) > 1e-4)
    scale = 1.0 if (has_fraction or np.max(np.abs(enc0)) < 10000) else SCALE_MM_PER_COUNT

    # Tính vận tốc từng bánh và tịnh tiến/góc
    # Lọc vi sai vận tốc với cửa sổ trượt (window filter)
    d_enc0 = np.diff(enc0, prepend=enc0[0]) * scale
    d_enc1 = np.diff(enc1, prepend=enc1[0]) * scale

    v_L = d_enc0 / DT  # mm/s
    v_R = d_enc1 / DT  # mm/s

    # Làm mượt nhẹ nhiễu quantization của encoder
    kernel_size = 11
    kernel = np.ones(kernel_size) / kernel_size
    v_L_smooth = np.convolve(v_L, kernel, mode="same")
    v_R_smooth = np.convolve(v_R, kernel, mode="same")

    v_tra = (v_L_smooth + v_R_smooth) / 2.0  # mm/s
    w_enc = (v_R_smooth - v_L_smooth) / (2.0 * ROTATION_RADIUS_MM)  # rad/s

    return {
        "meta": meta,
        "n": n,
        "t": t,
        "enc0": enc0,
        "enc1": enc1,
        "v_tra": v_tra,
        "gyro_z": gyro_z,
        "w_enc": w_enc,
        "accel_y": accel_y,
        "ang_acc": ang_acc,
        "u_tra": u_tra,
        "u_rot": u_rot,
        "vbat": vbat,
    }


def fit_1st_order_model(t, y, u):
    """
    Nhận dạng mô hình hàm truyền bậc 1:
      y(t) = K1 * u * (1 - exp(-t / T1))
    """
    n = len(t)
    if n < 50:
        return 1.0, 0.1, y

    # 1. Xác định giá trị xác lập y_ss (lấy trung bình 35% đoạn cuối)
    steady_start = int(n * 0.65)
    steady_end = int(n * 0.95)
    y_ss = np.median(y[steady_start:steady_end])

    if abs(u) < 1e-4:
        u_eff = 0.2
    else:
        u_eff = u

    # K1 = y_ss / u
    K1 = y_ss / u_eff

    # 2. Tìm thời hằng T1: thời điểm y đạt 63.2% của y_ss
    y_63 = 0.632 * y_ss
    idx_63 = np.where(y >= y_63)[0]
    if len(idx_63) > 0:
        T1 = t[idx_63[0]]
    else:
        T1 = 0.2  # Giá trị mặc định an toàn

    # Đảm bảo T1 dương và hợp lý
    if T1 <= 0.01:
        T1 = 0.05
    if T1 > 2.0:
        T1 = 0.5

    # 3. Tinh chỉnh bằng tối ưu hóa phi tuyến (Non-linear Least Squares)
    try:
        def model_func(t_val, k_val, tau_val):
            return k_val * u_eff * (1.0 - np.exp(-t_val / max(tau_val, 1e-3)))

        from scipy.optimize import curve_fit
        popt, _ = curve_fit(model_func, t, y, p0=[K1, T1], bounds=([0, 0.005], [np.inf, 3.0]))
        K1, T1 = popt[0], popt[1]
    except Exception:
        pass  # Nếu không có scipy hoặc fit lỗi, dùng kết quả phân tích ở trên

    # Tính đáp ứng mô hình lý thuyết
    y_sim = K1 * u_eff * (1.0 - np.exp(-t / T1))
    return K1, T1, y_sim


def compute_pid_gains(K1, T1, is_rotational=False):
    """
    Tính toán hệ số Feedforward và Feedback PID theo chuẩn Kerise v4.
    Mô hình: P(s) = K1 / (T1*s + 1)
    """
    # Feedforward
    C1 = 1.0 / K1           # điện áp / vận tốc xác lập (mm/s hoặc rad/s)
    C2 = T1 / K1           # điện áp / gia tốc (mm/s^2 hoặc rad/s^2)

    # Feedback PID Pole-Placement
    # Chu kỳ lấy mẫu Ts = 0.001s (1kHz)
    # Thời gian đáp ứng mong muốn tau_cl (Closed-loop time constant)
    if not is_rotational:
        tau_cl = 0.050  # 50ms cho chuyển động thẳng
        Kp = T1 / (K1 * tau_cl)
        Ki = 1.0 / (K1 * tau_cl)
        Kd = 0.0
    else:
        tau_cl = 0.025  # 25ms cho quay góc (cần nhanh hơn)
        Kp = T1 / (K1 * tau_cl)
        Ki = 1.0 / (K1 * tau_cl)
        Kd = 0.0

    return {
        "C1": C1,
        "C2": C2,
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
    }


def analyze_and_plot(filepath, show_plot=True):
    """Phân tích toàn diện dữ liệu SysID và vẽ đồ thị."""
    data = load_sysid_data(filepath)
    meta = data["meta"]
    t = data["t"]
    dir_type = int(meta.get("dir", 0))
    duty = float(meta.get("duty", 0.2))

    is_rotational = (dir_type == 1)

    print("\n" + "=" * 65)
    print("      STM32 KERISE v4 - KẾT QUẢ SYSTEM IDENTIFICATION")
    print("=" * 65)
    print(f"File log:    {filepath}")
    print(f"Chế độ:      {'ROTATIONAL (QUAY TẠI CHỖ)' if is_rotational else 'TRANSLATIONAL (TỊNH TIẾN THẲNG)'}")
    print(f"Mức xung:    Duty = {duty:.2f} ({duty*100:.0f}%) | Số mẫu: {data['n']} ({data['t'][-1]:.2f}s)")
    print(f"Điện áp pin: Vbat = {np.mean(data['vbat']):.2f} V")
    print("-" * 65)

    if not is_rotational:
        # Tịnh tiến thẳng
        v_exp = data["v_tra"]
        u_val = duty
        K1, T1, v_sim = fit_1st_order_model(t, v_exp, u_val)
        gains = compute_pid_gains(K1, T1, is_rotational=False)

        print(f"[XÁC LẬP] Vận tốc cực đại v_ss:   {K1*u_val:.1f} mm/s")
        print(f"[THAM SỐ] Hệ số khuếch đại K1:    {K1:.3f} (mm/s) / duty")
        print(f"[THAM SỐ] Thời hằng quán tính T1:  {T1:.4f} s ({T1*1000:.1f} ms)")
        print(f"[BỘ BÙ]   Feedforward C1:         {gains['C1']:.6e}")
        print(f"[BỘ BÙ]   Feedforward C2:         {gains['C2']:.6e}")
        print(f"[PID ĐỀ XUẤT] Kp = {gains['Kp']:.5f} | Ki = {gains['Ki']:.5f} | Kd = {gains['Kd']:.5f}")

    else:
        # Quay tại chỗ (dùng Gyro IMU làm chuẩn vận tốc góc)
        w_exp = data["gyro_z"]
        u_val = duty
        K1, T1, w_sim = fit_1st_order_model(t, w_exp, u_val)
        gains = compute_pid_gains(K1, T1, is_rotational=True)

        print(f"[XÁC LẬP] Vận tốc góc cực đại w_ss: {K1*u_val:.2f} rad/s ({K1*u_val*180/np.pi:.1f} deg/s)")
        print(f"[THAM SỐ] Hệ số khuếch đại K1:      {K1:.3f} (rad/s) / duty")
        print(f"[THAM SỐ] Thời hằng quán tính T1:    {T1:.4f} s ({T1*1000:.1f} ms)")
        print(f"[BỘ BÙ]   Feedforward C1:           {gains['C1']:.6e}")
        print(f"[BỘ BÙ]   Feedforward C2:           {gains['C2']:.6e}")
        print(f"[PID ĐỀ XUẤT] Kp = {gains['Kp']:.5f} | Ki = {gains['Ki']:.5f} | Kd = {gains['Kd']:.5f}")

    print("=" * 65)
    print("ĐOẠN CODE CẤU HÌNH C++ DÀNH CHO 'src/config/model.h':\n")
    if not is_rotational:
        print(f"  // Translational Model & Gains (Cập nhật cột đầu Polar):")
        print(f"  .K1 = ctrl::Polar({K1:.3f}f, ...),")
        print(f"  .T1 = ctrl::Polar({T1:.5f}f, ...),")
        print(f"  .Kp = ctrl::Polar({gains['Kp']:.5f}f, ...),")
        print(f"  .Ki = ctrl::Polar({gains['Ki']:.5f}f, ...),")
    else:
        print(f"  // Rotational Model & Gains (Cập nhật cột sau Polar):")
        print(f"  .K1 = ctrl::Polar(..., {K1:.3f}f),")
        print(f"  .T1 = ctrl::Polar(..., {T1:.5f}f),")
        print(f"  .Kp = ctrl::Polar(..., {gains['Kp']:.5f}f),")
        print(f"  .Ki = ctrl::Polar(..., {gains['Ki']:.5f}f),")
    print("=" * 65 + "\n")

    # Vẽ đồ thị nếu có matplotlib
    if HAS_MATPLOTLIB and show_plot:
        fig, axs = plt.subplots(3, 1, figsize=(9, 9), tight_layout=True)

        if not is_rotational:
            # Subplot 1: Vận tốc tịnh tiến
            axs[0].plot(t, v_exp, "b-", lw=1.8, label="Thực nghiệm (Encoder v_tra)")
            axs[0].plot(t, v_sim, "r--", lw=2.0, label=f"Mô hình bậc 1 (K1={K1:.1f}, T1={T1*1000:.1f}ms)")
            axs[0].set_ylabel("Vận tốc [mm/s]")
            axs[0].set_title("Đáp ứng bước vận tốc tịnh tiến (Translational Step Response)")
            axs[0].grid(True)
            axs[0].legend(loc="lower right")

            # Subplot 2: Vận tốc từng bánh L và R
            axs[1].plot(t, data["enc0"] * SCALE_MM_PER_COUNT, label="Vị trí Bánh Trái [mm]")
            axs[1].plot(t, data["enc1"] * SCALE_MM_PER_COUNT, label="Vị trí Bánh Phải [mm]")
            axs[1].set_ylabel("Quãng đường [mm]")
            axs[1].set_title("Quãng đường 2 bánh xe")
            axs[1].grid(True)
            axs[1].legend(loc="upper left")

            # Subplot 3: Gia tốc và PWM
            axs[2].plot(t, data["accel_y"], "g-", label="Gia tốc dọc (IMU Accel Y) [mm/s²]")
            axs[2].set_ylabel("Gia tốc [mm/s²]")
            axs[2].set_xlabel("Thời gian [s]")
            axs[2].set_title("Gia tốc đo được từ IMU")
            axs[2].grid(True)
            axs[2].legend(loc="upper right")

        else:
            # Subplot 1: Vận tốc góc
            axs[0].plot(t, data["gyro_z"], "m-", lw=1.8, label="Thực nghiệm (IMU Gyro Z) [rad/s]")
            axs[0].plot(t, data["w_enc"], "c:", lw=1.5, label="Ước lượng từ Encoder [rad/s]")
            axs[0].plot(t, w_sim, "r--", lw=2.0, label=f"Mô hình bậc 1 (K1={K1:.2f}, T1={T1*1000:.1f}ms)")
            axs[0].set_ylabel("Vận tốc góc [rad/s]")
            axs[0].set_title("Đáp ứng bước quay góc (Rotational Step Response)")
            axs[0].grid(True)
            axs[0].legend(loc="lower right")

            # Subplot 2: Góc quay tích lũy
            theta_deg = np.cumsum(data["gyro_z"]) * DT * 180.0 / np.pi
            axs[1].plot(t, theta_deg, "b-", lw=2.0, label="Góc xoay Yaw tích lũy [độ]")
            axs[1].set_ylabel("Góc quay [độ]")
            axs[1].set_title("Góc Yaw tích lũy theo thời gian")
            axs[1].grid(True)
            axs[1].legend(loc="upper left")

            # Subplot 3: Gia tốc góc
            axs[2].plot(t, data["ang_acc"], "orange", label="Gia tốc góc [rad/s²]")
            axs[2].set_ylabel("Gia tốc góc [rad/s²]")
            axs[2].set_xlabel("Thời gian [s]")
            axs[2].set_title("Gia tốc góc đo được")
            axs[2].grid(True)
            axs[2].legend(loc="upper right")

        out_img = os.path.splitext(filepath)[0] + ".png"
        fig.savefig(out_img, dpi=200)
        print(f"[INFO] Đã lưu đồ thị vào: {out_img}")
        plt.show()

    return K1, T1, gains


def main():
    parser = argparse.ArgumentParser(description="Kerise v4 System Identification & PID Tuner")
    parser.add_argument("file", nargs="?", default="", help="Đường dẫn file .tab / .txt dữ liệu đã lưu")
    parser.add_argument("--port", "-p", default="", help="Cổng Serial/Bluetooth (ví dụ: COM5 hoặc /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baudrate Serial (mặc định: 115200)")
    parser.add_argument("--dir", "-d", type=int, default=0, choices=[0, 1], help="0: Thẳng (Translational), 1: Quay (Rotational)")
    parser.add_argument("--duty", "-u", type=float, default=0.20, help="Mức PWM duty cấp (0.05 - 0.80, mặc định: 0.20)")
    parser.add_argument("--duration", "-t", type=int, default=1000, help="Thời gian chạy SysID [ms] (mặc định: 1000)")
    parser.add_argument("--no-plot", action="store_true", help="Không hiện cửa sổ đồ thị (chỉ in kết quả và lưu file)")

    args = parser.parse_args()

    data_file = args.file

    # Nếu có chỉ định cổng Serial thì ưu tiên chạy test và thu thập dữ liệu trực tiếp
    if args.port:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        dir_name = "rot" if args.dir == 1 else "tra"
        save_file = f"tools/sysid/data/sysid_{dir_name}_{int(args.duty*100)}pct_{timestamp}.tab"
        data_file = capture_from_serial(args.port, args.baud, args.dir, args.duty, args.duration, save_file)

    if not data_file:
        print("[HƯỚNG DẪN SỬ DỤNG]:")
        print("  1. Phân tích file JSON export từ VS Code Teleplot:")
        print("     python tools/sysid/sysid_analyzer.py teleplot_export.json")
        print("  2. Phân tích file log .tab chuẩn Kerise:")
        print("     python tools/sysid/sysid_analyzer.py tools/sysid/sample_tra.tab")
        print("  3. Thu thập trực tiếp qua Bluetooth/Serial:")
        print("     python tools/sysid/sysid_analyzer.py --port COM5 --dir 0 --duty 0.2")
        sys.exit(0)

    # Nếu file đầu vào là file JSON từ Teleplot, tự động chuyển đổi sang TAB trước
    if data_file.lower().endswith(".json"):
        import teleplot_to_kerise
        base, _ = os.path.splitext(data_file)
        tab_file = base + ".tab"
        data_file, _, _, _, _ = teleplot_to_kerise.convert_teleplot_to_kerise(data_file, tab_file)

    analyze_and_plot(data_file, show_plot=not args.no_plot)


if __name__ == "__main__":
    main()
