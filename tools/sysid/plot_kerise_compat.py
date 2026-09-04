#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
  KERISE v4 COMPATIBLE SYSID PLOTTER (plot.py)
  Author: Ryotaro Onuki <kerikun11+github@gmail.com> (Original Kerise v4)
  Compatibility updates: Antigravity Assistant
============================================================================
  Chạy được trực tiếp trên cả Windows / Linux / macOS.
  Không yêu cầu bắt buộc thư viện pyserial khi đọc từ file .tab / .csv!
============================================================================
"""

import os
import sys
import math
import argparse
import datetime
import numpy as np

# Cấu hình in UTF-8
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="backslashreplace")

# Matplotlib (tùy chọn)
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Serial (tùy chọn)
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


def serial_import(filename, serial_port, serial_baudrate):
    if not HAS_SERIAL:
        print("[ERROR] Cần thư viện pyserial để đọc trực tiếp từ cổng COM!")
        print("Gợi ý: Dùng Teleplot trong VS Code hoặc cài pyserial.")
        sys.exit(1)

    with serial.Serial(serial_port, serial_baudrate, timeout=10) as ser:
        ser.flush()
        print(f"[INFO] Cổng {serial_port} ({serial_baudrate}) đang lắng nghe...")
        firstline = ser.readline()
        if not firstline:
            print("[ERROR] Timeout không nhận được dữ liệu :(")
            sys.exit(1)
        ser.timeout = 0.1
        lines = ser.readlines()
        lines.insert(0, firstline)
        os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
        with open(filename, "w", encoding="utf-8") as f:
            for line in lines:
                f.write(line.decode("utf-8", errors="ignore"))


def plt_label(title, xlabel, ylabel):
    if not HAS_MATPLOTLIB:
        return
    plt.grid(True)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.tight_layout()


def iir_tra(enc, acc, u, T1, K1, dt, p_enc, p_acc, p_mod):
    if p_mod is None:
        p_mod = 1 - p_enc - p_acc
    if p_acc is None:
        p_acc = 1 - p_enc - p_mod
    n = len(enc)
    out = np.zeros(n)
    out[0] = enc[0]
    for i in range(1, n):
        v_acc = out[i - 1] + acc[i - 1] * dt
        A = math.exp(-dt / T1)
        v_mod = A * out[i - 1] + K1 * (1 - A) * u[i - 1]
        out[i] = p_enc * enc[i] + p_acc * v_acc + p_mod * v_mod
    return out


def iir_rot(enc, gyr, acc, u, T1, K1, dt, p_enc, p_gyr, p_acc, p_mod):
    if p_mod is None:
        p_mod = 1 - p_enc - p_acc - p_gyr
    if p_acc is None:
        p_acc = 1 - p_enc - p_mod - p_gyr
    n = len(enc)
    out = np.zeros(n)
    out[0] = (enc[0] * p_enc + gyr[0] * p_gyr) / (p_enc + p_gyr) if (p_enc + p_gyr) != 0 else 0
    for i in range(1, n):
        v_acc = out[i - 1] + acc[i - 1] * dt
        A = math.exp(-dt / T1)
        v_mod = A * out[i - 1] + K1 * (1 - A) * u[i - 1]
        out[i] = p_enc * enc[i] + p_gyr * gyr[i - 1] + p_acc * v_acc + p_mod * v_mod
    return out


def calc_x_y_th(v, w, dt):
    n = len(v)
    x, y = np.zeros(n), np.zeros(n)
    th = np.zeros(n)
    for i in range(1, n):
        th[i] = th[i - 1] + w[i - 1] * dt
        x[i] = x[i - 1] + v[i - 1] * math.cos(th[i]) * dt
        y[i] = y[i - 1] + v[i - 1] * math.sin(th[i]) * dt
    return x, y, th


def process(filename):
    print(f"\n[INFO] Đang xử lý file dữ liệu Kerise: {filename}")
    # Đọc dữ liệu (bỏ qua các dòng comment bắt đầu bằng #)
    raw = np.loadtxt(filename, delimiter="\t", comments="#")
    raw = raw.T  # Chuyển vị thành dạng hàng

    enc_raw = raw[0:2]
    gyro = raw[2]
    accel = raw[3:5]
    u_pwm = raw[5:7]

    dt = 1e-3  # 1ms
    machine_rotation_radius = 38.7619  # [mm]

    # Tính vi sai encoder: [L, R, (L+R)/2, (R-L)/2]
    enc_diff = np.diff(enc_raw, axis=1)
    enc_diff = np.vstack((enc_diff, (enc_diff[0] + enc_diff[1]) / 2.0))
    enc_diff = np.vstack((enc_diff, (enc_diff[1] - enc_diff[0]) / 2.0))
    v_enc = enc_diff[2] / dt
    w_enc = enc_diff[3] / dt / machine_rotation_radius
    n = len(enc_diff[0])

    print(f"[INFO] Tổng số mẫu: {n} | Thời gian đo: {n * dt:.2f} s")
    print(f"[INFO] Vận tốc tịnh tiến max: {np.max(np.abs(v_enc)):.1f} mm/s")
    print(f"[INFO] Vận tốc góc max      : {np.max(np.abs(w_enc)):.2f} rad/s")

    # Ước lượng K1, T1 từ dữ liệu
    u_tra = u_pwm[0]
    u_rot = u_pwm[1]

    if np.max(np.abs(u_rot)) > np.max(np.abs(u_tra)):
        # Mode quay (Rotational)
        u_val = float(np.max(np.abs(u_rot)))
        w_ss = float(np.median(w_enc[-int(n*0.2):]))
        K1_rot = w_ss / u_val if u_val > 0 else 66.72
        T1_rot = 0.15
        print(f"[SysID] Nhận diện chế độ QUAY (SPIN): K1 = {K1_rot:.2f} rad/s/duty")
    else:
        # Mode thẳng (Translational)
        u_val = float(np.max(np.abs(u_tra)))
        v_ss = float(np.median(v_enc[-int(n*0.2):]))
        K1_tra = v_ss / u_val if u_val > 0 else 5833.0
        T1_tra = 0.35
        print(f"[SysID] Nhận diện chế độ THẲNG (STRAIGHT): K1 = {K1_tra:.2f} mm/s/duty")

    if not HAS_MATPLOTLIB:
        print("\n[NOTE] Thư viện 'matplotlib' chưa được cài đặt trong môi trường Python này.")
        print("  -> Bạn đã có thể xem trực quan toàn bộ đồ thị theo thời gian thực trên VS Code Teleplot!")
        print("  -> File .tab đã sẵn sàng để mở bằng MATLAB: `load('" + filename + "')`")
        return

    # 1. Đồ thị vi sai Encoder
    plt.figure()
    plt.plot(enc_diff.T)
    plt_label("Diff of Encoder Value", "Sample", "Difference [mm/ms]")
    plt.legend(["Left", "Right", "(+Left+Right)/2", "(-Left+Right)/2"])

    # 2. Rotational Velocity
    T1_rot = 0.1499
    K1_rot = 66.72
    w_gyr = gyro[:-1]
    acc_rot = accel[1][:-1]
    u_rot_sig = u_pwm[1][:-1]

    w_cmp_a = iir_rot(w_enc, w_gyr, acc_rot, u_rot_sig, T1_rot, K1_rot, dt, p_enc=0.0, p_gyr=0.5, p_acc=None, p_mod=0)
    w_cmp_m = iir_rot(w_enc, w_gyr, acc_rot, u_rot_sig, T1_rot, K1_rot, dt, p_enc=0.0, p_gyr=0.5, p_acc=0, p_mod=None)
    w_cmp_eam = iir_rot(w_enc, w_gyr, acc_rot, u_rot_sig, T1_rot, K1_rot, dt, p_enc=0.0, p_gyr=0.2, p_acc=0.4, p_mod=0.4)

    plt.figure()
    plt.plot(w_enc)
    plt.plot(w_gyr)
    plt.plot(w_cmp_a)
    plt.plot(w_cmp_m)
    plt.plot(w_cmp_eam)
    plt_label("Rotational Velocity", "Time [ms]", "Angular Velocity [rad/s]")
    plt.legend(["Encoder", "IMU Gyro", "Gyro + Accel", "Gyro + Model", "Gyro + Accel + Model"])

    # 3. Translational Velocity
    T1_tra = 0.3694
    K1_tra = 5833.0
    acc_tra = accel[0][:-1]
    u_tra_sig = u_pwm[0][:-1]

    v_cmp_a = iir_tra(v_enc, acc_tra, u_tra_sig, T1_tra, K1_tra, dt, p_enc=0.05, p_acc=None, p_mod=0)
    v_cmp_m = iir_tra(v_enc, acc_tra, u_tra_sig, T1_tra, K1_tra, dt, p_enc=0.05, p_acc=0, p_mod=None)
    v_cmp_eam = iir_tra(v_enc, acc_tra, u_tra_sig, T1_tra, K1_tra, dt, p_enc=0.1, p_acc=0.45, p_mod=None)

    plt.figure()
    plt.plot(v_enc)
    plt.plot(v_cmp_a)
    plt.plot(v_cmp_m)
    plt.plot(v_cmp_eam)
    plt_label("Translational Velocity", "Time [ms]", "Velocity [mm/s]")
    plt.legend([
        "with Encoder",
        "with Encoder and Accel (IMU)",
        "with Encoder and Model (SysID)",
        "with Encoder, Accel and Model",
    ], loc="best")

    # 4. Quỹ đạo X-Y (Planar Shape)
    plt.figure()
    x, y, th = calc_x_y_th(v_enc, w_gyr, dt)
    plt.plot(x, y, label="with Encoder")
    x, y, th = calc_x_y_th(v_cmp_a, w_gyr, dt)
    plt.plot(x, y, label="with Encoder + Accel")
    x, y, th = calc_x_y_th(v_cmp_m, w_gyr, dt)
    plt.plot(x, y, label="with Encoder + Model")
    plt_label("Planar Shape", "x [mm]", "y [mm]")
    plt.axis("equal")
    plt.legend()

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Kerise v4 SysID Plotter")
    parser.add_argument("files", help="Danh sách file dữ liệu .tab / .csv", nargs="*")
    parser.add_argument("--port", "-p", help="Cổng Serial (nếu đọc trực tiếp)", default=None)
    parser.add_argument("--baud", "-b", help="Baudrate Serial", default=115200, type=int)
    args = parser.parse_args()

    files = args.files
    if not files and args.port:
        dt_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"tools/sysid/data/{dt_str}.tab"
        serial_import(filename, args.port, args.baud)
        files.append(filename)

    if not files:
        print("[HƯỚNG DẪN]:")
        print("  python tools/sysid/plot_kerise_compat.py <file_du_lieu.tab>")
        print("  Ví dụ: python tools/sysid/plot_kerise_compat.py tools/sysid/sample_tra.tab")
        sys.exit(0)

    for f in files:
        process(f)


if __name__ == "__main__":
    main()
