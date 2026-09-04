#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
  TELEPLOT JSON TO KERISE TAB CONVERTER
  Chuyển đổi dữ liệu Teleplot (.json) sang định dạng Kerise (.tab / .csv)
  Dùng cho công cụ plot.py và SysID của Micromouse Kerise v4.
=============================================================================
  Cách dùng:
    python tools/sysid/teleplot_to_kerise.py <file_teleplot.json> [file_xuat.tab]
    python tools/sysid/teleplot_to_kerise.py --calc <file_teleplot.json>

  Đầu ra:
    File .tab định dạng chuẩn Kerise:
    # enc[0]	enc[1]	gyro.z	accel.y	angular_accel	u.tra	u.rot	vbat
=============================================================================
"""

import os
import sys
import json
import argparse
import numpy as np

# Cấu hình in UTF-8 console Windows
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="backslashreplace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="backslashreplace")

# Alias từ khóa để tự động nhận dạng các kênh dữ liệu từ Teleplot
CHANNEL_ALIASES = {
    "enc0": ["enc0", "enc[0]", "enc_l", "encleft", "pos0", "left_pos", "left_encoder"],
    "enc1": ["enc1", "enc[1]", "enc_r", "encright", "pos1", "right_pos", "right_encoder"],
    "gyro_z": ["gyro_z", "gyro.z", "gyroz", "gyro", "imu_gyro", "w_est"],
    "accel_y": ["accel_y", "accel.y", "accely", "accel", "imu_accel", "acc_y"],
    "angular_accel": ["angular_accel", "ang_acc", "angacc", "imu_alpha", "alpha"],
    "u_tra": ["u_tra", "u.tra", "utra", "duty", "duty_tra", "pwm", "u_pwm"],
    "u_rot": ["u_rot", "u.rot", "urot", "duty_rot"],
    "vbat": ["vbat", "battery_voltage", "bat", "voltage", "battery", "v_bat"],
}


def find_channel_data(telemetries, canonical_name):
    """Tìm mảng giá trị của kênh trong dictionary telemetries dựa trên alias."""
    aliases = CHANNEL_ALIASES.get(canonical_name, [canonical_name])
    tele_keys = list(telemetries.keys())

    for alias in aliases:
        for key in tele_keys:
            if key.lower() == alias.lower() or key.lower().replace("_", "").replace(".", "") == alias.lower().replace("_", "").replace(".", ""):
                item = telemetries[key]
                if isinstance(item, dict) and "data" in item:
                    data = item["data"]
                    if len(data) >= 2:
                        return key, np.array(data[0], dtype=float), np.array(data[1], dtype=float)
    return None, None, None


def convert_teleplot_to_kerise(json_path, output_path=None):
    """Đọc file Teleplot JSON và ghi ra file TAB chuẩn Kerise."""
    if not os.path.exists(json_path):
        print(f"[ERROR] Không tìm thấy file: {json_path}")
        sys.exit(1)

    print(f"\n[INFO] Đang mở file Teleplot: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        try:
            root = json.load(f)
        except Exception as e:
            print(f"[ERROR] Lỗi phân tích JSON: {e}")
            sys.exit(1)

    telemetries = root.get("telemetries", {})
    if not telemetries:
        print("[ERROR] Không tìm thấy khóa 'telemetries' trong file JSON!")
        sys.exit(1)

    print(f"[INFO] Các biến có trong file Teleplot: {list(telemetries.keys())}")

    extracted = {}
    found_lengths = []

    # Trích xuất từng kênh
    for ch in ["enc0", "enc1", "gyro_z", "accel_y", "angular_accel", "u_tra", "u_rot", "vbat"]:
        actual_name, t_arr, val_arr = find_channel_data(telemetries, ch)
        if actual_name is not None:
            extracted[ch] = (actual_name, t_arr, val_arr)
            found_lengths.append(len(val_arr))
            print(f"  + Kênh '{ch}' -> tìm thấy từ '{actual_name}' ({len(val_arr)} mẫu)")
        else:
            extracted[ch] = (None, None, None)
            print(f"  - Kênh '{ch}' -> không có trong JSON (sẽ gán mặc định 0.0)")

    if not found_lengths:
        print("[ERROR] Không tìm thấy kênh dữ liệu hợp lệ nào trong file JSON!")
        sys.exit(1)

    # Chiều dài chung (cắt ngắn theo kênh có ít mẫu nhất để đồng bộ mốc thời gian)
    target_len = min(found_lengths)
    print(f"[INFO] Số lượng mẫu đồng bộ chung: {target_len} điểm")

    # Điền giá trị chuẩn hóa
    final_data = {}
    for ch in ["enc0", "enc1", "gyro_z", "accel_y", "angular_accel", "u_tra", "u_rot", "vbat"]:
        actual_name, t_arr, val_arr = extracted[ch]
        if val_arr is not None:
            final_data[ch] = val_arr[:target_len]
        else:
            if ch == "vbat":
                final_data[ch] = np.full(target_len, 7.4)
            else:
                final_data[ch] = np.zeros(target_len)

    # Ước lượng mode SysID và duty để bổ sung vào header
    u_tra_max = np.max(np.abs(final_data["u_tra"]))
    u_rot_max = np.max(np.abs(final_data["u_rot"]))
    if u_rot_max > u_tra_max:
        dir_val = 1
        duty_val = u_rot_max if u_rot_max > 0 else 0.25
        mode_str = "ROTATIONAL (SPIN)"
    else:
        dir_val = 0
        duty_val = u_tra_max if u_tra_max > 0 else 0.20
        mode_str = "TRANSLATIONAL (STRAIGHT)"

    vbat_mean = float(np.mean(final_data["vbat"]))

    # Xác định đường dẫn file đầu ra
    if output_path is None:
        base, _ = os.path.splitext(json_path)
        output_path = base + ".tab"

    # Ghi file TAB chuẩn Kerise
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        # Header metadata cho Kerise và SysID Analyzer
        f.write(f"# SYSID BEGIN dir={dir_val} duty={duty_val:.3f} samples={target_len} dt=0.001 vbat={vbat_mean:.2f}\n")
        f.write("# enc[0]\tenc[1]\tgyro.z\taccel.y\tangular_accel\tu.tra\tu.rot\tvbat\n")

        for i in range(target_len):
            f.write(f"{final_data['enc0'][i]:.4f}\t"
                    f"{final_data['enc1'][i]:.4f}\t"
                    f"{final_data['gyro_z'][i]:.6f}\t"
                    f"{final_data['accel_y'][i]:.4f}\t"
                    f"{final_data['angular_accel'][i]:.4f}\t"
                    f"{final_data['u_tra'][i]:.3f}\t"
                    f"{final_data['u_rot'][i]:.3f}\t"
                    f"{final_data['vbat'][i]:.2f}\n")

    print(f"\n[SUCCESS] Đã chuyển đổi thành công sang file Kerise TAB:")
    print(f"  -> File: {output_path}")
    print(f"  -> Chế độ nhận diện: {mode_str} (dir={dir_val}, duty={duty_val:.3f})")
    print(f"  -> Tổng số dòng dữ liệu: {target_len}")

    print("\n[HƯỚNG DẪN DÙNG VỚI KERISE plot.py]:")
    print(f"  python tools/sysid/plot_kerise_compat.py {output_path}")
    print(f"  Hoặc mở trong MATLAB: load('{output_path}')")

    return output_path, final_data, dir_val, duty_val, vbat_mean


def quick_calc_sysid(final_data, dir_val, duty_val):
    """Tính toán nhanh K1, T1 và thông số PID trực tiếp từ dữ liệu."""
    dt = 0.001
    enc0 = final_data["enc0"]
    enc1 = final_data["enc1"]
    n = len(enc0)
    t = np.arange(n) * dt

    rotation_radius_mm = 38.7619

    # Tính vi sai vận tốc từ quãng đường mm
    d_enc0 = np.diff(enc0, prepend=enc0[0])
    d_enc1 = np.diff(enc1, prepend=enc1[0])

    v_L = d_enc0 / dt  # mm/s
    v_R = d_enc1 / dt  # mm/s

    # Lọc trung bình trượt nhẹ 11 mẫu để giảm nhiễu vi sai
    kernel = np.ones(11) / 11.0
    v_L_smooth = np.convolve(v_L, kernel, mode="same")
    v_R_smooth = np.convolve(v_R, kernel, mode="same")

    v_tra = (v_L_smooth + v_R_smooth) * 0.5
    w_rot = (v_R_smooth - v_L_smooth) / (2.0 * rotation_radius_mm)

    if dir_val == 1:
        # Quay tại chỗ
        y = w_rot
        val_name = "Vận tốc góc w (rad/s)"
        u_val = duty_val
        is_spin = True
    else:
        # Chạy thẳng
        y = v_tra
        val_name = "Vận tốc thẳng v (mm/s)"
        u_val = duty_val
        is_spin = False

    # Điểm xác lập (lấy trung bình 20% cuối)
    tail_len = max(10, int(n * 0.2))
    y_ss = float(np.median(y[-tail_len:]))
    if abs(u_val) < 1e-4:
        u_val = 0.2

    K1 = y_ss / u_val

    # Tìm T1: thời gian đạt 63.2% giá trị xác lập
    y_63 = 0.6321 * y_ss
    idx_63 = np.where(y >= y_63)[0]
    if len(idx_63) > 0:
        T1 = float(t[idx_63[0]])
    else:
        T1 = 0.35

    # Tính Feedforward
    C1 = 1.0 / K1 if abs(K1) > 1e-6 else 0.0
    C2 = T1 / K1 if abs(K1) > 1e-6 else 0.0

    # Tính Feedback PID theo Pole Placement (omega_n)
    omega_n = 20.0 if not is_spin else 25.0
    zeta = 0.85
    Kp = (2.0 * zeta * omega_n * T1 - 1.0) / K1
    Ki = (omega_n * omega_n * T1) / K1
    Kd = 0.0
    if Kp < 0.001:
        Kp = 0.01

    print("\n" + "=" * 60)
    print("       KẾT QUẢ NHẬN DẠNG MÔ HÌNH HỆ THỐNG (SysID)")
    print("=" * 60)
    print(f" Đại lượng phân tích      : {val_name}")
    print(f" Mức xung thử nghiệm (Duty): {u_val * 100:.1f}% ({u_val:.3f})")
    print(f" Vận tốc xác lập (Steady) : {y_ss:.2f} {'rad/s' if is_spin else 'mm/s'}")
    print(f" Hệ số khuếch đại (K1)    : {K1:.4f} {'rad/s / duty' if is_spin else 'mm/s / duty'}")
    print(f" Hằng số thời gian (T1)   : {T1:.4f} s ({T1*1000:.1f} ms)")
    print("-" * 60)
    print(f" Feedforward C1 (1/K1)    : {C1:.6e}")
    print(f" Feedforward C2 (T1/K1)   : {C2:.6e}")
    print(f" Feedback Kp              : {Kp:.5f}")
    print(f" Feedback Ki              : {Ki:.5f}")
    print("=" * 60)

    print("\n[MÃ NGUỒN C++ DÁN VÀO src/config/model.h]:")
    if not is_spin:
        print(f"  // TRANSLATIONAL (Tịnh tiến thẳng):")
        print(f"  // K1.tra = {K1:.3f}f; T1.tra = {T1:.4f}f;")
        print(f"  // Kp.tra = {Kp:.4f}f; Ki.tra = {Ki:.4f}f;")
    else:
        print(f"  // ROTATIONAL (Quay tại chỗ):")
        print(f"  // K1.rot = {K1:.3f}f; T1.rot = {T1:.4f}f;")
        print(f"  // Kp.rot = {Kp:.4f}f; Ki.rot = {Ki:.4f}f;")


def main():
    parser = argparse.ArgumentParser(description="Chuyển đổi Teleplot JSON sang Kerise TAB và tính SysID")
    parser.add_argument("json_file", help="Đường dẫn file Teleplot JSON export")
    parser.add_argument("output_tab", nargs="?", default=None, help="Đường dẫn file .tab xuất ra (tùy chọn)")
    parser.add_argument("--calc", action="store_true", help="Tính toán luôn K1, T1 và PID sau khi chuyển đổi")
    args = parser.parse_args()

    out_file, final_data, dir_val, duty_val, vbat = convert_teleplot_to_kerise(args.json_file, args.output_tab)

    if args.calc:
        quick_calc_sysid(final_data, dir_val, duty_val)


if __name__ == "__main__":
    main()
