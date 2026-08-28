#!/usr/bin/env python3
"""
calibrate_inclinometer.py
Linear regression script for inclinometer calibration (Angle vs Voltage).
Reads calibration data and outputs C++ #define constants, R², RMSE, and residual errors.
"""

import sys
import os

def calibrate(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File not found '{file_path}'")
        sys.exit(1)

    angles = []
    voltages = []

    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "angle" in line.lower() or "voltage" in line.lower():
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    a = float(parts[0].replace(",", "."))
                    v = float(parts[1].replace(",", "."))
                    angles.append(a)
                    voltages.append(v)
                except ValueError:
                    continue

    n = len(angles)
    if n < 2:
        print("Error: Need at least 2 valid data points to perform linear regression.")
        sys.exit(1)

    mean_v = sum(voltages) / n
    mean_a = sum(angles) / n

    ss_vv = sum((v - mean_v) ** 2 for v in voltages)
    ss_va = sum((v - mean_v) * (a - mean_a) for v, a in zip(voltages, angles))
    ss_aa = sum((a - mean_a) ** 2 for a in angles)

    gain = ss_va / ss_vv
    offset = mean_a - gain * mean_v

    pred_angles = [gain * v + offset for v in voltages]
    residuals = [a - p for a, p in zip(angles, pred_angles)]
    ss_res = sum(r ** 2 for r in residuals)
    r2 = 1.0 - (ss_res / ss_aa) if ss_aa != 0 else 0.0
    rmse = (ss_res / n) ** 0.5
    max_err = max(abs(r) for r in residuals)

    print("=" * 65)
    print("      LIFT INCLINOMETER LINEAR REGRESSION CALIBRATION")
    print("=" * 65)
    print(f"Data file          : {file_path}")
    print(f"Points analyzed    : {n}")
    print(f"R² (Linearity)     : {r2:.7f} ({r2 * 100:.4f} %)")
    print(f"RMSE (Avg Error)   : {rmse:.4f} °")
    print(f"Max Absolute Error : {max_err:.4f} °")
    print("-" * 65)
    print(f"{'Voltage (V)':<12} | {'True Angle (°)':<15} | {'Model Angle (°)':<15} | {'Residual (°)':<12}")
    print("-" * 65)
    for v, a, p, r in zip(voltages, angles, pred_angles, residuals):
        print(f"{v:<12.3f} | {a:<15.2f} | {p:<15.2f} | {r:+12.3f}")
    print("=" * 65)
    print("\nGenerated C++ Configuration for lib/Lift/Lift.h:\n")
    print(f"// Calibration Inclinomètre (R² = {r2:.6f}, RMSE = {rmse:.3f} deg, Max Err = {max_err:.3f} deg)")
    print(f"#define ANALOG_TO_ANGLE_GAIN     {gain:.6f}f   // deg/V")
    print(f"#define ANALOG_TO_ANGLE_OFFSET   {offset:.6f}f   // deg\n")

if __name__ == "__main__":
    default_path = os.path.join(os.path.dirname(__file__), "..", "src", "lift-calibration.txt")
    target_file = sys.argv[1] if len(sys.argv) > 1 else default_path
    calibrate(target_file)
