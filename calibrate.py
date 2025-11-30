# Calibration data generated with code at `https://github.com/thmasq/hall-effect`

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
import textwrap


def load_log_file(filename):
    try:
        with open(filename, "r") as f:
            content = f.read()
        matches = re.findall(r"Voltage:\s*(\d+)\s*mV", content)
        if not matches:
            print("Error: No voltage readings found in log file.")
            sys.exit(1)
        return np.array([int(m) for m in matches])
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)


def detect_stable_plateaus(data, window_size=20, variance_threshold=10.0):
    """
    Finds regions where the signal variance is low (user is holding the trigger still).
    Returns a list of (start_index, end_index, average_voltage) for each plateau.
    """
    variance_signal = []

    for i in range(len(data) - window_size):
        window = data[i : i + window_size]
        variance_signal.append(np.var(window))

    variance_signal = np.array(variance_signal)

    stable_indices = np.where(variance_signal < variance_threshold)[0]

    if len(stable_indices) == 0:
        return []

    plateaus = []
    if len(stable_indices) > 0:
        current_start = stable_indices[0]
        current_end = stable_indices[0]

        for idx in stable_indices[1:]:
            if idx <= current_end + 5:
                current_end = idx
            else:
                if (current_end - current_start) > window_size:
                    avg_v = np.mean(data[current_start : current_end + window_size])
                    plateaus.append((current_start, current_end + window_size, avg_v))
                current_start = idx
                current_end = idx

        if (current_end - current_start) > window_size:
            avg_v = np.mean(data[current_start : current_end + window_size])
            plateaus.append((current_start, current_end + window_size, avg_v))

    return plateaus


def main():
    filename = sys.argv[1] if len(sys.argv) > 1 else "log.txt"
    print(f"--- Processing {filename} ---")

    voltages = load_log_file(filename)

    noise_est = np.std(np.diff(voltages))
    print(f"Estimated Signal Noise (StdDev): {noise_est:.2f} mV")

    plateaus = detect_stable_plateaus(voltages)

    if len(plateaus) < 3:
        print(
            f"Warning: Only found {len(plateaus)} stable regions. Expected 3 (0%, 50%, 100%)."
        )
        print("Using Min and Max of entire dataset as fallbacks.")
        v_low = np.min(voltages)
        v_high = np.max(voltages)
        v_mid = (v_low + v_high) / 2
    else:
        sorted_p = sorted(plateaus, key=lambda x: x[2])
        v_low = sorted_p[0][2]
        v_high = sorted_p[-1][2]

        math_avg = (v_low + v_high) / 2
        remaining = [p for p in plateaus if p != sorted_p[0] and p != sorted_p[-1]]
        if remaining:
            v_mid = min(remaining, key=lambda x: abs(x[2] - math_avg))[2]
        else:
            v_mid = math_avg

    print("\n--- CALIBRATION POINTS DETECTED ---")
    print(f"0%   (Low) : {v_low:.1f} mV")
    print(f"50%  (Mid) : {v_mid:.1f} mV")
    print(f"100% (High): {v_high:.1f} mV")

    linear_mid = (v_low + v_high) / 2
    deviation = v_mid - linear_mid
    if abs(deviation) > 20:
        print(f"-> Non-linear response detected (Deviation: {deviation:.1f} mV).")
    else:
        print("-> Response is mostly linear.")

    X = [v_low, v_mid, v_high]
    Y = [0.0, 0.5, 1.0]

    coeffs = np.polyfit(X, Y, 2)

    print("\n" + "=" * 40)
    print("      RUST IMPLEMENTATION      ")
    print("=" * 40)

    rust_template = textwrap.dedent(f"""
        // Auto-generated calibration constants
        const V_MIN: f32 = {v_low:.1f};
        const V_MAX: f32 = {v_high:.1f};

        // Polynomial Coefficients (a*x^2 + b*x + c)
        // Input: Voltage (mV), Output: Throttle (0.0-1.0)
        const POLY_A: f32 = {coeffs[0]:.8e};
        const POLY_B: f32 = {coeffs[1]:.8e};
        const POLY_C: f32 = {coeffs[2]:.8e};

        pub fn calculate_throttle(voltage_mv: f32) -> u8 {{
            // 1. Clamp input to calibrated range
            let v = voltage_mv.clamp(V_MIN, V_MAX);
            
            // 2. Apply Polynomial: ax^2 + bx + c
            let t = (POLY_A * v * v) + (POLY_B * v) + POLY_C;
            
            // 3. Clamp result to 0.0 - 1.0 (float precision safety)
            let t_safe = t.clamp(0.0, 1.0);
            
            (t_safe * 100.0) as u8
        }}
    """)
    print(rust_template)
    print("=" * 40)

    try:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        ax1.plot(voltages, label="Raw Sensor Data", color="lightgray")

        for p in plateaus:
            ax1.axvspan(p[0], p[1], color="green", alpha=0.1)
            ax1.hlines(p[2], p[0], p[1], color="green", linewidth=2)

        ax1.set_title(
            f"Raw Signal Analysis (Time Domain)\n0%={v_low:.0f}mV, 50%={v_mid:.0f}mV, 100%={v_high:.0f}mV"
        )
        ax1.set_xlabel("Sample Index")
        ax1.set_ylabel("Voltage (mV)")
        ax1.legend(loc="upper left")
        ax1.grid(True, alpha=0.3)

        v_range = np.linspace(v_low, v_high, 100)
        throttle_curve = (coeffs[0] * v_range**2) + (coeffs[1] * v_range) + coeffs[2]
        throttle_curve = throttle_curve * 100.0

        ax2.plot(v_range, throttle_curve, "b-", linewidth=2, label="Polynomial Fit")

        ax2.scatter(
            [v_low, v_mid, v_high],
            [0, 50, 100],
            color="red",
            s=100,
            zorder=5,
            label="Measured Anchors",
        )

        ax2.plot([v_low, v_high], [0, 100], "k--", alpha=0.3, label="Linear Reference")

        ax2.set_title("Calibration Curve (Voltage -> Throttle)")
        ax2.set_xlabel("Input Voltage (mV)")
        ax2.set_ylabel("Output Throttle (%)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        print("\nDisplaying plot... (Close window to exit)")
        plt.show()
    except Exception as e:
        print(f"Skipping plot: {e}")


if __name__ == "__main__":
    main()
