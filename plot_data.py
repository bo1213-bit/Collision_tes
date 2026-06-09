#!/usr/bin/env python3
import csv
import sys

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

def find_change_region(t_sec, values):
    """Auto-detect region where values change significantly, return (t_start, t_end)."""
    if len(values) < 10:
        return t_sec[0], t_sec[-1]

    diffs = [abs(values[i] - values[i - 1]) for i in range(1, len(values))]
    sorted_diffs = sorted(diffs)
    p995 = sorted_diffs[int(len(sorted_diffs) * 0.995)]
    threshold = max(p995 * 0.1, 1e-9)

    start_idx = 0
    for i, d in enumerate(diffs):
        if d > threshold:
            start_idx = i
            break

    end_idx = len(diffs) - 1
    for i in range(len(diffs) - 1, -1, -1):
        if diffs[i] > threshold:
            end_idx = i + 1
            break

    padding = max(50, int(len(diffs) * 0.02))
    t_start = max(t_sec[0], t_sec[max(0, start_idx - padding)])
    t_end = min(t_sec[-1], t_sec[min(len(t_sec) - 1, end_idx + padding)])

    if t_end - t_start < 0.5:
        return t_sec[0], t_sec[-1]

    return t_start, t_end


def main():
    csv_path = "data_output.csv"
    timestamps, accel1, force = [], [], []

    try:
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamps.append(float(row["timestamp"]))
                accel1.append(float(row["accel1"]))
                force.append(float(row["force"]))
    except FileNotFoundError:
        print(f"Error: {csv_path} not found. Run encoder_example first.")
        sys.exit(1)
    except KeyError as e:
        print(f"Error: column {e} not found in CSV. Re-run encoder_example to regenerate data.")
        sys.exit(1)

    if not timestamps:
        print("Error: no data in CSV file.")
        sys.exit(1)

    t0 = timestamps[0]
    t_sec = [(t - t0) / 1e6 for t in timestamps]

    # --- Two independent subplots, each with its own x-axis ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

    # ---- Subplot 1: Acceleration ----
    ax1.plot(t_sec, accel1, linewidth=0.8, color='C0')
    ax1.set_ylabel("Accel1 (m/s^2)")
    ax1.set_title("Acceleration")
    ax1.grid(True, alpha=0.3)

    # Auto-detect change region for acceleration
    t1_start, t1_end = find_change_region(t_sec, accel1)
    ax1.set_xlim(t1_start, t1_end)
    ax1.xaxis.set_major_locator(MultipleLocator(0.1))
    print(f"Accel change region: {t1_start:.3f}s - {t1_end:.3f}s")

    # Mark max
    max_accel = max(accel1)
    max_idx_a = accel1.index(max_accel)
    max_t_a = t_sec[max_idx_a]
    ax1.axhline(max_accel, color='red', linestyle='--', linewidth=0.6, alpha=0.7)
    ax1.plot(max_t_a, max_accel, 'ro', markersize=6)
    ax1.annotate(f"max={max_accel:.4f}",
                 xy=(max_t_a, max_accel),
                 xytext=(max_t_a, max_accel * 0.85),
                 fontsize=8, color='red',
                 arrowprops=dict(arrowstyle='->', color='red', lw=0.8),
                 ha='center', va='top')

    # ---- Subplot 2: Force ----
    ax2.plot(t_sec, force, linewidth=0.8, color='C1')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.set_title("Force")
    ax2.grid(True, alpha=0.3)

    # Auto-detect change region for force independently
    t2_start, t2_end = find_change_region(t_sec, force)
    ax2.set_xlim(t2_start, t2_end)
    ax2.xaxis.set_major_locator(MultipleLocator(0.1))
    print(f"Force change region: {t2_start:.3f}s - {t2_end:.3f}s")

    # Mark max
    max_force = max(force)
    max_idx_f = force.index(max_force)
    max_t_f = t_sec[max_idx_f]
    ax2.axhline(max_force, color='red', linestyle='--', linewidth=0.6, alpha=0.7)
    ax2.plot(max_t_f, max_force, 'ro', markersize=6)
    ax2.annotate(f"max={max_force:.4f}",
                 xy=(max_t_f, max_force),
                 xytext=(max_t_f, max_force * 0.85),
                 fontsize=8, color='red',
                 arrowprops=dict(arrowstyle='->', color='red', lw=0.8),
                 ha='center', va='top')

    fig.tight_layout()
    output_png = "data_output.png"
    fig.savefig(output_png, dpi=150)
    print(f"Saved plot to {output_png}")


if __name__ == "__main__":
    main()
