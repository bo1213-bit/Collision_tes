#!/usr/bin/env python3
"""实时绘图：边采集边显示，自动检测变化区域，每秒刷新"""
import csv
import sys
import time
import os

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator


def find_change_region(t_sec, velocity1):
    """自动找到速度变化显著的区域"""
    if len(velocity1) < 10:
        return t_sec[0], t_sec[-1]

    diffs = [abs(velocity1[i] - velocity1[i - 1]) for i in range(1, len(velocity1))]
    sorted_diffs = sorted(diffs)
    p995 = sorted_diffs[int(len(sorted_diffs) * 0.995)]
    threshold = max(p995 * 0.1, 0.001)

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

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title("Real-time Collision Monitor")

    def update(frame):
        if not os.path.exists(csv_path):
            return ax1, ax2

        timestamps, accel1, velocity1 = [], [], []
        try:
            with open(csv_path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    timestamps.append(float(row["timestamp"]))
                    accel1.append(float(row["accel1"]))
                    velocity1.append(float(row["velocity1"]))
        except Exception:
            return ax1, ax2

        if len(timestamps) < 2:
            return ax1, ax2

        t0 = timestamps[0]
        t_sec = [(t - t0) / 1e6 for t in timestamps]

        t_start, t_end = find_change_region(t_sec, velocity1)

        ax1.clear()
        ax2.clear()

        ax1.plot(t_sec, accel1, linewidth=0.8)
        ax1.set_ylabel("Accel1 (m/s^2)")
        ax1.set_title("ADC1 Acceleration (live)")
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(t_start, t_end)
        ax1.xaxis.set_major_locator(MultipleLocator(0.1))

        ax2.plot(t_sec, velocity1, linewidth=0.8)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Velocity1 (m/s)")
        ax2.set_title("ADC1 Integrated Velocity (live)")
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(t_start, t_end)
        ax2.xaxis.set_major_locator(MultipleLocator(0.1))

        fig.tight_layout()
        return ax1, ax2

    ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()