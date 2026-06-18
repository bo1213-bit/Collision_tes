#!/usr/bin/env python3
"""Analyze edge_times.csv to detect photoelectric sensor edge transitions and speed.

Two sensors (CH2, CH3) spaced 3cm apart. A falling edge occurs when the
voltage drops 300mV below its baseline (object blocks the light beam).
Speed = distance / time_between_CH2_and_CH3_falling_edges.

Usage:
    python3 analyze_edges.py [csv_path] [threshold_offset_mV]

Defaults:
    csv_path = /home/lab/Collision_test/edge_times.csv
    threshold_offset_mV = 300
"""

import csv
import sys

# --- Configuration ---
CSV_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/lab/Collision_test/edge_times.csv"
THRESHOLD_OFFSET = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0  # mV below baseline
DISTANCE_M = 0.03  # 3cm between sensors
BASELINE_SAMPLES = 50  # number of initial samples for baseline estimation
# ---------------------

ts, ch2, ch3 = [], [], []
with open(CSV_PATH) as f:
    reader = csv.reader(f)
    next(reader)  # skip header
    for row in reader:
        if len(row) >= 3:
            ts.append(int(row[0]))
            ch2.append(float(row[1]))
            ch3.append(float(row[2]))

if len(ts) == 0:
    print(f"ERROR: No data in {CSV_PATH}")
    sys.exit(1)

# Baseline = median of first N samples
baseline_ch2 = sorted(ch2[:BASELINE_SAMPLES])[BASELINE_SAMPLES // 2]
baseline_ch3 = sorted(ch3[:BASELINE_SAMPLES])[BASELINE_SAMPLES // 2]
THR2 = baseline_ch2 - THRESHOLD_OFFSET
THR3 = baseline_ch3 - THRESHOLD_OFFSET

print(f"Total samples: {len(ts)}")
print(f"Duration: {(ts[-1] - ts[0]) / 1e6:.2f}s")
print(f"CH2 baseline: {baseline_ch2:.1f}mV  threshold: {THR2:.1f}mV")
print(f"CH3 baseline: {baseline_ch3:.1f}mV  threshold: {THR3:.1f}mV")
print()

# Detect falling edges
ch2_falls = []
ch3_falls = []
was2 = ch2[0] > THR2
was3 = ch3[0] > THR3

for i in range(1, len(ts)):
    is2 = ch2[i] > THR2
    is3 = ch3[i] > THR3
    if was2 and not is2:
        ch2_falls.append(ts[i])
        print(f"CH2 FALL  ts={ts[i]}  {ch2[i-1]:.1f} -> {ch2[i]:.1f}mV")
    if was3 and not is3:
        ch3_falls.append(ts[i])
        print(f"CH3 FALL  ts={ts[i]}  {ch3[i-1]:.1f} -> {ch3[i]:.1f}mV")
    was2 = is2
    was3 = is3

print(f"\nCH2 falling edges: {len(ch2_falls)}")
print(f"CH3 falling edges: {len(ch3_falls)}")
print()

# Pair CH2 with CH3 and compute speed
if len(ch2_falls) == 0 and len(ch3_falls) == 0:
    print("No falling edges detected. Try lowering the threshold offset.")
    print(f"  e.g. python3 {sys.argv[0]} {CSV_PATH} 150")
    sys.exit(0)

n_pairs = min(len(ch2_falls), len(ch3_falls))
for i in range(n_pairs):
    dt = ch3_falls[i] - ch2_falls[i]
    if dt > 0:
        speed = DISTANCE_M / (dt / 1e6)
        print(f"Pair {i}: CH2={ch2_falls[i]} -> CH3={ch3_falls[i]}, dt={dt}us ({dt/1000:.2f}ms), v={speed:.3f} m/s")
    else:
        print(f"Pair {i}: CH2={ch2_falls[i]} -> CH3={ch3_falls[i]}, dt={dt}us (CH3 before CH2, skip)")