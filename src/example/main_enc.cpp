// ============================================================================
// Encoder-only print program: prints encoder position and acceleration.
//
// Design:
//   1. encoder_thread: reads encoder position at ~100 Hz → ring buffer +
//      writes latest position to atomics for main loop printing
//   2. Acceleration computed in the main loop using finite difference on
//      timestamped position samples (no Savitzky-Golay).
//      vel  = Δpos / Δt   (using actual timestamps)
//      acc  = Δvel / Δt   (using actual timestamps, then EMA-filtered)
//   3. Oscillation detection to suppress acceleration during oscillation
// ============================================================================

#include "Encoder.h"
#include "driver/EncoderDriver.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

// ---------------------------------------------------------------------------
// Global running flag + signal handler
// ---------------------------------------------------------------------------
static std::atomic<bool> g_running{true};

static void on_sigint(int) { g_running = false; }

static int64_t now_us() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
               now.time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// Ring buffer of (position [counts], timestamp [us]) samples.
// ---------------------------------------------------------------------------
static constexpr std::size_t kBufSize = 64;

struct Sample {
    int64_t pos   = 0;
    int64_t ts_us = 0;
};

// ---------------------------------------------------------------------------
// Oscillation detection: checks if position is bouncing within a bounded range.
// ---------------------------------------------------------------------------
class OscillationDetector {
public:
    OscillationDetector(double threshold_deg, std::size_t window_size)
        : threshold_deg_(threshold_deg), window_size_(window_size) {}

    bool feed(double pos_deg) {
        buf_[head_] = pos_deg;
        head_ = (head_ + 1) % window_size_;
        if (count_ < window_size_) {
            ++count_;
            return false;
        }

        double min = buf_[head_], max = buf_[head_];
        int reversals = 0;
        int prev_sign = 0;
        for (std::size_t i = 1; i < window_size_; ++i) {
            std::size_t idx = (head_ + i) % window_size_;
            if (buf_[idx] < min) min = buf_[idx];
            if (buf_[idx] > max) max = buf_[idx];

            std::size_t prev = (head_ + i - 1) % window_size_;
            double diff = buf_[idx] - buf_[prev];
            int sign = (diff > 0.001) ? 1 : ((diff < -0.001) ? -1 : 0);
            if (sign != 0 && sign != prev_sign && prev_sign != 0)
                ++reversals;
            if (sign != 0) prev_sign = sign;
        }

        double range = max - min;
        return (range <= threshold_deg_) && (reversals >= 2);
    }

private:
    double                  threshold_deg_;
    std::size_t             window_size_;
    std::array<double, 64>  buf_{};
    std::size_t             head_  = 0;
    std::size_t             count_ = 0;
};

// ===========================================================================
int main() {
    std::signal(SIGINT, on_sigint);

    // --- Encoder configuration ---
    driver::EncoderDriver::Config enc_cfg;
    enc_cfg.port       = "/dev/ttyUSB0";
    enc_cfg.baud_rate  = 115200;
    enc_cfg.parity     = 'N';
    enc_cfg.slave_addr = 1;

    auto enc = std::make_unique<Encoder>(
                     std::make_unique<driver::EncoderDriver>(enc_cfg));

    if (!enc->open()) {
        std::cerr << "Failed to open encoder on " << enc_cfg.port << "\n";
        return 1;
    }

    constexpr double counts_per_deg = 40.0;

    OscillationDetector osc_det(/*threshold=*/3.0, /*window=*/32);

    // --- Shared state ---
    Sample              ring[kBufSize]{};
    std::atomic<std::size_t> ring_head{0};
    std::atomic<std::size_t> ring_count{0};

    std::atomic<double> latest_acc{0.0};
    std::atomic<bool>   latest_acc_ready{false};
    std::atomic<bool>   is_oscillating{false};

    std::cout << "Reading encoder position and acceleration (Ctrl+C to stop)...\n";

    // --- Encoder reader thread ---
    std::thread t_enc = std::thread([&]() {
        while (g_running) {
            int64_t pos = 0;
            if (enc->read(pos)) {
                int64_t ts = now_us();
                std::size_t h = ring_head.load(std::memory_order_relaxed);
                ring[h] = {pos, ts};
                std::size_t next = (h + 1) % kBufSize;
                ring_head.store(next, std::memory_order_release);
                std::size_t cnt = ring_count.load(std::memory_order_relaxed);
                if (cnt < kBufSize)
                    ring_count.store(cnt + 1, std::memory_order_relaxed);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // --- Main loop: compute velocity and acceleration from timestamps ---
    double ema_vel        = 0.0;   // EMA-filtered velocity
    double ema_acc        = 0.0;   // EMA-filtered acceleration
    double last_print_vel = 0.0;
    double last_print_acc = 0.0;
    double last_print_pos = 0.0;
    bool   first          = true;

    while (g_running) {
        std::size_t count = ring_count.load(std::memory_order_relaxed);
        if (count < 4) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::size_t head = ring_head.load(std::memory_order_acquire);

        // Read the 4 most recent samples from the ring (chronological order).
        // head points to the next write slot, so newest is (head-1).
        auto idx = [&](int offset) -> std::size_t {
            return (head + kBufSize + offset) % kBufSize;
        };

        Sample s0 = ring[idx(-4)];  // oldest
        Sample s1 = ring[idx(-3)];
        Sample s2 = ring[idx(-2)];
        Sample s3 = ring[idx(-1)];  // newest

        // Compute velocities from adjacent samples, using actual timestamps.
        double dt1 = static_cast<double>(s1.ts_us - s0.ts_us) * 1e-6;
        double dt2 = static_cast<double>(s2.ts_us - s1.ts_us) * 1e-6;
        double dt3 = static_cast<double>(s3.ts_us - s2.ts_us) * 1e-6;

        double vel1 = 0.0, vel2 = 0.0, vel3 = 0.0;
        if (dt1 > 0.001) vel1 = static_cast<double>(s1.pos - s0.pos) / dt1;
        if (dt2 > 0.001) vel2 = static_cast<double>(s2.pos - s1.pos) / dt2;
        if (dt3 > 0.001) vel3 = static_cast<double>(s3.pos - s2.pos) / dt3;

        // Acceleration from velocity differences.
        double acc = 0.0;
        double dt_a = (dt2 + dt3) * 0.5;
        if (dt_a > 0.001 && dt2 > 0.001 && dt3 > 0.001) {
            acc = (vel3 - vel2) / dt_a;
        }

        // Convert → deg/s, deg/s²
        double vel_deg = vel3 / counts_per_deg;
        double acc_deg = acc  / counts_per_deg;

        // EMA low-pass filter (smoothing factor ~0.3).
        constexpr double alpha = 0.3;
        ema_vel = alpha * vel_deg + (1.0 - alpha) * ema_vel;
        ema_acc = alpha * acc_deg + (1.0 - alpha) * ema_acc;

        // Oscillation detection on latest position.
        double pos_deg = static_cast<double>(s3.pos) / counts_per_deg;
        bool osc = osc_det.feed(pos_deg);
        is_oscillating.store(osc, std::memory_order_relaxed);

        latest_acc.store(ema_acc, std::memory_order_relaxed);
        latest_acc_ready.store(true, std::memory_order_release);

        // --- Print ---
        bool pos_changed = std::fabs(pos_deg - last_print_pos) >= 2.0;
        bool vel_changed = !osc && std::fabs(ema_vel - last_print_vel) >= 5.0;
        bool acc_changed = !osc && std::fabs(ema_acc - last_print_acc) >= 5.0;

        if (pos_changed || vel_changed || acc_changed || first) {
            std::cout << "Pos: " << pos_deg << " deg";
            if (osc) {
                std::cout << "  Vel: (osc)  Acc: (osc)";
            } else {
                std::cout << "  Vel: " << ema_vel << " deg/s"
                          << "  Acc: " << ema_acc << " deg/s²";
            }
            std::cout << "  ts: " << s3.ts_us << "\n";
            if (pos_changed) last_print_pos = pos_deg;
            if (vel_changed) last_print_vel = ema_vel;
            if (acc_changed) last_print_acc = ema_acc;
            first = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (t_enc.joinable()) t_enc.join();
    enc->close();
    return 0;
}
