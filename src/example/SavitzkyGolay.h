#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// Savitzky-Golay 2nd-order polynomial second-derivative estimator.
///
/// Fixed window of 7 samples, polynomial order 2. The classical SG
/// coefficients for the second derivative at the centre of a length-7
/// window are:
///     c = [5, 0, -3, -4, -3, 0, 5] / 42
/// and the resulting value must be divided by dt² to obtain the actual
/// second derivative in (position units)/s².
///
/// This implementation evaluates the second derivative at the *centre*
/// of the window, so its output corresponds to the sample that is
/// (window/2) frames in the past. That latency (≈3 samples at 100 Hz
/// → 30 ms) is acceptable for the collision-detection use case.
///
/// All math is O(window) per feed() call; no allocations, no blocking.
class SavitzkyGolayAcc {
public:
    static constexpr std::size_t kWindow = 7;

    /// Push one (position, timestamp) sample. Returns true once the
    /// buffer is full and acceleration() is meaningful.
    bool feed(double pos, int64_t ts_us) {
        buf_pos_[head_] = pos;
        buf_ts_[head_]  = ts_us;
        head_ = (head_ + 1) % kWindow;
        if (count_ < kWindow) {
            ++count_;
            return count_ == kWindow;
        }
        return true;
    }

    bool ready() const { return count_ == kWindow; }

    /// Acceleration at the centre of the window, in (position units)/s².
    /// Returns 0 if not yet ready.
    double acceleration() const {
        if (count_ < kWindow) return 0.0;

        // Walk the ring in chronological order starting from the oldest.
        // The oldest element sits at `head_` (the next write slot).
        static constexpr std::array<int, kWindow> c = {5, 0, -3, -4, -3, 0, 5};
        double sum = 0.0;
        for (std::size_t i = 0; i < kWindow; ++i) {
            std::size_t idx = (head_ + i) % kWindow;
            sum += c[i] * buf_pos_[idx];
        }

        // Estimate dt from the timestamps spanning the window.
        std::size_t newest = (head_ + kWindow - 1) % kWindow;
        std::size_t oldest = head_;
        double span_s = static_cast<double>(buf_ts_[newest] - buf_ts_[oldest])
                        * 1e-6;
        if (span_s <= 0.0) return 0.0;
        double dt = span_s / static_cast<double>(kWindow - 1);

        return (sum / 42.0) / (dt * dt);
    }

    /// Timestamp (µs) of the *centre* sample, i.e. the sample the
    /// acceleration value actually refers to.
    int64_t centre_ts_us() const {
        if (count_ < kWindow) return 0;
        std::size_t centre = (head_ + kWindow / 2) % kWindow;
        return buf_ts_[centre];
    }

    void reset() {
        head_ = 0;
        count_ = 0;
    }

private:
    std::array<double, kWindow>  buf_pos_{};
    std::array<int64_t, kWindow> buf_ts_{};
    std::size_t head_  = 0;
    std::size_t count_ = 0;
};
