#pragma once

#include "driver/IEncoderDriver.h"

#include <cstdint>
#include <memory>

/**
 * Protocol-agnostic encoder facade.
 *
 * Owns an IEncoderDriver and delegates all operations to it.
 * To switch protocols, inject a different IEncoderDriver implementation.
 *
 * Usage:
 *   driver::EncoderDriver::Config cfg;
 *   cfg.port       = "/dev/ttyUSB0";
 *   cfg.baud_rate  = 115200;
 *   cfg.parity     = 'N';
 *   cfg.slave_addr = 1;
 *
 *   auto drv = std::make_unique<driver::EncoderDriver>(cfg);
 *   Encoder enc(std::move(drv));
 *   if (!enc.open()) { ... }
 *
 *   int64_t value = 0;
 *   if (enc.read(value)) { ... }
 *   enc.close();
 */
class Encoder {
public:
    explicit Encoder(std::unique_ptr<driver::IEncoderDriver> drv);
    ~Encoder();

    Encoder(const Encoder&)            = delete;
    Encoder& operator=(const Encoder&) = delete;

    bool open();
    void close();
    bool is_open() const;

    // Read the current encoder position.
    // For multi-turn absolute encoders the value is unsigned and cast to int64_t.
    bool read(int64_t& value);

    // Single-turn resolution: 2^st_bits, i.e. counts per revolution.
    // Returns 0 if unknown.
    uint32_t single_turn_resolution() const;

private:
    std::unique_ptr<driver::IEncoderDriver> drv_;
};
