#pragma once

#include <cstdint>

namespace driver {

struct EncoderValue {
    int64_t  signed_val;
    uint64_t unsigned_val;
    bool     is_unsigned;   // true when multi-turn absolute encoder
};

class IEncoderDriver {
public:
    virtual ~IEncoderDriver() = default;

    virtual bool open()           = 0;
    virtual void close()          = 0;
    virtual bool is_open() const  = 0;
    virtual bool read_value(EncoderValue& out) = 0;

    /// Single-turn resolution: 2^single_turn_bits (counts per revolution).
    /// Returns 0 if unknown.
    virtual uint32_t single_turn_resolution() const = 0;
};

} // namespace driver
