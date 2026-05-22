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
};

} // namespace driver
