#pragma once

#include "IEncoderDriver.h"
#include "../hal/ModbusRTU.h"

#include <memory>
#include <string>

namespace driver {

enum class EncoderProtocol {
    BISSC    = 0,
    ABZ      = 1,
    TAMAGAWA = 2
};

class EncoderDriver : public IEncoderDriver {
public:
    struct Config {
        std::string port;
        int  baud_rate  = 115200;
        char parity     = 'N';
        int  slave_addr = 1;
    };

    explicit EncoderDriver(const Config& cfg);
    ~EncoderDriver() override;

    bool open()          override;
    void close()         override;
    bool is_open() const override;
    bool read_value(EncoderValue& out) override;

    EncoderProtocol protocol()        const { return protocol_; }
    uint16_t        multi_turn_bits() const { return mt_bits_; }
    uint16_t        single_turn_bits() const { return st_bits_; }
    uint32_t        single_turn_resolution() const override;

private:
    static constexpr int REG_ENC_PROTO = 20;
    static constexpr int REG_MT_BITS   = 21;
    static constexpr int REG_ST_BITS   = 22;
    static constexpr int REG_ENC_VALUE = 100;

    bool init();

    Config                          config_;
    std::unique_ptr<hal::ModbusRTU> modbus_;
    EncoderProtocol                 protocol_;
    uint16_t                        mt_bits_;
    uint16_t                        st_bits_;
    bool                            open_;
};

} // namespace driver
