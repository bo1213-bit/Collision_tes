#include "EncoderDriver.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace driver {

EncoderDriver::EncoderDriver(const Config& cfg)
    : config_(cfg)
    , protocol_(EncoderProtocol::BISSC)
    , mt_bits_(0)
    , st_bits_(0)
    , open_(false)
{}

EncoderDriver::~EncoderDriver()
{
    close();
}

uint32_t EncoderDriver::single_turn_resolution() const
{
    if (st_bits_ == 0 || st_bits_ > 31) return 0;
    return uint32_t(1) << st_bits_;
}

bool EncoderDriver::open()
{
    if (open_) {
        return true;
    }
    try {
        modbus_ = std::make_unique<hal::ModbusRTU>(
            config_.port, config_.baud_rate, config_.parity);
    } catch (const std::exception& e) {
        std::cerr << "ModbusRTU init failed: " << e.what() << "\n";
        return false;
    }

    if (!modbus_->connect(config_.slave_addr)) {
        std::cerr << "modbus connect failed: " << std::strerror(errno) << "\n";
        modbus_.reset();
        return false;
    }

    if (!init()) {
        std::cerr << "EncoderDriver init failed\n";
        modbus_->disconnect();
        modbus_.reset();
        return false;
    }

    open_ = true;
    return true;
}

void EncoderDriver::close()
{
    if (!open_) {
        return;
    }
    modbus_->disconnect();
    modbus_.reset();
    open_ = false;
}

bool EncoderDriver::is_open() const
{
    return open_;
}

bool EncoderDriver::init()
{
    uint16_t regs[3] = {};
    if (modbus_->read_registers(REG_ENC_PROTO, 3, regs) != 3) {
        return false;
    }
    protocol_ = static_cast<EncoderProtocol>(regs[0]);
    mt_bits_  = regs[1];
    st_bits_  = regs[2];
    return true;
}

bool EncoderDriver::read_value(EncoderValue& out)
{
    if (!open_) {
        return false;
    }

    uint16_t tab[4] = {};
    if (modbus_->read_registers(REG_ENC_VALUE, 4, tab) != 4) {
        return false;
    }

    uint64_t raw = 0;
    std::memcpy(&raw, tab, sizeof(raw));

    if (protocol_ == EncoderProtocol::ABZ) {
        out.signed_val   = static_cast<int64_t>(raw);
        out.unsigned_val = 0;
        out.is_unsigned  = false;
    } else {
        if (mt_bits_ == 0) {
            out.signed_val   = static_cast<int64_t>(raw);
            out.unsigned_val = 0;
            out.is_unsigned  = false;
        } else {
            out.signed_val   = 0;
            out.unsigned_val = raw;
            out.is_unsigned  = true;
        }
    }

    return true;
}

} // namespace driver
