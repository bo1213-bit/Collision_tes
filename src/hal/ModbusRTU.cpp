#include "ModbusRTU.h"
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace hal {

ModbusRTU::ModbusRTU(const std::string& port, int baud, char parity,
                     int data_bits, int stop_bits)
    : ctx_(nullptr)
    , connected_(false)
    , port_(port)
    , baud_(baud)
    , parity_(parity)
    , data_bits_(data_bits)
    , stop_bits_(stop_bits)
{
    ctx_ = modbus_new_rtu(port_.c_str(), baud_, parity_, data_bits_, stop_bits_);
    if (!ctx_) {
        throw std::runtime_error(std::string("modbus_new_rtu failed: ") + modbus_strerror(errno));
    }
}

ModbusRTU::~ModbusRTU()
{
    disconnect();
    if (ctx_) {
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
}

bool ModbusRTU::connect(int slave_addr)
{
    if (connected_) {
        disconnect();
    }
    modbus_set_slave(ctx_, slave_addr);
    if (modbus_connect(ctx_) == -1) {
        return false;
    }
    connected_ = true;
    return true;
}

void ModbusRTU::disconnect()
{
    if (connected_) {
        modbus_close(ctx_);
        connected_ = false;
    }
}

bool ModbusRTU::is_connected() const
{
    return connected_;
}

int ModbusRTU::read_registers(int addr, int count, uint16_t* dest)
{
    if (!connected_) {
        return -1;
    }
    return modbus_read_registers(ctx_, addr, count, dest);
}

} // namespace hal
