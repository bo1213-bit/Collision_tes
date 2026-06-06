#pragma once

#include <cstdint>
#include <string>
#include <modbus/modbus.h>

namespace hal {

// Thin RAII wrapper around a libmodbus RTU context.
// Owns the modbus_t handle; not copyable.
class ModbusRTU {
public:
    ModbusRTU(const std::string& port, int baud, char parity,
              int data_bits = 8, int stop_bits = 1);
    ~ModbusRTU();

    ModbusRTU(const ModbusRTU&)            = delete;
    ModbusRTU& operator=(const ModbusRTU&) = delete;

    bool connect(int slave_addr);
    void disconnect();
    bool is_connected() const;

    // Returns number of registers read, or -1 on error.
    int read_registers(int addr, int count, uint16_t* dest);
    int read_input_registers(int addr, int count, uint16_t* dest);

    // Write holding registers (fc 06 / fc 16). Returns 1 on success, -1 on error.
    int write_register(int addr, uint16_t value);
    int write_registers(int addr, int count, const uint16_t* src);

    // Coils (fc 01 / fc 05 / fc 15) — returns bits read/written, -1 on error.
    int read_coils(int addr, int count, uint8_t* dest);
    int write_coil(int addr, int value);
    int write_coils(int addr, int count, const uint8_t* src);

    // Discrete inputs (fc 02).
    int read_input_bits(int addr, int count, uint8_t* dest);

private:
    modbus_t*   ctx_;
    bool        connected_;
    std::string port_;
    int         baud_;
    char        parity_;
    int         data_bits_;
    int         stop_bits_;
};

} // namespace hal
