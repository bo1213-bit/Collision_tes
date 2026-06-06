#pragma once

#include <cstdint>
#include <string>

namespace hal {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port, int baud);
    void close();
    bool is_open() const;

    int write(const uint8_t* data, int len);
    int read(uint8_t* buf, int len, int timeout_ms);

private:
    int fd_ = -1;
};

} // namespace hal
