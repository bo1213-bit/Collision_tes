#pragma once

#include "IDeviceDriver.h"
#include "../hal/CustomProtocol.h"
#include "../hal/SerialPort.h"

#include <memory>
#include <string>

namespace driver {

class DeviceDriver : public IDeviceDriver {
public:
    struct Config {
        std::string port;
        int  baud_rate  = 115200;
        char parity     = 'N';
        int  slave_addr = 1;
    };

    explicit DeviceDriver(const Config& cfg);
    ~DeviceDriver() override;

    bool open()          override;
    void close()         override;
    bool is_open() const override;

    bool read_adc(DeviceADC& out)        override;
    bool read_status(DeviceStatus& out)  override;

    // --- online record mode ---
    bool start_online_record(uint8_t ch_enable_mask,
                             uint8_t signal_type,
                             uint8_t range,
                             uint16_t sampling_rate) override;
    bool stop_online_record() override;
    bool recv_online_frame(IDeviceDriver::OnlineFrame& out, int timeout_ms) override;
    
    // Not implemented yet — not needed by main loop.
    bool read_dac_ch(uint16_t, DeviceDACChannel&) override;
    bool write_dac_ch(uint16_t, const DeviceDACChannel&) override;
    bool read_pwm_ch(uint16_t, DevicePWMChannel&) override;
    bool write_pwm_ch(uint16_t, const DevicePWMChannel&) override;
    bool write_dout(uint16_t, bool) override;
    bool write_pwm_enable(uint16_t, bool) override;
    bool write_adc_ch_enable(uint8_t) override;
    bool write_adc_range(uint8_t) override;
    bool read_calib(int32_t (&)[8], int32_t (&)[8]) override;
    bool write_calib(const int32_t (&)[8], const int32_t (&)[8]) override;

private:
    Config                           config_;
    std::unique_ptr<hal::SerialPort>     port_;
    std::unique_ptr<hal::CustomProtocol> proto_;
    bool                             open_;
};

} // namespace driver
