#pragma once

#include "driver/IDeviceDriver.h"

#include <cstdint>
#include <memory>

/**
 * ADC sensor facade.
 *
 * Owns an IDeviceDriver and delegates ADC reads to it.
 * Follows the same pattern as Encoder — inject the driver, then call
 * simple read methods without dealing with Modbus details.
 *
 * Usage:
 *   driver::DeviceDriver::Config cfg;
 *   cfg.port       = "/dev/ttyUSB0";
 *   cfg.baud_rate  = 115200;
 *   cfg.parity     = 'N';
 *   cfg.slave_addr = 1;
 *
 *   auto drv = std::make_unique<driver::DeviceDriver>(cfg);
 *   Sensor sensor(std::move(drv));
 *   if (!sensor.open()) { ... }
 *
 *   float mv = 0;
 *   if (sensor.read_ch(0, mv)) { ... }
 *   sensor.close();
 */
class Sensor {
public:
    explicit Sensor(std::unique_ptr<driver::IDeviceDriver> drv);
    ~Sensor();

    Sensor(const Sensor&)            = delete;
    Sensor& operator=(const Sensor&) = delete;

    bool open();
    void close();
    bool is_open() const;

    // Read ADC channel (0-7), returns millivolt value.
    bool read_ch(int ch, float& mv);

    // Read ADC channel (0-7), returns raw 24-bit ADC code.
    bool read_raw(int ch, int32_t& raw);

    // Read all 8 channels' mV values at once.
    bool read_all(float (&mv)[8]);

    // Read 1-second peak/valley for a channel (mV).
    bool read_peak(int ch, float& mv);
    bool read_valley(int ch, float& mv);

    // Access the underlying driver for config (e.g., set channel enable).
    driver::IDeviceDriver* drv() { return drv_.get(); }

private:
    std::unique_ptr<driver::IDeviceDriver> drv_;
};
