#include "Sensor.h"

Sensor::Sensor(std::unique_ptr<driver::IDeviceDriver> drv)
    : drv_(std::move(drv))
{}

Sensor::~Sensor()
{
    close();
}

bool Sensor::open()
{
    return drv_->open();
}

void Sensor::close()
{
    drv_->close();
}

bool Sensor::is_open() const
{
    return drv_->is_open();
}

bool Sensor::read_ch(int ch, float& mv)
{
    if (!drv_->is_open() || ch < 0 || ch > 7) {
        return false;
    }
    DeviceADC adc{};
    if (!drv_->read_adc(adc)) {
        return false;
    }
    mv = static_cast<float>(adc.mv_data[ch]);
    return true;
}

bool Sensor::read_raw(int ch, int32_t& raw)
{
    if (!drv_->is_open() || ch < 0 || ch > 7) {
        return false;
    }
    DeviceADC adc{};
    if (!drv_->read_adc(adc)) {
        return false;
    }
    raw = adc.raw24[ch];
    return true;
}

bool Sensor::read_all(float (&mv)[8])
{
    if (!drv_->is_open()) {
        return false;
    }
    DeviceADC adc{};
    if (!drv_->read_adc(adc)) {
        return false;
    }
    for (int i = 0; i < 8; ++i) {
        mv[i] = static_cast<float>(adc.mv_data[i]);
    }
    return true;
}

bool Sensor::read_peak(int ch, float& mv)
{
    if (!drv_->is_open() || ch < 0 || ch > 7) {
        return false;
    }
    DeviceADC adc{};
    if (!drv_->read_adc(adc)) {
        return false;
    }
    mv = static_cast<float>(adc.max_1s[ch]);
    return true;
}

bool Sensor::read_valley(int ch, float& mv)
{
    if (!drv_->is_open() || ch < 0 || ch > 7) {
        return false;
    }
    DeviceADC adc{};
    if (!drv_->read_adc(adc)) {
        return false;
    }
    mv = static_cast<float>(adc.min_1s[ch]);
    return true;
}
