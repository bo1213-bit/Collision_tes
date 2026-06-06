#include "Encoder.h"

Encoder::Encoder(std::unique_ptr<driver::IEncoderDriver> drv)
    : drv_(std::move(drv))
{}

Encoder::~Encoder()
{
    close();
}

bool Encoder::open()
{
    return drv_->open();
}

void Encoder::close()
{
    drv_->close();
}

bool Encoder::is_open() const
{
    return drv_->is_open();
}

bool Encoder::read(int64_t& value)
{
    if (!drv_->is_open()) {
        return false;
    }
    driver::EncoderValue ev{};
    if (!drv_->read_value(ev)) {
        return false;
    }
    value = ev.is_unsigned ? static_cast<int64_t>(ev.unsigned_val)
                           : ev.signed_val;
    return true;
}

uint32_t Encoder::single_turn_resolution() const
{
    return drv_->single_turn_resolution();
}
