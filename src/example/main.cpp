#include "Encoder.h"
#include "driver/EncoderDriver.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

int main()
{
    driver::EncoderDriver::Config cfg;
    cfg.port       = "/dev/ttyUSB0";
    cfg.baud_rate  = 115200;
    cfg.parity     = 'N';
    cfg.slave_addr = 1;

    Encoder enc(std::make_unique<driver::EncoderDriver>(cfg));

    if (!enc.open()) {
        std::cerr << "Failed to open encoder on " << cfg.port << "\n";
        return 1;
    }

    std::cout << "Encoder connected. Reading values (Ctrl+C to stop)...\n";

    while (true) {
        int64_t value = 0;
        if (enc.read(value)) {
            std::cout << "Encoder value: " << value << "\n";
        } else {
            std::cerr << "Read error\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    enc.close();
    return 0;
}
