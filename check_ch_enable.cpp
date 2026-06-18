#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "hal/CustomProtocol.h"
#include "hal/SerialPort.h"

int main() {
    hal::SerialPort port;
    if (!port.open("/dev/ttyACM0", 921600)) {
        std::cerr << "Failed to open /dev/ttyACM0\n";
        return 1;
    }

    hal::CustomProtocol proto(&port);

    // Step 1: Enable channels CH0~CH5 (data=0x3F=00111111b)
    uint8_t tx[2] = {1, 0x3F};  // ch=1(占位), data=0x3F
    if (!proto.send_command(0x01, tx, 2)) {
        std::cerr << "enable channels failed\n";
        return 1;
    }
    std::cout << "Sent enable channels command (0x01, data=0x3F)\n\n";

    // Small delay to let firmware apply the setting
    usleep(100000);

    // Step 2: Read back device params to verify
    // cmdId=0x02 getDeviceParam, no payload
    if (!proto.send_command(0x02)) {
        std::cerr << "send_command failed\n";
        return 1;
    }

    uint8_t buf[256];
    int n = proto.recv_response(0x02, buf, sizeof(buf), 500);
    if (n <= 0) {
        std::cerr << "recv_response failed, n=" << n << "\n";
        return 1;
    }

    std::cout << "Response payload length: " << n << " bytes\n";
    std::cout << "Raw payload hex: ";
    for (int i = 0; i < n; ++i) {
        printf("%02X ", buf[i]);
    }
    std::cout << "\n\n";

    // msgAdcWaveState  payload layout (from firmware msgStruct.h):
    // [0]  adc_ch_Enable
    // [1]  adcSignalType
    // [2]  adcRange
    // [3]  trigCh
    // [4..5] adcSamplingRate (BE u16)
    // [6..7] adcTrigValue (BE u16)
    // [8..9] adcFlashRecordIntervalTime (BE u16)
    // [10..13] adcSramRecordMaxTime (BE u32)
    // [14] recentRecordSampleNeedTimsMs
    // [15] sramAdutoTrigEnable

    uint8_t ch_enable = buf[0];
    std::cout << "adc_ch_Enable = 0x" << std::hex << (int)ch_enable << std::dec << " (";
    for (int i = 7; i >= 0; --i) {
        std::cout << ((ch_enable >> i) & 1);
    }
    std::cout << ")\n";

    std::cout << "\nChannel status:\n";
    for (int i = 0; i < 8; ++i) {
        bool enabled = (ch_enable >> i) & 1;
        std::cout << "  CH" << i << ": " << (enabled ? "ENABLED" : "DISABLED") << "\n";
    }

    port.close();
    return 0;
}