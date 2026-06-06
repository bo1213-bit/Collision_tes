#pragma once

#include "SerialPort.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace hal {

// Implements the firmware's custom binary protocol over a SerialPort.
//
// Frame format (big-endian for multi-byte fields):
//   [msgHead:2B = 0x55AA][cmdId:1B][frameLen:2B][payload...][CRC16:2B]
//
// frameLen is the total frame length including CRC.
class CustomProtocol {
public:
    explicit CustomProtocol(SerialPort* port);
    ~CustomProtocol() = default;

    CustomProtocol(const CustomProtocol&)            = delete;
    CustomProtocol& operator=(const CustomProtocol&) = delete;

    // Send a command frame. If tx_data is nullptr or tx_len is 0, sends a
    // minimal frame (no payload, frameLen=7).
    bool send_command(uint8_t cmd_id, const uint8_t* tx_data = nullptr,
                      int tx_len = 0);

    // Receive one response frame. Verifies msgHead==0x55AA, CRC, and cmd_id
    // match. Returns payload length on success, -1 on error.
    // rx_buf receives the payload only (without header or CRC).
    int recv_response(uint8_t cmd_id, uint8_t* rx_buf, int rx_max,
                      int timeout_ms);

    // Specialised receiver for online-record push-mode frames.
    // Unlike recv_response():
    //  - Hunts byte-by-byte for 0x55AA sync (recovers from desync)
    //  - Retries on partial reads instead of giving up on EAGAIN
    //  - Uses total elapsed time, not a single read deadline
    int recv_online_response(uint8_t cmd_id, uint8_t* rx_buf, int rx_max,
                             int timeout_ms);

    // Receive all response frames that match cmd_id until timeout.
    // Each frame's payload is appended to rx_buf. Returns total bytes
    // appended, or -1 if no frame received.
    int recv_all_responses(uint8_t cmd_id, uint8_t* rx_buf, int rx_max,
                           int timeout_ms);

    bool is_open() const;

private:
    static constexpr uint16_t MSG_HEAD = 0x55AA;

    uint16_t crc16(const uint8_t* data, int len);
    bool    check_crc(const uint8_t* frame, int frame_len);

    SerialPort* port_;
    std::vector<uint8_t> rx_buf_;
};

} // namespace hal
