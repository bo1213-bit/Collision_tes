#include "CustomProtocol.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace hal {

// CRC16 lookup tables — identical to firmware's crc16.c
static const uint8_t kCrcTableH[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t kCrcTableL[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

CustomProtocol::CustomProtocol(SerialPort* port)
    : port_(port)
{}

uint16_t CustomProtocol::crc16(const uint8_t* data, int len)
{
    uint8_t crcH = 0xFF;
    uint8_t crcL = 0xFF;
    uint8_t idx;

    while (len--) {
        idx  = crcH ^ *data++;
        crcH = crcL ^ kCrcTableH[idx];
        crcL = kCrcTableL[idx];
    }
    return (static_cast<uint16_t>(crcH) << 8) | crcL;
}

bool CustomProtocol::check_crc(const uint8_t* frame, int frame_len)
{
    if (frame_len < 2) return false;

    uint16_t rx_crc = (static_cast<uint16_t>(frame[frame_len - 2]) << 8)
                    | frame[frame_len - 1];
    uint16_t calc = crc16(frame, frame_len - 2);
    return rx_crc == calc;
}

bool CustomProtocol::send_command(uint8_t cmd_id, const uint8_t* tx_data,
                                  int tx_len)
{
    if (!port_ || !port_->is_open()) return false;

    int payload_len = (tx_data && tx_len > 0) ? tx_len : 0;
    uint16_t frame_len = 7 + payload_len; // header(5) + payload + CRC(2)

    std::vector<uint8_t> frame;
    frame.reserve(frame_len);

    // Header
    frame.push_back(static_cast<uint8_t>((MSG_HEAD >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(MSG_HEAD & 0xFF));
    frame.push_back(cmd_id);
    frame.push_back(static_cast<uint8_t>((frame_len >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(frame_len & 0xFF));

    // Payload
    if (payload_len > 0) {
        frame.insert(frame.end(), tx_data, tx_data + payload_len);
    }

    // CRC over everything so far
    uint16_t crc = crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));

    int written = port_->write(frame.data(), frame.size());
    if (written != static_cast<int>(frame.size())) {
        std::cerr << "CustomProtocol: write failed, wrote " << written
                  << " of " << frame.size() << " bytes: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

int CustomProtocol::recv_response(uint8_t cmd_id, uint8_t* rx_buf, int rx_max,
                                    int timeout_ms)
  {
      if (!port_ || !port_->is_open()) return -1;

      // Read 5-byte header
      uint8_t hdr[5];
      int n = port_->read(hdr, 5, timeout_ms);
      if (n < 5) {
          if (n > 0) {
              std::cerr << "CustomProtocol: short header (" << n << " bytes)\n";
          }
          return -1;
      }

      uint16_t head = (static_cast<uint16_t>(hdr[0]) << 8) | hdr[1];
      if (head != MSG_HEAD) {
          std::cerr << "CustomProtocol: bad header 0x"
                    << std::hex << head << std::dec << "\n";
          return -1;
      }

      uint8_t  rx_cmd   = hdr[2];
      uint16_t frame_len = (static_cast<uint16_t>(hdr[3]) << 8) | hdr[4];

      if (frame_len < 7) {
          std::cerr << "CustomProtocol: frame_len too small: "
                    << frame_len << "\n";
          return -1;
      }

      int remaining = frame_len - 5;
      uint8_t tail[8192];
      if (remaining <= 0 || remaining > static_cast<int>(sizeof(tail))) {
          std::cerr << "CustomProtocol: remaining " << remaining
                    << " out of range\n";
          return -1;
      }

      n = port_->read(tail, remaining, timeout_ms);
      if (n < remaining) {
          std::cerr << "CustomProtocol: short tail read " << n
                    << " of " << remaining << "\n";
          return -1;
      }

      // Assemble full frame for CRC check
      uint8_t full[8192 + 5];
      std::memcpy(full, hdr, 5);
      std::memcpy(full + 5, tail, remaining);

      if (!check_crc(full, frame_len)) {
          std::cerr << "CustomProtocol: CRC mismatch\n";
          return -1;
      }

      if (rx_cmd != cmd_id) {
          std::cerr << "CustomProtocol: unexpected cmd_id 0x"
                    << std::hex << static_cast<int>(rx_cmd)
                    << " (expected 0x" << static_cast<int>(cmd_id) << ")"
                    << std::dec << "\n";
          return -1;
      }

      int payload_len = remaining - 2; // exclude CRC
      if (payload_len > rx_max) {
          std::cerr << "CustomProtocol: payload " << payload_len
                    << " > rx_max " << rx_max << "\n";
          return -1;
      }
      std::memcpy(rx_buf, tail, payload_len);
      return payload_len;
  }

int CustomProtocol::recv_online_response(uint8_t cmd_id, uint8_t* rx_buf,
                                         int rx_max, int timeout_ms)
{
    if (!port_ || !port_->is_open()) return -1;

    auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(timeout_ms);

    // ---- Phase 1: hunt for 0x55AA sync word, one byte at a time ----
    uint8_t b;
    int    state = 0;   // 0 = wait for 0x55, 1 = wait for 0xAA

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return -1;

        int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now).count());
        if (remain_ms <= 0) return -1;

        int n = port_->read(&b, 1, remain_ms);
        if (n <= 0) {
            // no byte available right now — brief yield then retry
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        if (state == 0) {
            if (b == 0x55) state = 1;
        } else {
            if (b == 0xAA) break;   // found sync
            state = (b == 0x55) ? 1 : 0;   // 0x55 could start a new pair
        }
    }

    // ---- Phase 2: read the remaining header bytes (cmdId + frameLen) ----
    // We already consumed 0x55 and 0xAA; now read cmdId + frameLenH + frameLenL.
    uint8_t remain_hdr[3];
    int     pos = 0;
    while (pos < 3) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return -1;
        int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now).count());
        if (remain_ms <= 0) return -1;

        int n = port_->read(remain_hdr + pos, 3 - pos, remain_ms);
        if (n > 0) {
            pos += n;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    uint8_t  rx_cmd    = remain_hdr[0];
    uint16_t frame_len = (static_cast<uint16_t>(remain_hdr[1]) << 8)
                       | remain_hdr[2];

    if (frame_len < 7) {
        std::cerr << "CustomProtocol: recv_online bad frame_len "
                  << frame_len << "\n";
        return -1;
    }

    // ---- Phase 3: read payload + CRC ----
    int tail_len = frame_len - 5;  // bytes after header (payload + CRC)
    // Use a dynamically allocated buffer so we tolerate any sane frame_len.
    std::vector<uint8_t> tail(tail_len);
    pos = 0;
    while (pos < tail_len) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return -1;
        int remain_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now).count());
        if (remain_ms <= 0) return -1;

        int n = port_->read(tail.data() + pos, tail_len - pos, remain_ms);
        if (n > 0) {
            pos += n;
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    // ---- Phase 4: assemble and verify CRC ----
    std::vector<uint8_t> full(5 + tail_len);
    full[0] = 0x55;
    full[1] = 0xAA;
    full[2] = rx_cmd;
    full[3] = remain_hdr[1];
    full[4] = remain_hdr[2];
    std::memcpy(full.data() + 5, tail.data(), tail_len);

    if (!check_crc(full.data(), frame_len)) {
        std::cerr << "CustomProtocol: recv_online CRC mismatch\n";
        return -1;
    }

    if (rx_cmd != cmd_id) {
        std::cerr << "CustomProtocol: recv_online unexpected cmd_id 0x"
                  << std::hex << static_cast<int>(rx_cmd)
                  << " (expected 0x" << static_cast<int>(cmd_id) << ")"
                  << std::dec << "\n";
        return -1;
    }

    int payload_len = tail_len - 2;  // exclude CRC
    if (payload_len > rx_max) {
        std::cerr << "CustomProtocol: recv_online payload " << payload_len
                  << " > rx_max " << rx_max << "\n";
        return -1;
    }
    std::memcpy(rx_buf, tail.data(), payload_len);
    return payload_len;
}

bool CustomProtocol::is_open() const
{
    return port_ && port_->is_open();
}

} // namespace hal
