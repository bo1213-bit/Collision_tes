#include "DeviceDriver.h"

  #include <cerrno>
  #include <cstring>
  #include <iostream>

  namespace driver {

  // Firmware uses big-endian for all multi-byte fields.
  static inline uint16_t be16(const uint8_t* p) {
      return (static_cast<uint16_t>(p[0]) << 8) | p[1];
  }
  static inline int32_t be32s(const uint8_t* p) {
      uint32_t u = (static_cast<uint32_t>(p[0]) << 24)
                 | (static_cast<uint32_t>(p[1]) << 16)
                 | (static_cast<uint32_t>(p[2]) << 8)
                 | p[3];
      int32_t s;
      std::memcpy(&s, &u, sizeof(s));
      return s;
  }
  static inline void set_be16(uint8_t* p, uint16_t v) {
      p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
      p[1] = static_cast<uint8_t>(v & 0xFF);
  }

  DeviceDriver::DeviceDriver(const Config& cfg)
      : config_(cfg)
      , open_(false)
  {}

  DeviceDriver::~DeviceDriver()
  {
      close();
  }

  bool DeviceDriver::open()
  {
      if (open_) return true;

      port_ = std::make_unique<hal::SerialPort>();
      if (!port_->open(config_.port, config_.baud_rate)) {
          std::cerr << "SerialPort open " << config_.port
                    << " failed: " << std::strerror(errno) << "\n";
          port_.reset();
          return false;
      }

      proto_ = std::make_unique<hal::CustomProtocol>(port_.get());
      open_ = true;
      return true;
  }

  void DeviceDriver::close()
  {
      if (!open_) return;
      proto_.reset();
      port_->close();
      port_.reset();
      open_ = false;
  }

  bool DeviceDriver::is_open() const { return open_; }

  // ---------------------------------------------------------------------------
  // read_adc  —  firmware _cmdId_getAdcValue (0x08)
  // ---------------------------------------------------------------------------
  bool DeviceDriver::read_adc(DeviceADC& out)
  {
      if (!open_) return false;

      if (!proto_->send_command(0x08))
          return false;

      uint8_t buf[64];
      int n = proto_->recv_response(0x08, buf, sizeof(buf), 50);


      for (int i = 0; i < 8; ++i) {
          int32_t sum = be32s(buf + 4 + i * 4);
          out.raw24[i] = sum;

          uint16_t avg16 = static_cast<uint16_t>(sum / 256);
          float mv = static_cast<float>(avg16 - 32768) / 6.5563f;
          out.mv_data[i] = mv;

          out.max_1s[i]    = 0;
          out.min_1s[i]    = 0;
          out.uv_data[i]   = 0;
          out.calib_data[i] = 0;
      }
      return true;
  }

  // ---------------------------------------------------------------------------
  // read_status  —  firmware _cmdId_getIoAll (0x30)
  // ---------------------------------------------------------------------------
  bool DeviceDriver::read_status(DeviceStatus& out)
  {
      if (!open_) return false;

      if (!proto_->send_command(0x30))
          return false;

      uint8_t buf[64];
      int n = proto_->recv_response(0x30, buf, sizeof(buf), 500);
      if (n < 48) {
          std::cerr << "read_status: short response " << n << " bytes\n";
          return false;
      }

      for (int i = 0; i < 8; ++i) out.din[i]    = buf[i];
      for (int i = 0; i < 4; ++i) out.dout[i]   = buf[8 + i];
      for (int i = 0; i < 4; ++i) out.pwm_en[i] = buf[12 + i];
      out.adc_ch_en = 0;
      out.adc_range = 0;
      return true;
  }

  // ---------------------------------------------------------------------------
  // online record mode
  //   start: _cmdId_startOnlineRecord     (0x16)
  //   stop : _cmdId_stopOnlineRecord      (0x17)
  //   data : _cmdId_readOnlineRecordDatas (0x18)
  // ---------------------------------------------------------------------------
  namespace {
  constexpr uint8_t CMD_START_ONLINE = 0x16;
  constexpr uint8_t CMD_STOP_ONLINE  = 0x17;
  constexpr uint8_t CMD_ONLINE_DATA  = 0x18;
  } // namespace

  bool DeviceDriver::start_online_record(uint8_t ch_enable_mask,
                                         uint8_t signal_type,
                                         uint8_t range,
                                         uint16_t sampling_rate)
  {
      if (!open_) return false;

      // payload = msgAdcStartSramRecord 字段部分(去掉头/CRC):
      //   adc_ch_Enable        u8
      //   adcSignalType        u8
      //   adcRange             u8
      //   adcSamplingRate      u16  (BE)
      //   sramSamplingTimeMax  u32  (online 模式忽略,填 0)
      uint8_t payload[9];
      payload[0] = ch_enable_mask;
      payload[1] = signal_type;
      payload[2] = range;
      set_be16(payload + 3, sampling_rate);
      payload[5] = 0;
      payload[6] = 0;
      payload[7] = 0;
      payload[8] = 0;

      std::cerr << "[DBG] start_online_record: sending cmd=0x16, payload=";
      for (int i = 0; i < 9; ++i)
          std::cerr << std::hex << static_cast<int>(payload[i]) << " ";
      std::cerr << std::dec << "\n";

      bool ok = proto_->send_command(CMD_START_ONLINE, payload, sizeof(payload));
      std::cerr << "[DBG] start_online_record: send_command returned " << (ok ? "true" : "false") << "\n";
      return ok;
  }

  bool DeviceDriver::stop_online_record()
  {
      if (!open_) return false;
      return proto_->send_command(CMD_STOP_ONLINE);
  }

  bool DeviceDriver::recv_online_frame(IDeviceDriver::OnlineFrame& out, int timeout_ms)
  {
      if (!open_) return false;

      // online 帧 payload(去掉 5 字节头和 2 字节 CRC)≈ 8014 字节
      static thread_local uint8_t buf[8192];
      int n = proto_->recv_online_response(CMD_ONLINE_DATA, buf, sizeof(buf), timeout_ms);
      if (n <= 0) {
          static int err_cnt = 0;
          if (++err_cnt % 100 == 1)
              std::cerr << "[DBG] recv_online_response returned " << n
                        << " (err#" << err_cnt << ")\n";
          return false;
      }

      // _adcSendBuf 字节布局(已剥离头与 CRC,起点是 dataLen):
      //   [0..1]   dataLen        u16 BE
      //   [2]      adc_ch_Enable
      //   [3]      adcSignalType
      //   [4]      adcRange
      //   [5]      adcStartCh
      //   [6]      trigCh
      //   [7..8]   samplingRate   u16 BE
      //   [9..10]  trigValue      u16 BE
      //   [11..]   data[4000]     u16 BE × 4000
      if (n < 11) return false;

      out.data_len        = be16(buf + 0);
      out.adc_ch_enable   = buf[2];
      out.adc_signal_type = buf[3];
      out.adc_range       = buf[4];
      out.sampling_rate   = be16(buf + 7);

      out.enabled_ch_num = 0;
      for (int i = 0; i < 8; ++i) {
          if (out.adc_ch_enable & (1u << i)) {
              out.enabled_ch_idx[out.enabled_ch_num++] = static_cast<uint8_t>(i);
          }
      }

      uint16_t data_len = out.data_len;
      if (data_len > 4000) data_len = 4000;
      int avail = (n - 11) / 2;
      if (data_len > avail) data_len = static_cast<uint16_t>(avail);

      const uint8_t* p = buf + 11;
      for (uint16_t i = 0; i < data_len; ++i) {
          out.data[i] = be16(p + i * 2);
      }
      out.data_len = data_len;
      return true;
  }

  // --- 以下方法暂未实现,main 流程不依赖 ---
  bool DeviceDriver::read_dac_ch(uint16_t, DeviceDACChannel&)  { return false; }
  bool DeviceDriver::write_dac_ch(uint16_t, const DeviceDACChannel&) { return false; }
  bool DeviceDriver::read_pwm_ch(uint16_t, DevicePWMChannel&)  { return false; }
  bool DeviceDriver::write_pwm_ch(uint16_t, const DevicePWMChannel&) { return false; }
  bool DeviceDriver::write_dout(uint16_t, bool)                 { return false; }
  bool DeviceDriver::write_pwm_enable(uint16_t, bool)           { return false; }
  bool DeviceDriver::write_adc_ch_enable(uint8_t bitmap) {
    if (!open_) return false;

    uint8_t payload[2] = {0, bitmap};  // ch placeholder, data
    if (!proto_->send_command(0x01, payload, 2))
        return false;

    // 固件收到 cmdId=0x01 后调用 fun_msgSetAdcChState() + fun_getAdcParam(),
    // 返回的响应 cmdId=0x02, payload 第一个字节是 adc_ch_Enable
    uint8_t buf[64];
    int n = proto_->recv_response(0x02, buf, sizeof(buf), 500);
    if (n <= 0) {
        std::cerr << "[WARN] write_adc_ch_enable: no response from device\n";
        return false;
    }

    // msgAdcWaveState payload: [0]=adc_ch_Enable, [1]=adcSignalType, [2]=adcRange, ...
    uint8_t actual = buf[0];
    if (actual != bitmap) {
        std::cerr << "[WARN] write_adc_ch_enable: device rejected write. "
                  << "requested=0x" << std::hex << static_cast<int>(bitmap)
                  << " actual=0x" << static_cast<int>(actual)
                  << " (device may not be in Stop mode)\n" << std::dec;
        return false;
    }

    std::cerr << "[INFO] write_adc_ch_enable: 0x"
              << std::hex << static_cast<int>(bitmap) << " written OK\n"
              << std::dec;
    return true;
}
  bool DeviceDriver::write_adc_range(uint8_t)                   { return false; }
  bool DeviceDriver::read_calib(int32_t (&)[8], int32_t (&)[8]) { return false; }
  bool DeviceDriver::write_calib(const int32_t (&)[8], const int32_t (&)[8]) { return false; }

  } // namespace driver
