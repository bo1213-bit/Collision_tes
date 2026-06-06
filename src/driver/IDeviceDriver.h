 #pragma once

  #include <cstdint>

  // Firmware register value types — match the STM32 firmware struct layouts.
  // All multi-byte values are big-endian on the Modbus wire; libmodbus
  // converts to native (LE) automatically.

  struct DeviceADC {
      float   mv_data[8];       // adc_mV_Data16 — scaled mV per channel (float, no truncation)
      int16_t max_1s[8];        // 1-second peak
      int16_t min_1s[8];        // 1-second valley
      int32_t raw24[8];         // adcData24 — raw 24-bit ADC code
      int32_t uv_data[8];       // adc_uV_Data24 — microvolt per channel
      int32_t calib_data[8];    // calibrated value (*100000)
  };

  struct DeviceDACChannel {
      uint16_t wave_mode;
      uint16_t volt;
      uint16_t freq;
      uint16_t duty;
      uint16_t bias;
      uint16_t phase;
  };

  struct DevicePWMChannel {
      uint16_t freq;
      uint16_t duty;
      uint16_t pulse_num;
      uint16_t acc_unit;
  };

  struct DeviceStatus {
      uint8_t din[8];
      uint8_t dout[4];
      uint8_t pwm_en[4];
      uint8_t adc_ch_en;
      uint8_t adc_range;
  };

  namespace driver {

  class IDeviceDriver {
  public:
      // Online record mode frame
      struct OnlineFrame {
          uint8_t  adc_ch_enable;
          uint8_t  adc_signal_type;
          uint8_t  adc_range;
          uint16_t sampling_rate;
          uint16_t data_len;
          uint8_t  enabled_ch_num;
          uint8_t  enabled_ch_idx[8];
          uint16_t data[4000];
      };

      virtual ~IDeviceDriver() = default;

      virtual bool open()           = 0;
      virtual void close()          = 0;
      virtual bool is_open() const  = 0;

      // --- read data blocks ---
      virtual bool read_adc(DeviceADC& out)     = 0;
      virtual bool read_status(DeviceStatus& out) = 0;

      // --- online record mode ---
      virtual bool start_online_record(uint8_t ch_enable_mask,
                                       uint8_t signal_type,
                                       uint8_t range,
                                       uint16_t sampling_rate) = 0;
      virtual bool stop_online_record() = 0;
      virtual bool recv_online_frame(OnlineFrame& out, int timeout_ms) = 0;

      // --- per-channel DAC ---
      virtual bool read_dac_ch(uint16_t ch, DeviceDACChannel& out) = 0;
      virtual bool write_dac_ch(uint16_t ch, const DeviceDACChannel& val) = 0;

      // --- per-channel PWM ---
      virtual bool read_pwm_ch(uint16_t ch, DevicePWMChannel& out) = 0;
      virtual bool write_pwm_ch(uint16_t ch, const DevicePWMChannel& val) = 0;

      // --- DOUT coil control ---
      virtual bool write_dout(uint16_t ch, bool on) = 0;
      virtual bool write_pwm_enable(uint16_t ch, bool on) = 0;

      // --- ADC config ---
      virtual bool write_adc_ch_enable(uint8_t bitmap) = 0;
      virtual bool write_adc_range(uint8_t range) = 0;

      // --- calibration ---
      virtual bool read_calib(int32_t (&factor)[8], int32_t (&zero)[8]) = 0;
      virtual bool write_calib(const int32_t (&factor)[8], const int32_t (&zero)[8]) = 0;
  };

  } // namespace driver