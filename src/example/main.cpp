// ============================================================================
// 主程序:把"数据采集"和"加速度处理"彻底解耦
//
// 设计要点:
//   1. 每个硬件设备(编码器 / ADC 传感器)各开一个采集线程,只负责"读 + 入队"
//   2. 编码器额外开一个处理线程,跑 Savitzky-Golay 算加速度,结果放原子变量
//   3. main 主循环不再直接读硬件,只从队列/原子变量"消费"最新数据
//   4. 队列用 BoundedQueue(满了丢最旧),所以慢消费永远不会阻塞快采集
//   5. adc_thread 绑定到独立 CPU 核心,避免被其他线程/中断抢占
// ============================================================================

#include "BoundedQueue.h"
#include "Encoder.h"
#include "SavitzkyGolay.h"
#include "Sensor.h"
#include "driver/DeviceDriver.h"
#include "driver/EncoderDriver.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <pthread.h>

// ---------------------------------------------------------------------------
// CPU 亲和性工具: 把当前线程绑定到指定核心
// ---------------------------------------------------------------------------
static bool pin_to_core(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "[WARN] pin_to_core(" << cpu_id << ") failed: "
                  << std::strerror(rc) << "\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 队列里塞的"一帧数据"。带上时间戳,后面如果要做时间对齐就靠它。
// ---------------------------------------------------------------------------
struct EncFrame {
    int64_t pos   = 0;   // 编码器原始计数值
    int64_t ts_us = 0;   // 读到这一帧的时间戳(微秒)
};

struct AdcFrame {
    float   mv[8] = {};  // ADC 8 通道电压(mV)
    int64_t ts_us = 0;
};

// ---------------------------------------------------------------------------
// 全局退出标志:Ctrl+C 触发信号处理函数把它设成 false,所有线程下一轮就退
// std::atomic 保证多线程读写不会出现撕裂
// ---------------------------------------------------------------------------
static std::atomic<bool> g_running{true};

static void on_sigint(int) { g_running = false; }

// 取当前微秒时间戳(steady_clock 单调,不会被系统时间调整影响)
static int64_t now_us() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
               now.time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------
// 采集线程 1:编码器
//   循环职责:读编码器 → 打时间戳 → push 进队列 → sleep 100ms
//   push_drop_oldest 永不阻塞,所以即使下游处理跟不上,这里也照常 100ms 一次
// ---------------------------------------------------------------------------
static void encoder_thread(Encoder* enc, BoundedQueue<EncFrame>* q) {
    while (g_running) {
        int64_t pos = 0;
        if (enc->read(pos)) {
            q->push_drop_oldest({pos, now_us()});
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ---------------------------------------------------------------------------
// 采集线程 2:ADC 传感器 —— 轮询模式（cmdId=0x08 请求-响应）
//   绑定到独立 CPU 核心,全速轮询,不 sleep —— 由 select IO 自然限速
// ---------------------------------------------------------------------------
static void adc_thread(Sensor* sensor, BoundedQueue<AdcFrame>* q, int cpu_id = -1) {
    if (cpu_id >= 0) pin_to_core(cpu_id);
    while (g_running) {
        AdcFrame f;
        if (sensor->read_all(f.mv)) {
            f.ts_us = now_us();
            q->push_drop_oldest(std::move(f));
        }
        // 去掉 sleep: 由 recv_response 的 select() 进行自然阻塞
    }
}

#if 0  // [已废弃] adc_thread_online —— Online Record 推送模式
// ---------------------------------------------------------------------------
// [新] 采集线程 2:ADC 传感器 —— Online Record 推送模式
//
//   流程：start_online_record() → 循环 recv_online_frame() → stop_online_record()
//   下位机主动推送大帧（~8018 字节），帧内 data[4000] 是 interleaved u16 原始 ADC 码，
//   按 enabled_ch_idx[] 顺序交织，需要解交织后 u16→mV 打包 AdcFrame 入队。
// ---------------------------------------------------------------------------
static void adc_thread_online(driver::IDeviceDriver* drv,
                              BoundedQueue<AdcFrame>* q,
                              uint8_t ch_mask,
                              uint16_t sampling_rate) {
    // 1) 启动下位机 online record 模式
    //    signal_type=0(原始 ADC), range=0(默认量程)
    if (!drv->start_online_record(ch_mask, 0, 0, sampling_rate)) {
        std::cerr << "[ERROR] start_online_record failed\n";
        return;
    }
    std::cerr << "[INFO] Online record started, ch_mask=0x"
              << std::hex << static_cast<int>(ch_mask) << std::dec
              << ", rate=" << sampling_rate << " Hz\n";

    // 2) 循环接收帧
    while (g_running) {
        driver::IDeviceDriver::OnlineFrame frame;
        if (!drv->recv_online_frame(frame, 1000)) {
            // 超时或错误，继续尝试
            static int dbg_cnt = 0;
            if (++dbg_cnt % 50 == 1)
                std::cerr << "[DBG] recv_online_frame timeout/error x" << dbg_cnt << "\n";
            continue;
        }

        if (frame.enabled_ch_num == 0) continue;

        // 3) 解交织: data[] 按 enabled_ch_idx[] 交织存放
        //    布局: ch0_s0, ch1_s0, ..., chN-1_s0, ch0_s1, ch1_s1, ...
        uint16_t samples_per_ch = frame.data_len / frame.enabled_ch_num;
        for (uint16_t s = 0; s < samples_per_ch; ++s) {
            AdcFrame f;
            f.ts_us = now_us();
            for (uint8_t ci = 0; ci < frame.enabled_ch_num; ++ci) {
                uint8_t ch = frame.enabled_ch_idx[ci];
                uint16_t raw = frame.data[s * frame.enabled_ch_num + ci];
                // u16 ADC code → mV（和 read_adc 公式一致，但未经过 256 次求和）
                f.mv[ch] = (static_cast<float>(raw) - 32768.0f) / 6.5563f;
            }
            q->push_drop_oldest(std::move(f));
        }
    }

    // 4) 停止 online record
    drv->stop_online_record();
    std::cerr << "[INFO] Online record stopped\n";
}
#endif  // [已废弃] adc_thread_online

// ---------------------------------------------------------------------------
// 处理线程:从编码器队列取数据 → 喂给 Savitzky-Golay → 算加速度 → 发布
//
//   计算结果通过两个原子变量"发布"给 main:
//     - latest_acc       :最新一次算出的加速度(rev/s²)
//     - latest_acc_ready :至少算出过一次(SG 要 7 个点才有第一个结果)
//
//   pop_wait 用 500ms 超时,这样即使队列空着也能定期回到 while 条件检查 g_running
// ---------------------------------------------------------------------------
static void encoder_processing_thread(BoundedQueue<EncFrame>* q,
                                      double counts_per_deg,
                                      std::atomic<double>* latest_acc,
                                      std::atomic<bool>*   latest_acc_ready) {
    SavitzkyGolayAcc sg;
    while (g_running) {
        EncFrame f;
        if (!q->pop_wait(f, std::chrono::milliseconds(500))) continue;

        // feed 返回 true 表示窗口已满,acceleration() 可用
        if (sg.feed(static_cast<double>(f.pos), f.ts_us)) {
            double acc = sg.acceleration() / counts_per_deg;  // counts/s² → deg/s²
            latest_acc->store(acc, std::memory_order_relaxed);
            latest_acc_ready->store(true, std::memory_order_release);
        }
    }
}

// ===========================================================================
int main() {
    // Ctrl+C 优雅退出
    std::signal(SIGINT, on_sigint);

    // --- 编码器配置 ---
    driver::EncoderDriver::Config enc_cfg;
    enc_cfg.port       = "/dev/ttyUSB0";
    enc_cfg.baud_rate  = 115200;
    enc_cfg.parity     = 'N';
    enc_cfg.slave_addr = 1;

    // --- 传感器配置 ---
    driver::DeviceDriver::Config sensor_cfg;
    sensor_cfg.port       = "/dev/ttyACM0";
    sensor_cfg.baud_rate  = 921600;;
    sensor_cfg.parity     = 'N';
    sensor_cfg.slave_addr = 1;

    auto enc    = std::make_unique<Encoder>(
                     std::make_unique<driver::EncoderDriver>(enc_cfg));
    auto sensor = std::make_unique<Sensor>(
                     std::make_unique<driver::DeviceDriver>(sensor_cfg));

    // 两个设备独立打开:一个失败不影响另一个
    bool enc_ok    = enc->open();
    bool sensor_ok = sensor->open();

    if (!enc_ok)    std::cerr << "[WARN] Encoder (" << enc_cfg.port
                              << ") failed to open\n";
    if (!sensor_ok) std::cerr << "[WARN] Sensor (" << sensor_cfg.port
                              << ") failed to open\n";

    if (!enc_ok && !sensor_ok) {
        std::cerr << "No device available, exiting.\n";
        return 1;
    }

    // 打开 CSV 文件记录加速度和速度数据
    std::ofstream csv_file("data_output.csv");
    if (csv_file.is_open()) {
        csv_file << "timestamp,accel1,velocity1,force\n";
    } else {
        std::cerr << "[WARN] Cannot open data_output.csv for writing\n";
    }

    constexpr float adc1_scale = 0.2f;//  0.196f;  

    // ---------------------------------------------------------------------
    // 共享状态:队列 + 用于"发布最新加速度"的原子变量
    // (提前声明,校准阶段也需要用 adc_q)
    // ---------------------------------------------------------------------
    BoundedQueue<EncFrame> enc_q(128);
    BoundedQueue<AdcFrame> adc_q(128);

    std::atomic<double> latest_enc_acc{0.0};
    std::atomic<bool>   latest_enc_acc_ready{false};

    // ---------------------------------------------------------------------
    // ADC 校准:用 cmdId=0x08 轮询 2 秒算 offset
    // ---------------------------------------------------------------------
    float adc1_offset = 0.0f;
    float adc2_offset = 0.0f;

    if (sensor_ok) {
        // 初始化时开启 CH2 和 CH3 通道
        sensor->drv()->write_adc_ch_enable(0x0F);  // CH0+CH1+CH2+CH3

        std::cout << "Calibrating offsets via read_all (2s)...\n";
        auto calib_start = std::chrono::steady_clock::now();
        float sum1 = 0.0f, sum2 = 0.0f;
        int   cnt = 0;
        while (std::chrono::steady_clock::now() - calib_start < std::chrono::seconds(2)) {
            float mv[8];
            if (sensor->read_all(mv)) {
                sum1 += mv[0];
                sum2 += mv[1];
                ++cnt;
            }
        }
        adc1_offset = (cnt > 0) ? sum1 / cnt : 0.0f;
        adc2_offset = (cnt > 0) ? sum2 / cnt : 0.0f;
        std::cout << "ADC1 offset = " << adc1_offset
                  << ", ADC2 offset = " << adc2_offset
                  << " (" << cnt << " samples)\n";
    }

    // 单位换算用:counts → deg(标定值:编码器 40 counts = 1°)
    constexpr double counts_per_deg = 40.0;

    std::cout << "Reading encoder and sensor (print on change only, Ctrl+C to stop)...\n";

    // ---------------------------------------------------------------------
    // 启动所有后台线程
    //   adc_thread 绑定到 CPU 核心 2（采集专用）
    //   main       绑定到 CPU 核心 3（消费专用）
    //   两个核心通过 BoundedQueue 通信，避免缓存行竞争
    // ---------------------------------------------------------------------
    constexpr int ADC_CPU = 2;  // ADC 采集线程跑在 core 2
    constexpr int MAIN_CPU = 3; // main 消费循环跑在 core 3

    pin_to_core(MAIN_CPU);  // 把 main 线程也绑定到专用核心

    std::thread t_enc_read, t_enc_proc, t_adc_read;
    if (enc_ok) {
        t_enc_read = std::thread(encoder_thread, enc.get(), &enc_q);
        t_enc_proc = std::thread(encoder_processing_thread,
                                 &enc_q, counts_per_deg,
                                 &latest_enc_acc, &latest_enc_acc_ready);
    }
    if (sensor_ok) {
        t_adc_read = std::thread(adc_thread, sensor.get(), &adc_q, ADC_CPU);
    }

    // ---------------------------------------------------------------------
    // 主消费循环:不再亲自读硬件,只"看队列 + 看原子变量",按原逻辑打印
    // ---------------------------------------------------------------------
    float  velocity1      = 0.0f;          // ADC1 积分得到的速度
    int64_t prev_adc_ts  = 0;              // 上一次 ADC 采样的时间戳(微秒)

    // 记录 adc_frame[2]/[3] 的原始值到 edge_times.csv
    std::ofstream edge_file("/home/lab/Collision_test/edge_times.csv");
    if (edge_file.is_open()) {
        edge_file << "ts_us,ch2_mv,ch3_mv" << std::endl;
    }

    int loop_cnt = 0;
    while (g_running) {
        // --- a) 从 ADC 队列取数据: 队列空时阻塞等待,不空时立即返回 ---
        AdcFrame adc_frame;
        bool has_adc = false;
        float accel1 = 0.0f;
        float force  = 0.0f;

        if (sensor_ok) {
            // 第一帧阻塞等待,后续帧非阻塞排空
            if (adc_q.pop_wait(adc_frame, std::chrono::milliseconds(500))) {
                do {
                    has_adc = true;

                    float adc1_val = adc_frame.mv[0] - adc1_offset;
                    force          = adc_frame.mv[1] - adc2_offset;
                    accel1         = adc1_val * adc1_scale;

                    // 记录 CH2/CH3 原始电压值到文件，用于离线分析突变时刻
                    if (edge_file.is_open()) {
                        edge_file << adc_frame.ts_us << ","
                                  << adc_frame.mv[2] << ","
                                  << adc_frame.mv[3] << "\n";
                    }

                    // 打印 mv[2] 和时间戳，用于观察采样频率（用 \n 避免 flush 阻塞）
                    std::cout << adc_frame.ts_us << "  ch2=" << adc_frame.mv[2] << "mV\n";

                    if (prev_adc_ts != 0) {
                        float dt_s = (adc_frame.ts_us - prev_adc_ts) / 1.0e6f;
                        velocity1 += accel1 * dt_s;
                    }
                    prev_adc_ts = adc_frame.ts_us;

                    // 写入 CSV
                    if (csv_file.is_open()) {
                        csv_file << adc_frame.ts_us << ","
                                 << accel1 << ","
                                 << velocity1 << ","
                                 << force << "\n";
                    }
                } while (adc_q.pop_wait(adc_frame, std::chrono::milliseconds(0)));
            }
        }


        // --- b) 读取编码器加速度（原子变量，无锁） ---
        bool   enc_acc_ready = latest_enc_acc_ready.load(std::memory_order_acquire);
        double enc_acc       = latest_enc_acc.load(std::memory_order_relaxed);
        (void)enc_acc_ready; (void)enc_acc;
    }

    // ---------------------------------------------------------------------
    // 关闭:先 shutdown 队列让阻塞中的 pop_wait 立刻返回,再 join 所有线程
    // ---------------------------------------------------------------------
    enc_q.shutdown();
    adc_q.shutdown();
    if (t_enc_read.joinable()) t_enc_read.join();
    if (t_enc_proc.joinable()) t_enc_proc.join();
    if (t_adc_read.joinable()) t_adc_read.join();

    if (enc_ok)    enc->close();
    if (sensor_ok) sensor->close();
    return 0;
}
