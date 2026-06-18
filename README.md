# Collision Test 项目说明

## 编译

```bash
cd /home/lab/Collision_test/build
cmake ..
cmake --build .
```

编译产物：

| 可执行文件 | 说明 |
|---|---|
| `build/encoder_example` | 主程序，采集编码器 + ADC 传感器数据 |
| `build/check_ch_enable` | 检查/设置 ADC 通道使能状态 |
| `build/encoder_print` | 编码器单独测试程序 |

## 运行与数据采集

### 1. 主程序（采集编码器 + 传感器数据）

```bash
./build/encoder_example
```

按 `Ctrl+C` 停止。运行期间生成以下文件：

| 文件 | 内容 |
|---|---|
| `data_output.csv` | 加速度、速度、力数据（每帧一行） |
| `edge_times.csv` | CH2/CH3 光电传感器原始电压值（ts_us, ch2_mv, ch3_mv） |

### 2. 检查 ADC 通道使能状态

```bash
./build/check_ch_enable
```

## Python 分析脚本

### `analyze_edges.py` — 光电传感器边沿检测与速度计算

从 `edge_times.csv` 中检测 CH2 和 CH3 电压下降沿（遮挡传感器），计算通过速度。

两个传感器间距 3cm，速度 = 0.03m / (CH3 下降沿时间 - CH2 下降沿时间)。

```bash
# 默认阈值（基线 - 300mV）
python3 analyze_edges.py

# 自定义阈值偏移量（mV）
python3 analyze_edges.py /home/lab/Collision_test/edge_times.csv 150
```

**使用流程：**

```bash
./build/encoder_example    # 采集数据，Ctrl+C 停止
python3 analyze_edges.py   # 分析光电传感器边沿，输出速度
```

### `plot_data.py` — 离线绘图

读取 `data_output.csv`，绘制加速度和力的变化曲线，自动缩放至变化区域，保存为 `data_output.png`。

```bash
python3 plot_data.py
```

### `plot_live.py` — 实时绘图

边采集边显示加速度和速度曲线，每秒刷新一次。

```bash
python3 plot_live.py
```

需要和 `encoder_example` 同时运行（两个终端窗口）。

## 完整工作流示例

```bash
# 终端 1：编译
cd /home/lab/Collision_test/build
cmake .. && cmake --build .

# 终端 1：运行主程序采集数据
./build/encoder_example
# ... 进行实验，按 Ctrl+C 停止 ...

# 分析光电传感器数据，输出速度
python3 ../analyze_edges.py

# 终端 2（可选）：实时查看曲线
python3 ../plot_live.py

# 实验结束后（可选）：生成静态图表
python3 ../plot_data.py
```