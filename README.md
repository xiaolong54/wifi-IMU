<div align="center">

# 🧭 Wireless IMU

### 基于 ESP32-C3 + ICM-20948 的无线惯性测量单元

**简体中文** · 基于 ESP32-C3 + ICM-20948

[![ESP32-C3](https://img.shields.io/badge/MCU-ESP32--C3-E7352D?logo=espressif&logoColor=white)](https://www.espressif.com/en/products/socs/esp32-c3)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4.x-424242?logo=espressif&logoColor=white)](https://github.com/espressif/esp-idf)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Mixed-blue)](#开源协议)
[![Stars](https://img.shields.io/github/stars/xiaolong54/wifi-IMU?style=social)](https://github.com/xiaolong54/wifi-IMU/stargazers)
[![Forks](https://img.shields.io/github/forks/xiaolong54/wifi-IMU?style=social)](https://github.com/xiaolong54/wifi-IMU/network/members)
[![Last Commit](https://img.shields.io/github/last-commit/xiaolong54/wifi-IMU)](https://github.com/xiaolong54/wifi-IMU/commits/main)

**低延迟无线九轴姿态追踪** — 固件本地完成 VQF 四元数融合，经 WiFi 实时推流至上位机，支持多设备并发、磁力计椭球校准与 OTA 无线升级。

<div align="center">
  <img src="图片/实物图1.png" width="42%" alt="实物图 1">
  <img src="图片/实物图2.png" width="42%" alt="实物图 2">
</div>

</div>

---

## 目录

| 章节 | 内容 |
|:-----|:-----|
| [核心规格](#核心规格) | 关键参数速查表 |
| [快速开始](#快速开始) | 三步跑通：配网 → 烧录 → 上位机 |
| [特性](#特性) | 系统能力一览 |
| [项目结构](#项目结构) | 目录树与各模块职责 |
| [硬件](#硬件) | 引脚分配、传感器、PCB、BOM、外壳 |
| [固件](#固件) | 环境搭建、配置项、编译烧录 |
| [上位机](#上位机) | 依赖、启动、功能与按键 |
| [通信协议](#通信协议) | 端口分配与数据通道 |
| [技术参数](#技术参数) | 完整参数表 |
| [已知问题](#已知问题) | 硬件限制与规避说明 |
| [开源协议](#开源协议) | 分目录授权清单 |
| [致谢](#致谢) | 第三方项目与灵感来源 |

---

## 核心规格

<div align="center">

| 🧠 姿态解算 | 📶 传输 | 🔋 续航 | 📡 采样率 |
|:---:|:---:|:---:|:---:|
| **VQF** 九轴融合 | **WiFi** UDP + TCP | **9.4 h** 实测 | **225 Hz** 陀螺/加计<br>**100 Hz** 磁力计 |

</div>

---

## 快速开始

三步即可看到设备在上位机里动起来。

### 第 1 步 · 配置 WiFi

固件的 SSID 与密码通过 `menuconfig` 填写：

```bash
cd SOFTWARE/ESP32C3固件
idf.py menuconfig
# 导航到：VQF WiFi Transport → 填写 WiFi SSID 与 WiFi Password
```

### 第 2 步 · 编译并烧录

```bash
cd SOFTWARE/ESP32C3固件

idf.py build                              # 编译
idf.py -p /dev/ttyUSB0 flash             # 烧录（按实际串口修改）
idf.py -p /dev/ttyUSB0 monitor           # 查看日志
```

> 💡 若需切换传感器类型、输出模式或传输通道，编辑 `SOFTWARE/ESP32C3固件/main/imu_config.h` 即可，无需改动其他文件。

### 第 3 步 · 启动上位机

```bash
cd SOFTWARE/多IMU数据展示
pip install -r requirements.txt
python multi_imu_dashboard.py
```

设备上电后会自动出现在设备列表中，**按 `↑` / `↓` 切换选中设备**。

---

## 特性

| 能力 | 说明 |
|:-----|:-----|
| **九轴姿态解算** | ICM-20948（陀螺仪 + 加速度计 + AK09916 磁力计），225 Hz 采集，VQF 算法在 MCU 端本地实时解算 |
| **无线传输** | WiFi STA 模式；UDP 二进制协议（Q15 紧凑格式）承载数据流，TCP 承载控制命令 |
| **自动设备发现** | mDNS 服务广播 + UDP Beacon 双通道，上位机开机即插即用，无需手动填 IP |
| **多设备管理** | Dashboard 同时连接多台 IMU，实时切换查看 |
| **磁力计校准** | 运行时椭球拟合，参数持久化至 NVS；上位机提供 3D 点云 + 球面覆盖热力图 + 范数曲线实时引导 |
| **OTA 固件升级** | 通过 WiFi HTTP 推送整包固件，无需数据线 |
| **电源管理** | 锂电池供电，ADC 电压监测；电压分箱 + 能量回归融合预测剩余时间；低电量自动关机 |
| **WS2812 状态指示** | WiFi 连接状态、电池电量显示、长按关机动画 |
| **按键交互** | 短按查看电量，长按 3 秒关机 |
| **3D 打印外壳** | 提供 STEP 格式电池仓外壳与连接器外壳 |

---

## 项目结构

```
wifi-IMU/
├── README.md                             # 本文件
├── PROTOCOL.md                           # 通信协议全文（数据包布局 / 命令 / OTA）
│
├── 图片/                                 # 实物图与实测曲线
│
├── 测试数据/                             # 原始实测数据（可复现）
│   └── battery_vbin_B4.csv               # 电池放电分箱实测：电压区间 → 放电时长
│
├── 3D打印文件/                           # STEP 格式外壳模型
│   ├── batteryBoxNew-Body001.step        # 电池仓外壳
│   └── connter-Body.step                 # 连接器外壳
│
├── BOM/                                  # 物料清单
│   └── 淘宝imu+esp32关键元件采购表.xlsx   # 关键元件规格与购买链接
│
├── HARDWARE/                             # 立创 EDA PCB 设计（.epro2）
│   ├── esp32c3信息传输板.epro2            # ESP32-C3 无线传输板
│   └── ICM-20948测试板.epro2              # ICM-20948 传感器测试板
│
└── SOFTWARE/
    ├── ESP32C3固件/                       # ESP32-C3 固件（ESP-IDF 5.x）
    │   ├── main/
    │   │   ├── main.c                    # 主程序
    │   │   ├── imu_config.h              # 传感器 / 输出模式 / 传输通道配置
    │   │   ├── i2c_scanner.c             # I2C 地址扫描工具（独立烧录）
    │   │   ├── CMakeLists.txt
    │   │   └── 引脚描述/                   # 引脚接线速查
    │   │       └── esp32信息传输板引脚描述.md
    │   ├── components/
    │   │   ├── icm20948/                 # ICM-20948 九轴驱动
    │   │   ├── lsm9ds1/                  # LSM9DS1 九轴驱动（可选）
    │   │   ├── mpu9250/                  # MPU9250 六轴驱动（可选）
    │   │   ├── vqf/                      # VQF 姿态融合算法
    │   │   ├── wifi_transport/           # WiFi 传输层（UDP + TCP + mDNS）
    │   │   ├── power_manager/            # 电源管理（电量、按键、自动关机）
    │   │   ├── led_manager/              # WS2812 LED 管理
    │   │   ├── mag_calibration/          # 磁力计椭球校准
    │   │   └── ota_update/               # OTA 无线固件更新
    │   ├── CMakeLists.txt                # project(vqf_esp32)，版本号在此定义
    │   ├── partitions.csv                # OTA 双分区表（4 MB Flash）
    │   └── sdkconfig
    │
    └── 多IMU数据展示/                     # Python 上位机（MIT）
        ├── multi_imu_dashboard.py         # Dashboard 界面（Pygame + OpenGL）
        ├── multi_imu_core.py              # 核心服务（发现、连接、控制 API）
        ├── api_reference.md               # MultiImuService 完整 API 参考
        ├── requirements.txt               # 依赖清单
        └── LICENSE                        # MIT
```

> 📄 各顶层子目录下的 `LICENSE` 为对应协议的全文，见[开源协议](#开源协议)。

### 配套文档

| 文档 | 用途 |
|:-----|:-----|
| [`PROTOCOL.md`](PROTOCOL.md) | 网络架构、Beacon 格式、12 字节旋转包布局、命令清单、OTA 协议、端口汇总 |
| [`api_reference.md`](SOFTWARE/多IMU数据展示/api_reference.md) | `MultiImuService` 数据模型与全部 API 方法 |
| [`esp32信息传输板引脚描述.md`](SOFTWARE/ESP32C3固件/main/引脚描述/esp32信息传输板引脚描述.md) | 实物接线速查 |
| [`测试数据/battery_vbin_B4.csv`](测试数据/battery_vbin_B4.csv) | 电池放电实测原始数据，可复现续航结论 |

---

## 硬件

### 引脚分配

| GPIO | 功能 | 说明 |
|:-----|:-----|:-----|
| IO1 | IMU 中断 | ICM-20948 数据就绪中断 |
| IO2 | 按键检测 | ADC 采样，读取值 × 2 = 按键电压，> 1.5 V 视为按下 |
| IO3 | 电源来源检测 | HIGH = 电池供电，LOW = USB 供电 |
| IO5 | 电池电压检测 | ADC 采样，读取值 × 2 = 电池实际电压 |
| IO6 | I2C SDA | 连接 IMU 传感器 |
| IO7 | I2C SCL | 连接 IMU 传感器 |
| IO10 | 系统使能 | 启动时拉高、关机时拉低（自锁电路控制） |

### 传感器参数

| 项目 | 参数 |
|:-----|:-----|
| IMU | ICM-20948，陀螺仪 ±2000°/s，加速度计 ±8 g |
| 磁力计 | AK09916（ICM-20948 内置），100 Hz 连续测量模式 |
| 采样率 | 陀螺仪 + 加速度计 225 Hz；磁力计 100 Hz |
| 总线 | I2C @ 400 kHz |
| I2C 地址 | ICM-20948 `0x68`（AD0=LOW，`0x69` 为 AD0=HIGH）；AK09916 `0x0C` |

### PCB 设计文件

设计文件为立创 EDA `.epro2` 格式，位于 `HARDWARE/`：

| 板卡 | 内容 |
|:-----|:-----|
| **ESP32-C3 信息传输板** | ESP32-C3、锂电池充放电、WS2812 LED、按键、系统自锁电路 |
| **ICM-20948 测试板** | ICM-20948 九轴传感器，经排针 / 排线与传输板连接 |

### BOM 物料清单

`BOM/淘宝imu+esp32关键元件采购表.xlsx` —— 含所有关键元件的规格说明与淘宝购买链接。

### 3D 打印外壳

| 文件 | 说明 |
|:-----|:-----|
| `batteryBoxNew-Body001.step` | 电池仓主体外壳 |
| `connter-Body.step` | 连接器外壳 |

> ⚠️ **已知硬件问题**：ICM-20948 测试板 PCB 丝印上的 X / Y 轴标注有误，实际轴方向以 ICM-20948 数据手册定义为准，详见[已知问题](#已知问题)。

---

## 固件

### 环境要求

| 依赖 | 版本 |
|:-----|:-----|
| ESP-IDF | v5.4.x（推荐 v5.4.2） |
| Python | 3.8+（上位机使用） |
| 硬件 | ESP32-C3，4 MB Flash |

### 配置

所有关键配置集中在 `SOFTWARE/ESP32C3固件/main/imu_config.h`：

```c
// 传感器选择（当前默认 ICM-20948）
#define IMU_SENSOR_TYPE      IMU_SENSOR_ICM20948   // 可选 LSM9DS1 / MPU9250

// 输出模式
#define OUTPUT_MODE          OUTPUT_MODE_VISUALIZER // 四元数协议（供上位机使用）
// 其他可选：OUTPUT_MODE_RAW / OUTPUT_MODE_MAG_DIAG / OUTPUT_MODE_MAG_CAL

// 传输通道
#define TRANSPORT_MODE       TRANSPORT_MODE_WIFI    // WiFi 无线传输
// 可选 TRANSPORT_MODE_SERIAL（USB 串口输出）

// ICM-20948 读取触发方式
#define ICM_READ_TRIGGER_MODE  ICM_READ_TRIGGER_INT // 中断触发（推荐）

// 磁力计开关（仅对 LSM9DS1 有效，MPU9250 无磁力计）
#define USE_MAGNETOMETER     1

// VQF 时间积分使用实测采样间隔（更精确）
#define VQF_USE_ACTUAL_DT    1
```

WiFi SSID 和密码通过 `menuconfig` 配置：

```bash
cd SOFTWARE/ESP32C3固件
idf.py menuconfig
# 导航到：VQF WiFi Transport → 填写 WiFi SSID 和 WiFi Password
```

### 编译与烧录

```bash
cd SOFTWARE/ESP32C3固件

idf.py build                    # 编译
idf.py -p /dev/ttyUSB0 flash   # 烧录（按实际串口修改端口）
idf.py -p /dev/ttyUSB0 monitor # 查看日志
```

固件版本号在 `SOFTWARE/ESP32C3固件/CMakeLists.txt` 的 `PROJECT_VER` 中定义，经 `APP_VERSION` 宏传给上位机显示。

### I2C 地址扫描

如需确认传感器是否接好，可将 `main.c` 替换为 `i2c_scanner.c` 编译烧录，串口会打印所有 I2C 设备地址。

> ⚠️ 需要先修改 `i2c_scanner.c` 中的引脚定义以匹配实际接线。

---

## 上位机

Python 程序，提供多设备管理、3D 实时姿态可视化与磁力计校准界面。

### 依赖安装

```bash
cd SOFTWARE/多IMU数据展示
pip install -r requirements.txt   # pygame / PyOpenGL / numpy
```

### 启动

```bash
python multi_imu_dashboard.py
```

### 功能概览

| 功能 | 说明 |
|:-----|:-----|
| **自动设备发现** | 监听 UDP Beacon，新设备上线自动出现在设备列表 |
| **3D 姿态显示** | OpenGL 实时渲染 PCB 板模型，根据四元数旋转 |
| **多设备管理** | 支持同时连接多台 IMU，`↑` / `↓` 切换选中设备 |
| **电池监控** | 实时显示电压、电量百分比、预估剩余时间、充电状态 |
| **磁力计校准** | 3D 点云 + 球面覆盖热力图 + 范数曲线，实时引导旋转方向 |
| **OTA 固件升级** | 按 `U` 向选中设备推送 `build/vqf_esp32.bin` |
| **远程关机** | 按 `O` 远程关闭选中设备 |

### 按键速查

| 按键 | 功能 |
|:-----|:-----|
| `↑` / `↓` | 切换选中的 IMU 设备 |
| `C` | 开始磁力计校准 |
| `X` | 退出校准视图 / 停止采集 |
| `E` | 擦除已保存的磁力计校准参数 |
| `O` | 远程关机 |
| `U` | OTA 固件升级 |
| `ESC` | 退出程序 |

### 磁力计校准流程

1. 按 `C` 进入校准可视化界面
2. 缓慢向各个方向旋转设备，尽量覆盖整个球面
3. 界面右侧显示球面覆盖热力图，左下角显示范数曲线
4. 覆盖率和范数稳定性达标后，底部提示"校准完成条件已满足"
5. 按 `X` 停止采集，设备自动拟合椭球并保存校准参数至 NVS
6. 校准参数断电后依然有效

---

## 通信协议

设备通过 WiFi 与上位机建立三通道通信，命令通道另有独立的 HTTP OTA 通道：

| 通道 | 端口 | 方向 | 用途 |
|:-----|:-----:|:-----:|:-----|
| **TCP 命令** | 4210 | 上位机 → 设备 | 文本命令，如 `$CMD,CAL_START`、`$CMD,OFF` |
| **UDP 数据** | 4211 | 设备 → 上位机 | 12 字节 Q15 四元数旋转包 / 文本消息 |
| **UDP 发现** | 4212 | 设备 → 上位机 | 周期性 Beacon 帧，用于自动发现 |
| **HTTP OTA** | 8080 | 上位机 → 设备 | 固件整包上传（`POST /update`） |

TCP keepalive：空闲 10 s 后开始探测，每 1 s 一次，连续 3 次失败判定连接断开。

> 📡 完整规格（数据包字节布局、Q15 编码、命令清单、Beacon 格式、OTA 用法）见 [`PROTOCOL.md`](PROTOCOL.md)。

---

## 技术参数

| 参数 | 值 |
|:-----|:-----|
| 主控 | ESP32-C3（160 MHz RISC-V，4 MB Flash） |
| IMU 传感器 | ICM-20948（9 轴） |
| 采样率 | 225 Hz（陀螺仪 + 加速度计）/ 100 Hz（磁力计） |
| 姿态算法 | VQF（Versatile Quaternion-based Filter） |
| 传输方式 | WiFi STA（UDP 二进制 + TCP 命令） |
| 供电 | 3.7 V 锂电池 / USB |
| 续航 | 实测约 **9.4 小时**（1100 mAh 锂电池，满电 → 3.20 V 自动关机，WiFi 全程在线） |
| 低电量关机阈值 | 3.20 V |
| 电池电压分箱 | 50 箱（4.20 V → 3.20 V，每箱 0.02 V） |
| OTA 端口 | HTTP 8080 |
| 固件版本 | v1.2.5 |
| 上位机版本 | v1.2.0 |

---

## 已知问题

### 1. 电池无法充满至 100%（充电截止电压偏低）

<div align="center">
  <img src="图片/电池放电实测曲线.png" width="100%" alt="电池放电实测曲线">
</div>

> 上图为 1100 mAh 锂电池从满电到自动关机的**实测放电曲线**（原始数据见 [`测试数据/battery_vbin_B4.csv`](测试数据/battery_vbin_B4.csv)），同时用于实证设备续航。

| 项目 | 结论 |
|:-----|:-----|
| **现象** | 充电时电池电压最高只能到 **≈ 4.04 V（电量约 85%）**。图中 4.04–4.20 V（对应电量 87%–99%）这 8 个区间**完全 0 采样、0 分钟**（红色斜线柱），说明充电电路始终无法把电池充到标称 4.20 V，电量百分比因此永远显示不到 100% |
| **原因** | 硬件充电管理 IC 的截止电压设置偏低，属充电电路特性，**非固件问题** |
| **影响** | 不影响正常使用，实际可用容量约为标称的 85% |
| **续航实证** | 从满电（≈ 4.04 V）持续放电至 3.20 V 自动关机、全程 WiFi 在线，累计 **≈ 567 分钟（9.4 小时）**，与技术参数一致 |

### 2. ICM-20948 测试板丝印 XY 轴标注有误

PCB 丝印上 X / Y 轴标注与实际传感器芯片坐标系定义不一致。固件已通过软件轴映射补偿（见 `main.c` 中的 `ICM_MAG_SIGN_*` 宏），使用时机姿态方向正确。若自行设计 PCB，请以 ICM-20948 数据手册中的坐标系定义为准。

---

## 开源协议

本项目采用分目录混合授权，不同类型文件适用不同协议：

| 文件类型 | 路径 | 协议 |
|:---------|:-----|:-----|
| 硬件设计文件（PCB） | `HARDWARE/` | [CERN-OHL-S v2](https://ohwr.org/cern_ohl_s_v2.txt) |
| 3D 打印模型 | `3D打印文件/` | [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) |
| ESP32-C3 固件 | `SOFTWARE/ESP32C3固件/` | [GPL-3.0](https://www.gnu.org/licenses/gpl-3.0.html) |
| Python 上位机 | `SOFTWARE/多IMU数据展示/` | [MIT](https://opensource.org/licenses/MIT) |
| 文档与物料清单 | `README.md`、`BOM/` | [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) |

各组件目录下均已放置对应协议全文：`HARDWARE/LICENSE`、`3D打印文件/LICENSE`、`SOFTWARE/ESP32C3固件/LICENSE`、`SOFTWARE/多IMU数据展示/LICENSE`。

第三方组件沿用其原始协议：

| 组件 | 协议 | 来源 |
|:-----|:-----|:-----|
| VQF 姿态融合算法 | [MIT](https://opensource.org/licenses/MIT) | [github.com/dlaidig/vqf](https://github.com/dlaidig/vqf) |

---

## 致谢

- **[SparkFun Electronics](https://www.sparkfun.com/)** — ICM-20948 分线板的开源 PCB 设计为传感器测试板提供了重要的布局参考（[SparkFun ICM-20948 Breakout](https://www.sparkfun.com/products/15335)）
- **[SlimeVR](https://github.com/SlimeVR/SlimeVR-Tracker-ESP)** — 二进制 UDP 协议与静止优化策略的灵感来源
- **[ESP-IDF](https://github.com/espressif/esp-idf)** — Espressif 提供的优秀嵌入式开发框架
- **立创 EDA** — 国产 PCB 设计工具，本项目硬件设计基于该平台完成
