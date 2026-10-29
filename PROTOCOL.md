# Wireless IMU 通信协议

本文档描述 ESP32-C3 固件与上位机之间的完整通信协议。所有多字节字段均为 **小端序 (Little-Endian)**。

> 端口与包格式定义见固件 `SOFTWARE/ESP32C3固件/components/wifi_transport/include/wifi_transport.h`，上位机解析见 `SOFTWARE/多IMU数据展示/multi_imu_core.py` 顶部常量。

---

## 1. 网络架构

设备工作在 WiFi STA 模式，与上位机处于同一局域网，使用 4 个独立通道：

| 通道 | 端口 | 协议 | 方向 | 用途 |
|------|------|------|------|------|
| 数据通道 | **UDP 4211** | 二进制 | 设备 → 上位机 | 姿态数据 + 文本消息 |
| 命令通道 | **TCP 4210** | 文本 (`$CMD,...`) | 双向（上位机发起） | 控制命令 |
| 发现通道 | **UDP 4212** | 文本广播 | 设备 → 广播 | 设备上线发现 |
| OTA 通道 | **HTTP 8080** | HTTP POST | 上位机 → 设备 | 固件升级 |

设备同时通过 **mDNS** 广播：服务类型 `_vqf-imu._tcp`，主机名 `vqf-imu-<node_id>`，TXT 记录含 node、tcp/udp 端口、固件版本。

---

## 2. 设备发现

### 2.1 UDP Beacon（端口 4212）

设备连上 WiFi 后周期性向广播地址发送文本信标：

- 含自定义 ID：
  `VQF_DISC,node=<MAC后6位>,id=<设备ID>,ip=<IP>,tcp=4210,udp=4211,ver=<固件版本>`
- 无自定义 ID：
  `VQF_DISC,node=<MAC后6位>,ip=<IP>,tcp=4210,udp=4211,ver=<固件版本>`

上位机监听 UDP 4212 即可发现局域网内所有设备，无需预置 IP。

### 2.2 mDNS

| 项 | 值 |
|----|----|
| 服务类型 | `_vqf-imu._tcp` |
| 主机名 | `vqf-imu-<node_id>.local` |
| 实例名 | `VQF IMU <node_id>` |
| TXT 记录 | `node`、`id`、`tcp`、`udp`、`ver` |

---

## 3. 数据通道（UDP 4211）

每个数据包第一字节为 `type`，区分两种包。

### 3.1 姿态旋转包（type = `0x01`）— 共 12 字节

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | type | 固定 `0x01`（`VQF_PKT_ROTATION`） |
| 1 | 1 | uint8 | flags | bit0 = `REST`（设备静止），bit1 = `REST_SKIP`（静止时跳过发送） |
| 2 | 2 | uint16 | seq | 序列号，递增、溢出回绕，用于丢包检测 |
| 4 | 2 | int16 | quat_w | 四元数 w，Q15 格式 |
| 6 | 2 | int16 | quat_x | 四元数 x，Q15 格式 |
| 8 | 2 | int16 | quat_y | 四元数 y，Q15 格式 |
| 10 | 2 | int16 | quat_z | 四元数 z，Q15 格式 |

**Q15 解码**：`float = int16 / 32767.0`，四元数已归一化（和为 1）。

对应 C 结构体（packed）：

```c
typedef struct __attribute__((packed)) {
    uint8_t  type;     // 0x01
    uint8_t  flags;    // bit0: REST, bit1: REST_SKIP
    uint16_t seq;      // 递增序号（溢出回绕）
    int16_t  quat[4];  // w, x, y, z —— Q15 格式 (val / 32767.0)
} vqf_pkt_rotation_t; // sizeof = 12
```

Python 解析：`struct.unpack("<BBH4h", data[:12])` → `(type, flags, seq, w, x, y, z)`，再各乘 `1/32767.0`。

### 3.2 文本包（type = `0x02`）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 1 | uint8 | type | 固定 `0x02`（`VQF_PKT_TEXT`） |
| 1 | 1 | uint8 | flags | 保留 |
| 2 | 可变 | char[] | text | 文本内容（如磁力计校准参数 `$CAL,PARAMS,...`、电池/状态文本） |

---

## 4. 命令通道（TCP 4210）

上位机建立 TCP 连接后发送文本命令（`\n` 结尾），设备按需回复。命令前缀 `$CMD,`：

| 命令 | 功能 | 设备回复 |
|------|------|----------|
| `$CMD,CAL_START` | 开始磁力计椭球校准 | 校准进度/结果（文本包） |
| `$CMD,CAL_STOP` | 停止校准采集 | — |
| `$CMD,CAL_ERASE` | 擦除 NVS 中保存的校准参数 | — |
| `$CMD,OFF` | 远程关机 | `$CMD,OFF_ACK` 后断电（见已知问题，部分场景需配合 Reset） |
| `$CMD,SET_ID,<id>` | 设置自定义设备 ID（持久化至 NVS） | — |
| `$CMD,GET_ID` | 查询当前设备 ID | — |

TCP keepalive：空闲 10s 后开始探测，每 1s 探测一次，连续 3 次失败判定连接断开。

---

## 5. OTA 固件升级（HTTP 8080）

- **上传固件**：`POST http://<设备IP>:8080/update`，请求体为固件二进制，`Content-Type: application/octet-stream`
- **查询信息**：`GET http://<设备IP>:8080/info`

> ⚠️ 必须用整包 body 上传（如 Python `urllib.request`）。`curl -F` multipart 方式会被判为 "Invalid firmware"。

参考上传脚本：

```python
import urllib.request
url = 'http://<ESP32_IP>:8080/update'
with open('build/vqf_esp32.bin', 'rb') as f:
    req = urllib.request.Request(url, data=f.read())
    req.add_header('Content-Type', 'application/octet-stream')
    urllib.request.urlopen(req, timeout=120)
```

---

## 6. 默认端口汇总

| 用途 | 端口 |
|------|------|
| TCP 命令通道 | 4210 |
| UDP 数据通道 | 4211 |
| UDP 发现通道 | 4212 |
| HTTP OTA | 8080 |
