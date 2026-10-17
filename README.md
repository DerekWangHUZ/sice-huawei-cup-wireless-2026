# 通感一体的异构边缘智算系统

> **电子科技大学信通学院“华为杯”无线通信技术创新赛（校内赛）参赛项目**

---

## 📌 项目概述

本项目面向无线通信与通感一体前沿技术，设计并实现了一套**通感一体的异构边缘智算系统**：
* **空口通感一体**：废除外置独立 Wi-Fi 信标，直接复用移动穿戴端回传 IMU 姿态数据的 UDP 射频报文作为空间多径激励源。同一空中数据流，既承担传感器通信功能，又充当 CSI 空间感知信标。
* **异构边缘计算**：ESP32-S3 担任感知主控与网络中枢，Artix-7 FPGA 负责硬件级快速响应与声光告警，ESP32-CAM 负责事件触发抓拍，实现多层次异构边缘协同。

---

## 📂 仓库目录结构

为了实现团队三人按模块包干、异步协作，本仓库按硬件与职责边界划分为独立子目录：

```text
.
├── firmware/                   # 嵌入式固件工程源码
│   ├── base_station/           # [成员 A] ESP32-S3 核心主控工程 (SoftAP / CSI / SD / OLED)
│   │   ├── main/               #    固件源码（fall_core 判定核心 / fpga_link / csi_stub
│   │   │                       #              / oled_display / sd_log / log_fmt / board_pins）
│   │   └── tools/offline_test/ #    离线自测与上板发假包工具（无板可跑）
│   ├── wearable/               # [成员 B] ESP32-CAM 穿戴标签工程 (MPU6050 / UDP推流 / 抓拍)
│   └── coprocessor/            # [成员 A] Artix-7 FPGA 协处理器工程
│       ├── rtl/                #    RTL 源码（uart_rx / cmd_decoder / seg_display / buzzer_ctrl / top）
│       ├── constrs/            #    引脚约束
│       └── sim/                #    testbench + 无头仿真脚本（run_sim.ps1）
│
├── algorithm/                  # [成员 C] 算法与数据处理
│   └── (离线分析脚本与特征提取逻辑)
│
├── hardware/                   # 硬件工程资料
│   ├── schematics/             # 电气原理图、引脚对应表（基站接线表.md）
│   └── photos/                 # 硬件实物定妆照与佩戴效果图
│
├── docs/                       # 项目协作规范与参赛文档
│   ├── 团队分工与异步协作方案.md  # 三人包干范围、自测手段与交付矩阵
│   ├── 团队协作规范与接口契约.md  # Wi-Fi参数、18字节UDP协议、S3↔FPGA控制帧协议(§四)
│   ├── 成员A任务状态.md          # 成员 A 负责部分的当前状态
│   ├── FPGA实验板资料参考.md     # FPGA 实验板引脚与片上资源约束
│   ├── Vivado工程移植方案.md     # FPGA 工程移植与版本管理方案
│   └── CHANGELOG.md             # 架构级变更记录与决策依据
│
├── data/                       # 标准测试数据集 (样本 CSV)
│
├── .gitignore                  # 忽略构建缓存、Vivado临时文件等
└── README.md                   # 本说明文件
```

---

## 🛠️ 硬件拓扑与网络参数

### 1. 网络配置
* **热点名称 (SSID)**：`CSI_FALL_AP`
* **热点密码**：`12345678`
* **固定信道**：`Channel 6` (2.4GHz)
* **基站 IP (ESP32-S3)**：`192.168.4.1`
* **穿戴端 IP (ESP32-CAM)**：`192.168.4.2`
* **通信端口**：姿态推流 UDP `8888`，抓拍控制 UDP `8889`

### 2. 姿态传输帧格式 (18 字节定长二进制流, 50Hz)
| 字节偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| `Byte 0~1` | 帧头 | `uint16_t` | 固定 `0xAA 0x55` |
| `Byte 2~3` | 序号 | `uint16_t` | `0~65535` 循环自增 |
| `Byte 4~15`| 六轴数据 | `int16_t[6]` | AccX, AccY, AccZ, GyroX, GyroY, GyroZ (小端序) |
| `Byte 16`  | 校验和 | `uint8_t` | Byte 2~15 异或校验 (XOR) |
| `Byte 17`  | 帧尾 | `uint8_t` | 固定 `0x0D` |

### 3. 基站本地外设（OLED / microSD）

| 外设 | 接口 | 引脚（ESP32-S3） | 用途 |
|---|---|---|---|
| OLED 1.3" 128x64 | I2C0 | SDA=GPIO4 / SCL=GPIO5 | 基站状态屏：AP/IP、判定状态、包与帧计数、SD 文件名、告警与 FPGA 链路 |
| microSD 模块 | SPI2 | CS=GPIO10 / SCK=GPIO12 / MOSI=GPIO11 / MISO=GPIO13 | 按契约 §2.1 记录姿态 CSV（`/DATA_001.CSV` 起按卡上已有编号递增） |

全量接线、电平要求与引脚避让清单见
[基站接线表.md](hardware/schematics/基站接线表.md)。
**现场改线只需修改 `firmware/base_station/main/board_pins.h`**（引脚与功能开关集中点）。
CSV 表头与行格式集中在 `firmware/base_station/main/log_fmt.c`，并由宿主机离线自测逐字节断言。

> 烧录、连热点、发假包、每步现象与排障等**上机操作内容按仓库规范不入库**，
> 位于开发者本机手册《上机操作说明-跌倒报警联调.md》。

---

## 🚀 快速上手与协作指南

1. **查阅分工与契约**：在开展具体开发前，请务必详细阅读 `docs/` 目录下的：
   * [团队分工与异步协作方案.md](docs/团队分工与异步协作方案.md)
   * [团队协作规范与接口契约.md](docs/团队协作规范与接口契约.md)
   * [CHANGELOG.md](docs/CHANGELOG.md)（架构变更与决策依据）
   * [Vivado工程移植方案.md](docs/Vivado工程移植方案.md)（FPGA 工程移植与版本管理）
2. **各子目录独立提交**：
   * 成员 A 仅维护 `firmware/base_station` 与 `firmware/coprocessor`；
   * 成员 B 仅维护 `firmware/wearable`；
   * 成员 C 仅维护 `algorithm`、`data` 与 `docs`。
   各人提交互不交叉，天然避免代码冲突。
3. **开机与联调顺序**：
   * 先给基站上电（启动 SoftAP）；
   * 再给穿戴端上电（自动连入热点并开始推流）；
   * 通过基站 OLED 屏显或串口确认状态。
