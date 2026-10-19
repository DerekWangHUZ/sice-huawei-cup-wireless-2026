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
│   ├── base_station/           # [成员 A] ESP32-S3 核心主控工程 (SoftAP / CSI / SD / OLED 多页面)
│   │   ├── main/               #    固件源码（fall_core 判定核心 / fpga_link+fpga_uplink 双向链路
│   │   │                       #              / ui_proto / ui_app 多页面 / settings / oled_display
│   │   │                       #              / csi_stub / sd_log / log_fmt / board_pins）
│   │   └── tools/offline_test/ #    离线自测与上板发假包工具（无板可跑）
│   ├── wearable/               # [成员 B] ESP32-CAM 穿戴标签工程 (MPU6050 / UDP推流 / 抓拍)
│   │   ├── main/wearable_firmware/ #  Arduino 草图（18B 推流 / UDP 8889 抓拍 / 网页与 /pic）
│   │   └── tools/udp_receiver.py   #  电脑端 18 字节协议接收与校验
│   └── coprocessor/            # [成员 A] Artix-7 FPGA 协处理器工程
│       ├── rtl/                #    RTL 源码（uart_rx/uart_tx / cmd_decoder / input_ctrl 键盘拨码
│       │                       #              / seg_display / buzzer_ctrl / led_ctrl / top）
│       ├── constrs/            #    引脚约束
│       ├── sim/                #    testbench + 无头仿真脚本（run_sim.ps1）
│       ├── program_bit.tcl     #    JTAG 下载比特流（掉电即失，改完立刻看现象用）
│       └── program_mcs.tcl     #    固化到板载配置 Flash（掉电不丢；本板上电后需按一次 CONFIG）
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
│   ├── 团队协作规范与接口契约.md  # Wi-Fi参数、18字节UDP协议、S3↔FPGA双向帧协议(§四)
│   ├── 开关按键使用说明书.md      # 面向现场操作者：拨码/键盘/OLED 五页/告警解除/排障
│   ├── 成员A任务状态.md          # 成员 A 负责部分的当前状态
│   ├── FPGA实验板资料参考.md     # FPGA 实验板引脚与片上资源约束
│   └── Vivado工程移植方案.md     # FPGA 工程移植与版本管理方案
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
* **通信端口**：姿态推流 UDP `8888`，抓拍控制 UDP `8889`，照片页 `http://192.168.4.2/pic`

### 2. 姿态传输帧格式 (18 字节定长二进制流, 50Hz)
| 字节偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| `Byte 0~1` | 帧头 | `uint16_t` | 固定 `0xAA 0x55` |
| `Byte 2~3` | 序号 | `uint16_t` | `0~65535` 循环自增 |
| `Byte 4~15`| 六轴数据 | `int16_t[6]` | AccX, AccY, AccZ, GyroX, GyroY, GyroZ (小端序) |
| `Byte 16`  | 校验和 | `uint8_t` | Byte 2~15 异或校验 (XOR) |
| `Byte 17`  | 帧尾 | `uint8_t` | 固定 `0x0D` |

### 3. 穿戴端能力（ESP32-CAM）

| 能力 | 说明 |
|---|---|
| 姿态推流 | MPU 六轴原始值以 50 Hz 打成 18 字节帧发往基站 `192.168.4.1:8888/UDP` |
| 抓拍命令 | 监听 `192.168.4.2:8889/UDP`，校验 4 字节 `55 AA 01 FF` |
| 异步抓拍 | 独立工作任务取图并存入 PSRAM，主循环继续 50 Hz 采样，不因抓拍阻塞 |
| 补光与闪灯 | 取图前点亮板载闪光灯预热 150 ms，约 1 s 后自动关闭 |
| 照片查看 | `http://192.168.4.2/pic` 返回最近一次触发的照片；未抓拍时 404 |
| 摄像头网页 | `http://192.168.4.2/` 提供实时视频流与相机参数调节 |

穿戴端以静态地址 `192.168.4.2` 接入基站热点；基站 DHCP 地址池为
`192.168.4.10 ~ 192.168.4.100`，该地址不会被分配给其它设备。
抓拍命令报文头 `55 AA` 与上行姿态帧头 `AA 55` 方向相反，不要混淆。
详见 [firmware/wearable/README.md](firmware/wearable/README.md)。

### 4. 基站本地外设（OLED / microSD / FPGA 人机输入）

| 外设 | 接口 | 引脚（ESP32-S3 / FPGA） | 用途 |
|---|---|---|---|
| OLED 1.3" 128x64 | I2C0 | SDA=GPIO4 / SCL=GPIO5 | **多页面 UI**：P0 状态（中文大字 + 告警冲击幅值定格）、P1 遥测（幅值/三轴/帧率/条形图）、P2 系统（网络/运行时长/SD/链路/判定参数）、P3 设置（9 项可视化调节，NVS 持久化）、P4 演示（场景名 + 预期声光表现 + 剧本进度）；页面优先级：告警 > 演示 > 锁定 > 轮播 |
| microSD 模块 | SPI2 | CS=GPIO10 / SCK=GPIO12 / MOSI=GPIO11 / MISO=GPIO13 | 按契约 §2.1 记录姿态 CSV（`/DATA_001.CSV` 起按卡上已有编号递增） |
| 4×4 矩阵键盘 | FPGA 板载 | 行 K3/M6/P10/R10，列 R12/T12/R11/T10 | 页面切换（1~5）、设置导航（A/B/C/D）、演示场景选择（0~7）；扫描+消抖在 FPGA，事件经上行 UART 报 S3 |
| 拨码开关 sw[1]/sw[2] | FPGA 板载 | T8 / T7 | sw[1]=演示模式总开关；sw[2]=本地硬静音（含上电自检） |

全量接线、电平要求与引脚避让清单见
[基站接线表.md](hardware/schematics/基站接线表.md)。
**现场改线只需修改 `firmware/base_station/main/board_pins.h` 与
`firmware/coprocessor/constrs/pins.xdc`**（两侧引脚集中点）。
CSV 表头与行格式集中在 `firmware/base_station/main/log_fmt.c`，并由宿主机离线自测逐字节断言。

FPGA 声光输出（6 位数码管整屏英文词 `HELLO/rEAdy/run/FALL` + `N.NNG` 数值、
蜂鸣器运行期可调音/静音、12 颗 LED 动效 + 链路灯，LED12~16 禁用）
的完整状态定义见 [团队协作规范与接口契约.md](docs/团队协作规范与接口契约.md) §4.8；
板间为**双向 UART**（下行控制帧 + 上行 UI 帧），协议见同文档 §4。

---

## 🚀 快速上手与协作指南

1. **查阅分工与契约**：在开展具体开发前，请务必详细阅读 `docs/` 目录下的：
   * [团队分工与异步协作方案.md](docs/团队分工与异步协作方案.md)
   * [团队协作规范与接口契约.md](docs/团队协作规范与接口契约.md)
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
