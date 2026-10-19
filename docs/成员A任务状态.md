# 成员 A 负责部分 — 当前状态

> 本文件描述系统的当前事实。

---

## 一、完成状态

| 板 | 工程路径 | 完成内容 | 状态 |
|---|---|---|---|
| ESP32-S3（基站） | `firmware/base_station` | SoftAP（CSI_FALL_AP/信道6）+ UDP 8888 收包 + 18B 帧解析 + 跌倒判定状态机 + UART 双向链路（下行控制帧 / 上行 UI 帧）+ OLED 五页面 UI + 键盘导航 + 可视化设置（NVS）+ 演示剧本 + microSD CSV 记录 + 抓拍指令 | ✅ 完成 |
| Artix-7（协处理器） | `firmware/coprocessor` | UART 收发 + 帧解码（滑动重同步）+ 4×4 键盘扫描与拨码消抖 + 上行快照/键事件 + 6 位数码管词形显示 + 蜂鸣器运行期可调音/静音 + 12 颗 LED 动效与链路灯 + 显示仲裁 + 上电自检 + 可固化到板载配置 Flash | ✅ 完成 |

**验证方式**：FPGA 无头仿真 `SIM RESULT: PASS`（60 项断言）；Vivado 综合/实现/比特流 `BITSTREAM_OK`、`All user specified timing constraints are met`；S3 `idf.py build` 0 error / 0 warning；离线自测 17 场景全过。

---

## 二、判定逻辑

规则依据契约 §2.2（三段式）：

1. **冲击段**：合加速度 `|a| > IMPACT_THRESH_G`（3.0g）
2. **静止段**：冲击后**持续**静止满 `STILL_WINDOW_MS`（1.5s）
   （角速度低 + 加速度回到重力附近；被运动打断则计时清零重来）
3. **判定**：持续静止段满 → 确认跌倒；
   冲击后 `PENDING_MAX_MS`（4s）内未满足则按非跌倒退出

**告警解除**（两种并存）：

| 方式 | 判据 |
|---|---|
| 自动 | 持续起身活动：角速度 > `GETUP_GYRO_DPS`(100°/s) 保持 `GETUP_HOLD_MS`(400ms) |
| 人工 | OLED 侧按 KEY0 两次（3s 内）确认，调用 `fall_detector_force_clear()` |

**告警锁存语义**：告警不因数据停摆而解除（人昏迷 + 设备故障恰是最需要报警的时刻）。

**姿态门**（`POSTURE_CHECK_ENABLE=0`，默认关闭）：开启后确认前额外要求重力方向与
自校准竖直参考夹角 > 60°（躺平），用于压制"快速坐下"误报。前提是佩戴朝向固定。

阈值与全部可调参数集中在 `main/fall_core.h` 顶部。

---

## 三、代码结构

### 基站（ESP32-S3）

| 模块 | 文件 | 职责 |
|---|---|---|
| 姿态帧解析 + 跌倒判定 | `main/fall_core.c/.h` | 纯 C99 核心，阈值集中在头文件顶部；含人工解除接口 |
| CSI 占位 | `main/csi_stub.c/.h` | 接口已留好，`csi_ready()` 恒 false |
| FPGA 下行链路 | `main/fpga_link.c/.h` | UART1 双向（TX GPIO17 / RX GPIO18），下行命令 + `0x03` 音频配置 + 发送互斥 |
| FPGA 上行接收 | `main/fpga_uplink.c/.h` | UART1 RX 读取 + 快照/键事件投递 + 链路心跳存活 |
| 上行帧解析 | `main/ui_proto.c/.h` | 纯 C99，与 `rtl/cmd_decoder.v` 逐字段对应，离线自测覆盖 |
| 可视化设置 | `main/settings.c/.h` | NVS 持久化 9 项设置（对比度/轮播/遥测周期/提示音/蜂鸣器/音调/出厂/边缘补偿），即改即存即生效 |
| 多页面 UI | `main/ui_app.c/.h` | 页面状态机（P0~P4）、键盘导航、演示剧本任务 |
| 引脚与功能开关 | `main/board_pins.h` | OLED/SD 引脚 + `ENABLE_*` 开关集中点，改线只改这里 |
| OLED 显示 | `main/oled_display.c/.h` | I2C 驱动（SSD1306/SH1106 通吃）+ 五页面渲染 + 滚动绘制 + 边缘补偿 |
| SD 记录 | `main/sd_log.c/.h` | SPI2+FATFS 挂载、文件编号递增、环形缓冲 + 记录任务、在位探活 |
| CSV 格式 | `main/log_fmt.c/.h` | 纯 C99，契约 §2.1 表头与行格式（宿主机离线可测） |
| 主程序 | `main/base_station.c` | SoftAP/收包/判定/下发/日志 + 状态快照访问器 + 演示期显示所有权仲裁 |

### 协处理器（Artix-7）

| 模块 | 文件 | 职责 |
|---|---|---|
| UART 收发 | `rtl/uart_rx.v` / `rtl/uart_tx.v` | 115200 8N1，收发各一 |
| 命令解码 | `rtl/cmd_decoder.v` | 滑动窗口重同步；`0x01/0x02/0x03/0x10` 命令；`ever_linked` 链路标志 |
| 人机输入与上行 | `rtl/input_ctrl.v` | 4×4 键盘扫描 + 20ms 消抖 + 拨码消抖 + 演示场景本地控制 + 上行组帧 |
| 数码管 | `rtl/seg_display.v` | 词形显示（`HELLO`/`rEAdy`/`run`/`FALL`/`N.NNG`）+ 老化看门狗 + 自检画面 |
| 蜂鸣器 | `rtl/buzzer_ctrl.v` | 载波 × 三短一停节拍；运行期静音/调音；待确认提示音；试听 |
| LED 效果 | `rtl/led_ctrl.v` | 12 路软 PWM：呼吸/流水/KITT/爆闪 + 链路灯 |
| 顶层 | `rtl/top.v` | 显示仲裁（真实告警 / POST / 演示）、禁用 LED 掩码、上行接线 |

---

## 四、工程布局

RTL/约束放在 `rtl/`、`constrs/`，不放 Vivado 生成的 `coprocessor.srcs/`
（工程生成物，重建脚本 `-force` 会清空）。工程文件
`firmware/coprocessor/coprocessor.xpr` 由 `rebuild.tcl` 原地重建；
比特流用 `build_bit.tcl` 无头生成，`gen_mcs.tcl` 转 mcs，
`program_mcs.tcl` 烧进配置 Flash。详见 [Vivado工程移植方案.md](Vivado工程移植方案.md)。

---

## 五、备注

- **FPGA 板 LED 约定**：板上共 17 颗（丝印 LED0~LED16）；LED12~16 按项目要求禁用，
  工程只驱动主排 `led[0..11]` 共 12 颗。丝印 LED16 无课程引脚映射（不亮是"没接"
  而非"坏了"），详见 [FPGA实验板资料参考.md](FPGA实验板资料参考.md) §3。
- **人机输入**：4×4 矩阵键盘（行扫描 + 列上拉 + 20ms 消抖）与拨码 SW1/SW2，
  键位与开关功能表见契约 §4.9 与 [开关按键使用说明书.md](开关按键使用说明书.md)。
- **FPGA 复位**：`rst_n`(T9) 须开内部上拉（`PULLUP true`），已在 `pins.xdc` 落实。
- **上电启动**：该板需上电后按一次 CONFIG 按钮才会从 Flash 启动，详见
  [FPGA实验板资料参考.md](FPGA实验板资料参考.md) §9b。
- 数码管按 6 位实现（位选 `an[0..5]`）；段码极性/位选极性做成模块参数
  （`seg_display.v` 的 `SEG_INVERT`/`AN_ACTIVE_LOW`），上板不符改参数即可。
- 板间协议（下行 + 上行）权威定义见 [团队协作规范与接口契约.md](团队协作规范与接口契约.md) §四。
- **上机操作文档按仓库规范不入库**（烧录/接线/连热点/发假包/每步现象/排障），
  由开发者自行保管，仓库内不引用。

---

## 六、范围外事项

- 真实 CSI 提取（需穿戴端在场，`csi_stub` 占位留接口；CSV 的 `CSI_Variance` 列恒 0）
- SD 运行中热插拔的完整重挂载（六脚模块无卡检测脚，靠扇区探活发现拔卡）
