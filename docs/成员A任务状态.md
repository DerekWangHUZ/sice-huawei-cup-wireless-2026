# 成员 A 负责部分 — 当前状态

---

## 架构变更（2026-10-14）

**STM32 协处理器已撤除，基站由三块板改为两块板（ESP32-S3 + Artix-7 FPGA）。**

撤除原因不是技术障碍，而是在系统既定架构下 STM32 找不到技术上不可替代的职责（姿态数据的第一落点必然是 S3；告警输出资源被 FPGA 实验板全面覆盖）。完整论证、原工程留档与跨团队影响见 [CHANGELOG.md](CHANGELOG.md)。

---

## 当前状态（2026-10-15 更新）

**主链路"跌倒判定 → 声光报警"两板各自成立，全部经无头验证（无 GUI、无真机）。**

| 板 | 工程路径 | 完成内容 | 验收证据 | 状态 |
|---|---|---|---|---|
| ESP32-S3（基站） | `firmware/base_station` | SoftAP（CSI_FALL_AP/信道6）+ UDP 8888 收包 + 18B 帧解析 + 跌倒判定状态机 + UART 下发报警命令 + 抓拍指令 | `idf.py build` 全量重编 0 error（`Project build complete`） | ✅ 完成 |
| Artix-7（协处理器） | `firmware/coprocessor` | UART 命令接收 + 帧解码（滑动重同步）+ 6 位数码管显示 + 蜂鸣器告警 + LED 映射 + 上电自检（单板可验） | 无头仿真 9 场景全过（`ALL CHECKS PASSED`）；综合/实现/比特流无头完成（`top.bit`，0 error） | ✅ 完成 |

### 离线自测（改判定逻辑后必跑）

判定核心 `fall_core.c` 为纯 C99、零平台依赖，离线测试直接编译同一份源文件运行
（`tools/offline_test/`，宿主编译器为 MATLAB 自带 lcc64）：

- 6 场景全过：静止恒 IDLE / 走动进 WALK / 跌倒躺平 IDLE→WALK→PENDING→CONFIRMED /
  冲击后爬起不误报 / 坏帧全拒 / 垃圾字节后重同步 6/6
- 合成数据同时用于上板发假包（`send_stream.py`），离线测过的帧 = 上板发的帧

### 真机验证记录（2026-10-16，COM4 实板，S3 单板未接 FPGA）

发假包验证，日志与离线自测逐位一致：

- `UART link ready` 正常。此前发现 `uart_driver_install` rx 缓冲传 0 被驱动拒绝
  （legacy 驱动要求 > HW FIFO 128），已修复为 256 并重烧（见 `fpga_link.c`）
- walk 场景：`IDLE→WALK seq=16 mag2=8158290 gyro2=1221414`，与合成数据复算值完全相同
- fall_lying 场景完整复现离线时序：`WALK→PENDING seq=25`（冲击
  mag2=90425590 > 3g²=37748736）→ **1511ms** 后 `PENDING→CONFIRMED seq=99`
  （≥ STILL_WINDOW_MS 1500）→ 下发 `TX cmd=0x01`（ALARM）+ 抓拍指令
- 两场景间 33s 无数据，收到下一帧时判 `WALK→IDLE`（掉线保护
  DATA_TIMEOUT_MS=3000，设计行为，非异常）
- `STATS frames_bad=1`：来自同网另一设备（192.168.4.2）的非协议数据包，
  被正确拒收计数——坏帧防护真机有效
- 结论：真机跳变帧号/时间窗/特征值与离线自测一致，"离线测的就是真机跑的"成立
- **ALARM/CLEAR 下发真机复现**：`TX cmd=0x01 [AA 55 01 00 00 00 01 0D]`、
  `TX cmd=0x02 [AA 55 02 00 00 00 02 0D]`，字节序与契约 §4 逐字节一致
- **bad_frames 真机验证**：两轮共 20 个坏帧（坏校验/坏帧尾交替）全部拒收，
  `frames_bad` 精确 +10/+10，无漏网
- **告警解除语义修正（2026-10-16，重要）**：ALARM_HOLD 定时解除为错误语义
  （人躺地上昏迷时报警自停）——改为**锁存 + 起身解除**：唯一解除判据是
  持续起身活动（`GETUP_GYRO_DPS` 100°/s 持续 `GETUP_HOLD_MS` 400ms），
  数据停摆时告警保持。迭代过程与依据见 CHANGELOG [2026-10-16]。
  离线自测扩为 8 场景（新增"停摆保持"与"起身解除 fall_getup"回归）。
  演示收尾：fall_lying 报警后发 `walk`（人来扶）或 `fall_getup` 解除。

### 代码结构

| 模块 | 文件 | 职责 |
|---|---|---|
| 姿态帧解析 + 跌倒判定 | `main/fall_core.c/.h` | 纯 C99 核心，阈值集中在头文件顶部 |
| CSI 占位 | `main/csi_stub.c/.h` | 接口已留好，`csi_ready()` 恒 false |
| FPGA 链路 | `main/fpga_link.c/.h` | UART1 GPIO17，115200，协议见契约 §4 |
| 主程序 | `main/base_station.c` | SoftAP/收包/判定/下发/日志 |
| FPGA 侧 | `coprocessor/rtl/*.v` | uart_rx / cmd_decoder / seg_display / buzzer_ctrl / top；上电自检 2s（契约 §4.7） |

### FPGA 工程布局

RTL/约束放在 `rtl/`、`constrs/`，不放 Vivado 生成的 `coprocessor.srcs/`
（那是工程生成物，重建脚本 `-force` 会清空）。工程文件
`firmware/coprocessor/coprocessor.xpr` 由 `rebuild.tcl` 原地重建；
比特流用 `build_bit.tcl` 无头生成（已有现成 `top.bit`）。详见
[Vivado工程移植方案.md](Vivado工程移植方案.md)。

### 备注
- **FPGA 板已知硬件缺陷**：`led[15]`（引脚 M1）在本板物理不亮，非工程问题，验收忽略。
- **FPGA 复位坑**：`rst_n`(T9) 须开内部上拉（`PULLUP true`），已在 `pins.xdc` 落实。
- 详细引脚/资源见 `docs/FPGA实验板资料参考.md`。
- 数码管按 6 位实现（位选 `an[0..5]`）；段码极性/位选极性做成模块参数
  （`seg_display.v` 的 `SEG_INVERT`/`AN_ACTIVE_LOW`），上板不符改参数即可。
- 板间协议权威定义见 [团队协作规范与接口契约.md](团队协作规范与接口契约.md) §四。
- **上机操作文档不在仓库**，位于本机下载文件夹《上机操作说明-跌倒报警联调.md》
  （烧录/接线/连热点/发假包/每步现象/排障，仓库结构里不放实操手册）。

---

## 本期明确不做（原因与决策见 CHANGELOG）

- MicroSD 数据记录、OLED 状态显示（模块未到货）
- 真实 CSI 提取（需穿戴端在场，`csi_stub` 占位留接口）
- 穿戴端联调（需成员 B 在场；抓拍指令照发，目标不在只打日志）
