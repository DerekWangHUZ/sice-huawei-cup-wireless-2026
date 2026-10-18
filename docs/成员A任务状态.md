# 成员 A 负责部分 — 当前状态

---

## 2026-10-18 判定逻辑修订（对照文献评审）

对照业界同类实现（KFall 数据集 ESP32+SVM 项目、Kepski & Kwolek 2017 等）
评审判定算法，落地两项修订 + 一项可选钩子（依据与论证见
[CHANGELOG.md](CHANGELOG.md) 同日条目）：

| 项 | 内容 |
|---|---|
| P0 确认逻辑修订 | PENDING 改"持续静止计时 + PENDING_MAX_MS(4s) 观察上限"，修复快照式确认的两条漏报路径（到期帧恰逢抽动 / 静止晚于到期才出现） |
| P1 姿态门（默认关） | `POSTURE_CHECK_ENABLE=0`：确认前检查重力方向与自校准竖直参考夹角 >60°，压"快速坐下"误报；未校准 fail-open；**待佩戴位置定死后开启** |
| P2 阈值依据 | `IMPACT_THRESH_G` 保持 3.0g（压误报优先），注释补文献依据（学术 2.5g / KFall 3.0g→recall 0.827）；调参建议交成员 C（SD 卡 CSV 阈值扫描） |
| 回归 | 离线自测 11 -> **14 场景全过**：新增 fall_late_still / fall_boundary_twitch（旧逻辑漏报，新逻辑确认）、bump_walk_sit（观察上限防误报）；fall_recover 数据延长至 4s 上限到期；fall_lying 确认时刻 2020ms -> 2040ms（+1 帧，迁移序列不变） |
| 待办 | 真机回归（用 `send_stream.py` 同一批合成帧重推）；成员 B 定佩戴位置后评估开启姿态门 |

---

## 2026-10-17 新增：OLED 状态显示 + microSD 数据记录

1.3" 四脚 I2C OLED 与六脚 microSD 模块到货，已接入基站固件。
**代码：编译 + 离线验证通过；上板：OLED 上线、四幕联调通过、SD 记录打通**（见下方真机记录）。

| 项 | 内容 |
|---|---|
| 新增源文件 | `main/board_pins.h`（引脚与开关集中点）、`main/oled_display.c/.h`、`main/sd_log.c/.h`、`main/log_fmt.c/.h` |
| OLED | I2C0（SDA=GPIO4 / SCL=GPIO5），128x64，**中文 16x16 + ASCII 6x8 点位图状态屏**（2026-10-18 升级为中文大字，见下节）；地址自动探测 0x3C/0x3D；**SSD1306 与 SH1106 通吃**（页寻址），型号用 `OLED_CTRL_SH1106` 宏切换 |
| microSD | SPI2（CS=GPIO10 / SCK=GPIO12 / MOSI=GPIO11 / MISO=GPIO13），FATFS，文件名 `DATA_001.CSV` 起按卡上已有编号递增 |
| CSV 格式 | 严格契约 §2.1，表头与行格式集中在 `log_fmt.c`，被离线自测逐字节断言 |
| 实时性 | 写卡走**独立任务 + 环形缓冲（256 项）**：收包任务只做非阻塞入队，绝不因写卡打断 100ms 收包/判定节拍 |
| 容错 | SD 挂载失败 / OLED 无应答 / 写卡中途失败 —— 一律只禁用该功能，**绝不影响跌倒判定与报警主链路**（真机已确认） |
| 验收证据 | `idf.py fullclean` 后全量重编 **0 error / 编译告警 0**（1085/1085 步，退出码 0）；离线自测 **11/11 全过**（原 8 场景无回归 + 新增 3 个 CSV 契约场景） |
| 体积 | `base_station.bin` = 928,320 B (0xE2A40)，按工程配置的 1M app 分区余量 120,256 B (11%)；**实板 flash 为 16MB（见下），余量其实很宽裕** |
| 引脚对应表 | [hardware/schematics/基站接线表.md](../hardware/schematics/基站接线表.md)（接线/电平/引脚避让，不含上机步骤） |

### 真机验证记录（2026-10-18，COM4）

串口为板上 `USB-Enhanced-SERIAL CH343` 桥（COM4）。

**① 板卡事实：flash 实际 16MB**

启动日志：`spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k)`。
工程按 2MB 配置、实际芯片 16MB —— 当前够用，**将来要扩 app 分区/加文件系统时有 8 倍余量**，
不必再迁就 1M 分区。

**② OLED：I2C 与初始化通过**

`OLED: panel ready: addr=0x3C ctrl=SSD1306 size=128x64`

- 地址探测命中 **0x3C**；初始化序列走完 → **模块是 SSD1306，`OLED_CTRL_SH1106` 保持默认 0 正确**；
- GPIO4/5 接线确认可用；
- ⚠️ 屏幕**显示内容**尚未人工目视确认（无摄像头）——本次唯一未覆盖的验收项。

**③ 主链路四幕联调：通过，且与离线自测/上轮实测逐位一致**

用 `tools/offline_test/send_stream.py` 的同一批合成帧（离线测过的帧 = 上板发的帧）连续推流：

| 环节 | 真机日志 | 参照值 |
|---|---|---|
| 收包质量 | `STATS packets=637 frames_ok=637 frames_bad=0` | 全部解出，0 坏帧 |
| 走动 | `STATE IDLE -> WALK`（gyro2=1221414） | 一致 |
| 冲击 | `STATE WALK -> PENDING seq=25 mag2=90425590` | **与上轮实测值相同** |
| 确认 | `STATE PENDING -> CONFIRMED seq=99`，距冲击 **1517ms** | seq=99 / 1511ms |
| 报警下行 | `TX cmd=0x01 [AA 55 01 00 00 00 01 0D]` | **逐字节一致** |
| 抓拍指令 | `snap cmd sent to 192.168.4.2:8889` | 契约 §1.3 |
| 解除 | `STATE CONFIRMED -> IDLE` + `TX cmd=0x02 [AA 55 02 00 00 00 02 0D]` | **逐字节一致** |
| 状态同步 | `TX cmd=0x10 p1=0x01 / 0x02` | 契约 §4.3 |

→ **结论：新增两个模块没有让已验证的主链路回归。** 冲击阈值、确认帧号、时间窗、
下行帧字节这些决定安全性的值全部吻合。

**④ SD 记录：已通过**

真机实测（`DATA_004.CSV` 开头）：

```
Timestamp_ms,Seq,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,CSI_Variance,State
24026,0,41,61,2048,8,-7,5,0,0
24047,1,41,61,2048,8,-7,5,0,0
24067,2,41,61,2048,8,-7,5,0,0
```

- 表头与契约 §2.1 逐字节一致，共 10 字段；
- `Seq` 0,1,2… 连续无跳号 → 写入无损坏；
- `Acc = 41/61/2048` 正是 idle 场景 0.02g/0.03g/1.0g 的原始 ADC 值 → 数值链路正确；
- 时间戳 ~20ms 递增（50Hz）；`CSI_Variance` 占位 0；`State` 0 = IDLE；
- 文件名递增：`DATA_001` → `DATA_008`（每次上电取卡上最大编号 +1）。

**⑤ 降级设计：真机已确认**

SD / OLED 任一初始化失败时只禁用自身功能，SoftAP / UDP 收包 / 跌倒判定 /
OLED / FPGA 下行**全部照常工作** —— 与设计预期一致。

**⑥ 本次未覆盖**

- **FPGA 未接**：声光致动未验（只验到"FPGA 会收到什么帧"，字节级）；
- **OLED 屏内容未目视**。

### 上板后更新的事实

（"上板不符改参数"类备注；以下几条已由真机确认，不再是待确认项）

- **OLED 控制器 = SSD1306**（探测 + 初始化确认）→ `OLED_CTRL_SH1106` **保持 0**，
  原"若错位 2 列则是 SH1106"的备用开关未被触发；
- **GPIO4 / 5 已确认可用**（OLED 通信成功）；
- **GPIO10~13 已确认可用**（SD 卡识别成功、SPI 通信正常）→ **SD 挂载失败与接线/引脚无关**；
- 仍待确认：**六脚 SD 模块的供电丝印**。卡识别只需小电流，写卡大电流下才可能暴露供电不足，
  故这一条保留（标 `3.3V` 的只能供 3V3，标 `5V` 的按模块说明供电；信号侧始终 3.3V）。

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
| Artix-7（协处理器） | `firmware/coprocessor` | UART 命令接收 + 帧解码（滑动重同步）+ 6 位数码管显示（状态助记符 + 实时合加速度读数）+ 蜂鸣器载波节拍告警 + LED 效果引擎（`led_ctrl.v`）+ 上电自检（单板可验）+ 可固化到板载配置 Flash | 无头仿真 9 场景全过（`ALL CHECKS PASSED`）；综合/实现/比特流无头完成（`top.bit`，0 error，`All user specified timing constraints are met`）；`top.mcs` 已生成 | ✅ 完成 |

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
| 引脚与功能开关 | `main/board_pins.h` | OLED/SD 引脚 + `ENABLE_*` 开关集中点，改线只改这里 |
| OLED 显示 | `main/oled_display.c/.h` | I2C 驱动（SSD1306/SH1106 通吃）+ 8 行状态屏 |
| SD 记录 | `main/sd_log.c/.h` | SPI2+FATFS 挂载、文件编号递增、环形缓冲 + 记录任务 |
| CSV 格式 | `main/log_fmt.c/.h` | 纯 C99，契约 §2.1 表头与行格式（宿主机离线可测） |
| 主程序 | `main/base_station.c` | SoftAP/收包/判定/下发/日志 + OLED 状态快照任务 |
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
- **上机操作文档按仓库规范不入库**（烧录/接线/连热点/发假包/每步现象/排障），
  由开发者自行保管，仓库内不引用。

---

## 2026-10-18 增强：FPGA 声光效果 + OLED 中文状态屏

决策依据与完整验证见 [CHANGELOG](CHANGELOG.md) 同日「声光效果增强」条目。
**协议 §4 帧格式/命令码/状态码零改动**，S3 侧 `fpga_link` 无需重编即可配套。

| 项 | 内容 |
|---|---|
| 蜂鸣器 | `rtl/buzzer_ctrl.v` 改「约 2.7kHz 载波 × 三短一停节拍」（响150/停100/响150/停100/响150/静600 ms），新增 `beat` 包络输出。**音量提升来自载波**：实板若为有源蜂鸣器且叠加后音色变差，`TONE_ENABLE=0` 一处回退 |
| 数码管 | `rtl/seg_display.v` 分档显示：静止/走动 = `S`+状态码右对齐 + 小数点 1Hz 心跳；待确认 = 数字位 4Hz 快闪；告警 = `FALL--` 约 3.3Hz 急闪 + 6 位小数点 30ms/步流水 |
| LED | 新增 `rtl/led_ctrl.v`（16 路 8 位软 PWM）：告警随蜂鸣节拍爆闪 / 静止 `led[0]` 呼吸灯 / 走动环形流水 / 待确认 KITT 扫描；`top.v` 由内联映射改为实例化该模块 |
| OLED | 新增 `main/font_cjk16.h`（14 字 16x16，约 0.5KB）+ UTF-8 混排、2 倍放大、反色/实底三组绘制原语；屏面 = 反色标题栏「跌倒检测基站」+ 32x32 大号中文状态（告警 2Hz 反色闪烁）+ `PKT/SEQ` + `SD/F/坏帧/丢弃` |
| 字模工具 | `tools/font_gen/gen_cjk16.ps1`（本机宋体渲染 + 墨水居中 + 输出 ASCII 点阵图供目视校验；脚本纯 ASCII，规避 PowerShell 5.1 按 ANSI 解析中文注释的问题） |
| 验收证据 | FPGA 仿真 `SIM RESULT: PASS`（9 场景全过，负测试零回归）；Vivado `BITSTREAM_OK`、DRC 0 Errors、`All user specified timing constraints are met`、627 LUT(3.01%)/509 FF(1.22%)；固件 `fullclean` 全量重编 **0 error / 0 编译告警**，`base_station.bin` = 929,888 B（1M 分区余 11%）；离线自测 **14/14** |
| 未覆盖 | **本次未上板**：① 蜂鸣器响度/音色试听；② OLED 中文目视（含上下半页是否颠倒） |

---

## 2026-10-18 第二次变更：掉电固化 + 数码管改为实时遥测读数

决策依据与完整验证见 [CHANGELOG](CHANGELOG.md) 同日第二条条目。
**协议 §4.3 / §4.5 / §4.8 已修订**（复用 `SET_STATE` 的 `p2` 保留位传递遥测），
S3 侧 `fpga_link` / `base_station` 需重编重烧才能看到新读数。

| 项 | 内容 |
|---|---|
| 掉电固化 | 新增 `program_mcs.tcl`：无头把 `top.mcs` 烧进板载配置 Flash（`n25q32-3.3v-spi-x1_x2_x4`，带器件型号护栏），内置 SPI 桥接位流自动定位与加载。此前只有 `program_bit.tcl` 走 JTAG 写 SRAM，**断电即失**。**烧写本身已验证成功**（`Erase`/`Program`/`Verify` 全过、读回逐字节一致） |
| 数码管 | `rtl/seg_display.v` 重写为「2 位状态助记符 + 4 位实时合加速度读数」：`St`/`Go`/`Pd`/`AL` + `XX.XX`（单位 g，`an[2]` 的 dp 作小数点）。6 位全部承载信息，取消原 `S0`/`FALL--` 这类半屏空白；原小数点流水动画取消（dp 已有小数点语义） |
| 遥测通路 | `cmd_decoder.v` 在 `SET_STATE` 时锁存 `p2` → `mag_x100`/`mag_valid`/`mag_update`；`0xFFFF` 为"暂无遥测"哨兵 → 读数位显示 `----`（不伪装成 `00.00`）；`base_station.c` 以 1Hz 周期发遥测刷新帧，CONFIRMED 期间停发（读数定格为冲击幅值） |
| 读数老化 | **FPGA 侧 3s 老化看门狗**（`seg_display.v` 的 `TELEM_STALE_DIV`）：超时自动切 `----`，不依赖 S3 是否继续发哨兵。**告警期间冻结**（`alarm_on=1` 停摆计时器），读数定格为冲击幅值直到告警解除。之所以放 FPGA 侧，是因为实测发现 S3 的 `SO_RCVTIMEO` 在本 IDF 构建下不生效、无数据时收包任务阻塞在 `recvfrom`（既有缺陷，见下行） |
| 换算函数 | `fall_core.c` 新增 `fall_acc_mag_centi_g()`：纯整数逐位开方（不引入 `math.h`，保持纯 C99 以便离线自测直接 `#include`），输出 0.01g 单位 |
| 验收证据 | FPGA 仿真 `SIM RESULT: PASS`（含新增遥测读数、老化看门狗、助记符、哨兵断言）；Vivado `BITSTREAM_OK`、DRC 0 Errors、`All user specified timing constraints are met`；`top.mcs` 6,165,628 B 已生成；固件全量重编 **0 error / 0 编译告警**；离线自测 **15/15**（新增场景 15 遥测换算逐点锁定） |
| ⚠️ **未解决**：上电不自动启动 | 烧写成功、读回一致，但**断电再上电不会自动启动**（`DONE`=0，全无反应），**按一次板上 CONFIG 按钮即正常**。该现象即证明 Flash 内容、MSEL、烧写方式、mcs 格式均无问题，唯一失效的是"上电瞬间的自动触发"，指向板级上电复位电路。**该板在本次改动前无需按 CONFIG，上电即自动启动；本次未触及配置电路相关约束，但无法定位致因，按现状无法修复。** 详见 `docs/FPGA实验板资料参考.md` §9b |
| 本轮发现的既有缺陷（非本次改动引入，待单独修） | **S3 收包 socket 的 100ms `SO_RCVTIMEO` 实测不生效**：连续 25s 无数据时 `STATS` 一次都不打印（PowerShell 直读串口 + `idf.py monitor` 两种方式均如此），日志停在 `main_task: Returned from app_main()`；有帧到达时 `STATS` 正常。与历史记录"33s 无数据、**收到下一帧时**才判掉线"吻合。曾改用 `select()` 显式超时重编重烧，实机表现相同，因无法取得可验证改善而**回退**。影响面：空闲路径的 `fall_detector_tick`（告警超时解除/掉线保护）与遥测哨兵；**不影响收包主链路**，也不影响数码管读数真实性（已由 FPGA 侧 3s 老化兜住） |
| 未覆盖（需上机） | ① S3 重烧后数码管读数目视；② 蜂鸣器音色试听 |

---

## 本期明确不做（原因与决策见 CHANGELOG）

- 真实 CSI 提取（需穿戴端在场，`csi_stub` 占位留接口；CSV 的 `CSI_Variance` 列当前恒 0）
- 穿戴端联调（需成员 B 在场；抓拍指令照发，目标不在只打日志）
- SD 运行中热插拔重挂载（六脚模块无卡检测脚，拔卡即停止记录，重新上电恢复）
