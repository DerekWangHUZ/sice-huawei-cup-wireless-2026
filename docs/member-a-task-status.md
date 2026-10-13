# 成员 A 负责部分 — 当前状态

> 记录人：成员 A（Derek）
> 更新时间：2026-10
> 范围：本人负责的三个固件工程**初始化骨架**的当前状态（仅“把工程建起来”，尚未写业务功能代码）。

## 目标约定

按项目要求，本阶段交付物是**三个可编译 / 可烧录 / 可运行的空工程骨架**，不含任何业务功能逻辑。
教学环节（如何初始化）由本人以“傻瓜式”步骤带做，业务代码留待骨架全部验收后再开发。

## 三块板骨架状态

| 板 | 工程路径 | 工具链 / 配置 | 验收证据 | 状态 |
|---|---|---|---|---|
| ESP32-S3（基站） | `firmware/base_station` | ESP-IDF v6.1，`idf.py`，target `esp32s3`，串口 COM4（CH343/WCH） | monitor 打印 `Calling app_main()`；`printf` 初始化信息可观测 | ✅ 已建好并跑通 |
| STM32 F401RE（协处理器） | `firmware/coprocessors/stm32` | STM32CubeMX 生成 `.ioc`（NUCLEO-F401RE，HSE 84MHz，PA5=LD2）→ STM32CubeIDE 编译/烧录（ST-Link，VCP=COM3 115200） | `while(1)` 内 `HAL_GPIO_TogglePin` + `HAL_Delay(500)`，LD2 以 1Hz 闪烁，0 error 0 warning | ✅ 已建好并跑通 |
| Artix-7（协处理器 / 报警） | `firmware/coprocessors/fpga` | Vivado RTL 工程，`xc7a35tftg256-1`，50MHz/D4，16 LED P9…M1 | 下载日志 `End of startup status: HIGH`；15 颗 LED 正常闪烁（分频/同步均验证） | ✅ 已建好并跑通 |

### 备注
- **FPGA 板已知硬件缺陷**：`led[15]`（引脚 M1）在本板物理不亮，非工程问题，验收忽略。
- **FPGA 复位坑**：`rst_n`(T9) 须开内部上拉（`PULLUP true`），否则设计卡复位、LED 全灭；详见 `docs/fpga-board-reference.md`。
- 三块板详细引脚/资源见 `docs/fpga-board-reference.md`；STM32 / ESP32 初始化步骤后续可补入 `docs/setup-tutorial.md`。

## 接口契约（暂未实现，仅记录待办）

按团队《团队协作规范与接口契约》，未来三板间通过 UART 对接，约定 **18 字节 UDP 姿态帧**（帧头 `0xAA55…`）：
- ESP32-S3 ↔ STM32 / FPGA 走 UART；PuTTY 统一 115200 8N1。
- 本阶段仅占位，未写任何编解码逻辑。

## 尚未开始（业务功能，待骨架验收后开发）

- ESP32-S3：UDP + CSI 采集 + SD 卡 + OLED（基站侧）
- STM32 F401RE：姿态解算（协处理器侧）
- Artix-7：报警逻辑（协处理器侧）

## 本次会话清理记录

删除的临时 / 垃圾文件（均为本会话产生，非仓库原有脚手架）：
- `firmware/base_station/monitor_dump.txt`
- `firmware/base_station/sdkconfig.old`（ESP-IDF 自动备份，`.gitignore` 已忽略）
- `firmware/coprocessors/fpga/fpga.runs/`（含 `top.bit`、`runme.log`，被 `*.runs/` 忽略）

保留：仓库原有 `.gitkeep` 脚手架、`base_station/build/`（ESP-IDF 正常产物，`build/` 已忽略）。
