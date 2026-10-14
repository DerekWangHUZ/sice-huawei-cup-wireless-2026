# 成员 A 负责部分 — 当前状态

---

## 架构变更（2026-10-14）

**STM32 协处理器已撤除，基站由三块板改为两块板（ESP32-S3 + Artix-7 FPGA）。**

撤除原因不是技术障碍，而是在系统既定架构下 STM32 找不到技术上不可替代的职责（姿态数据的第一落点必然是 S3；告警输出资源被 FPGA 实验板全面覆盖）。完整论证、原工程留档与跨团队影响见 [CHANGELOG.md](CHANGELOG.md)。

---

## 当前两块板状态

| 板 | 工程路径 | 工具链 / 配置 | 验收证据 | 状态 |
|---|---|---|---|---|
| ESP32-S3（基站） | `firmware/base_station` | ESP-IDF v6.1，`idf.py`，target `esp32s3`，串口 COM4（CH343/WCH） | monitor 打印 `Calling app_main()`；`printf` 初始化信息可观测 | ✅ 骨架已跑通 |
| Artix-7（协处理器 / 报警） | `firmware/coprocessor` | Vivado 2026.1 RTL 工程，`xc7a35tftg256-1`，50MHz/D4，16 LED P9…M1 | 下载日志 `End of startup status: HIGH`；15 颗 LED 正常闪烁（分频/同步均验证） | ✅ 骨架已跑通 |

> FPGA 工程于 2026-10-14 由原 `firmware/coprocessors/fpga`（工程名 `fpga`）重建为 `firmware/coprocessor`（工程名 `coprocessor`），源码 `top.v` / `pins.xdc` 逐字未变。`.xpr` 不再纳入版本控制，详见 CHANGELOG。

### 备注
- **FPGA 板已知硬件缺陷**：`led[15]`（引脚 M1）在本板物理不亮，非工程问题，验收忽略。
- **FPGA 复位坑**：`rst_n`(T9) 须开内部上拉（`PULLUP true`），否则设计卡复位、LED 全灭；详见 `docs/FPGA实验板资料参考.md`。
- 详细引脚/资源见 `docs/FPGA实验板资料参考.md`。
- **待核实**：`docs/FPGA实验板资料参考.md` §7 记载数码管为 6 位（位选 `an[0..5]` 共 6 个引脚），与板子照片一致；写驱动代码前按此实现，若上板显示异常再核对。

---

## 接口契约（暂未实现，仅记录待办）

按团队《团队协作规范与接口契约》，未来 S3 与 FPGA 通过 UART 对接：
- ESP32-S3 ↔ FPGA 走 UART；PuTTY 统一 115200 8N1。
- 穿戴端姿态推流为 18 字节 UDP 帧（帧头 `0xAA55…`，50 Hz），见 README。
- 本阶段仅占位，未写任何编解码逻辑。

---

## 尚未开始（业务功能）

- **ESP32-S3**：SoftAP + UDP 8888 收包 + 18 字节帧解析 + 跌倒判定状态机 + 向 FPGA 下发报警命令；CSI 采集暂以桩实现占位。
- **Artix-7**：UART 命令接收 + 命令解码 + 数码管驱动 + 蜂鸣器告警。

**本期 MVP 明确不做**（原因与决策见 CHANGELOG）：
- MicroSD 数据记录、OLED 状态显示（模块未到货）
- 真实 CSI 提取（需穿戴端在场，暂以桩实现）
- STM32 协处理器（已撤除）
