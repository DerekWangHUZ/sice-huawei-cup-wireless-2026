# Vivado 工程移植方案

> `firmware/coprocessor` 在换机器、换路径、别人 clone 仓库后如何可靠重建。

## 目录布局

```text
firmware/coprocessor/
├── rtl/            # RTL 源码（uart_rx / cmd_decoder / seg_display / buzzer_ctrl / led_ctrl / top）
├── constrs/        # pins.xdc
├── sim/            # tb_top.v + run_sim.ps1（无头仿真）
├── rebuild.tcl     # 重建工程（原地生成 coprocessor.xpr，-force 可重复执行）
├── build_bit.tcl   # 无头综合+实现+出比特流（产物 coprocessor.runs/impl_1/top.bit）
├── program_bit.tcl # 无头 JTAG 下载 top.bit（掉电即失，需板子接 JTAG）
├── gen_mcs.tcl     # 无头生成 top.mcs（供固化到板载配置 Flash）
└── program_mcs.tcl # 无头把 top.mcs 烧进板载配置 Flash（掉电不丢，需板子接 JTAG）
```

RTL 与约束直接放在 `rtl/`、`constrs/`，与 Vivado 生成的工程数据目录
（`coprocessor.srcs/`、`.cache/`、`.runs/` 等）分离——重建脚本 `-force`
覆盖工程时不会触及源码。

## 克隆后重建

```bash
cd firmware/coprocessor
vivado -mode batch -source rebuild.tcl -tclargs --origin_dir .
```

在 **PowerShell / CMD / bash** 里直接执行。`--origin_dir .` 表示以当前目录为源文件根。

跑完后 **`coprocessor.xpr` 生成于当前目录**（`firmware/coprocessor/coprocessor.xpr`），
双击即可用 Vivado GUI 打开。重复执行会带 `-force` 覆盖重建，安全。

无头出比特流（可选，等效 GUI 的 Run Synthesis → Run Implementation → Generate Bitstream）：

```bash
vivado -mode batch -source build_bit.tcl
# 产物: coprocessor.runs/impl_1/top.bit
```

无头 JTAG 下载（等效 GUI 的 Program Device，**掉电即失**；改完代码想立刻看现象用这条）：

```bash
vivado -mode batch -source program_bit.tcl
# 成功标志: PROGRAM_OK + "End of startup status: HIGH"
# 未接板/接错板: NO_DEVICE / PART_MISMATCH（脚本会拒绝，不会误写别的器件）
```

固化到板载配置 Flash 所需的 `top.mcs`：

```bash
vivado -mode batch -source gen_mcs.tcl
# 产物: coprocessor.runs/impl_1/top.mcs
```

**固化到板载配置 Flash（掉电不丢）**：

```bash
vivado -mode batch -source program_mcs.tcl
# 成功标志: MCS_PROGRAM_OK
# 未接板/接错板: NO_DEVICE / PART_MISMATCH；器件不匹配: CFGMEM_PART_MISSING
# 缺 mcs: MCS_MISSING（先跑 gen_mcs.tcl）
```

- 烧写对象是板载配置 Flash（本板自动识别为 `n25q32-3.3v-spi-x1_x2_x4`，
  与 `gen_mcs.tcl` 的 `-size 32 / SPIx1` 对应），**不是** FPGA 内部 SRAM。
- 脚本会自动加载 Vivado 自带的 SPI 桥接位流
  （`spi_xc7a35t_pullnone.bit`，位于 Vivado 安装目录的
  `data/xicom/cfgmem/bitfile.zip`）。这是 JTAG 间接访问 Flash 的必要前置步骤：
  GUI 会自动做，手写 Tcl 若遗漏会报
  `[Labtools 27-3347] Failure to set flash parameters`（该提示有误导性）。
- **改完 RTL 必须重新 `gen_mcs.tcl` + `program_mcs.tcl`**，否则 Flash 里仍是旧
  比特流；只想立刻看现象仍可用 `program_bit.tcl`（那次有效，掉电即失）。
- ⚠️ **本板已知问题**：烧写成功（`Erase`/`Program`/`Verify` 全过、读回一致）后，
  断电再上电**仍不会自动启动**，需要**按一次板上 CONFIG 按钮**。
  该现象与比特流、Flash 内容、mcs 格式、烧写方式均无关（按 CONFIG 能正常
  运行即可证明），指向板级上电复位电路。详见
  [`FPGA实验板资料参考.md`](FPGA实验板资料参考.md) §9b 的"已知问题"小节。

> 若 `vivado` 不在 PATH（本机默认如此），用安装目录下的 `bin\vivado.bat` 全路径调用，
> 例如 `C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat -mode batch -source program_mcs.tcl`。

---

## 维护须知

以下内容队友不用看。

### `.xpr` 不入库

`.xpr` 是工程状态快照而非工程定义，内部写死导出机器的绝对路径。实测（Vivado 2026.1）复制到新路径后：工程"能打开"、综合能跑完，但**文件列表解析不可靠**。

### 工程结构变化后重新导出

**必须在 Vivado Tcl Console 中执行**（GUI 底部 Tcl 面板，或 `vivado -mode tcl` 进入）：

```tcl
open_project "<工程目录>/coprocessor.xpr"
write_project_tcl -paths_relative_to "<工程目录>" -no_copy_sources -force ./rebuild.tcl
```

| 参数 | 作用 |
|---|---|
| `-paths_relative_to <dir>` | 脚本内源文件路径相对于**工程目录**，而非脚本输出目录 |
| `-no_copy_sources` | 不把源文件复制进脚本目录（默认即 1），保持脚本与源码分离 |
| `-force` | 覆盖已有脚本 |

导出后需手动把脚本里的本机绝对路径替换为 `<PROJECT_DIR>` 占位符，否则会泄露目录结构。

> ⚠️ 增删源文件、改 IP、改 run 策略后必须重新导出。脚本是工程快照，不是一次性产物。

### `.gitignore` 覆盖

`*.xpr`、`*.runs/`、`*.cache/`、`*.hw/`、`*.ip_user_files/`、`*.sim/`、`.Xil/`、`*.log`、`*.jou`、`*.str`、`*.bit.bin`、`xsim.dir/`、`*.wdb`、`*.pb`。只提交源码（`rtl/`、`constrs/`、`sim/` 脚本）+ `rebuild.tcl`、`build_bit.tcl`、`gen_mcs.tcl`。
