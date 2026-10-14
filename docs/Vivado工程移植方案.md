# Vivado 工程移植方案

> `firmware/coprocessor` 在换机器、换路径、别人 clone 仓库后如何可靠重建。

## 克隆后重建

```bash
cd firmware/coprocessor
vivado -mode batch -source rebuild.tcl -tclargs --origin_dir .
```

在 **PowerShell / CMD / bash** 里直接执行。`--origin_dir .` 表示以当前目录为源文件根。

跑完后 `coprocessor/coprocessor.xpr` 生成，双击即可用 Vivado GUI 打开。

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

`*.xpr`、`*.runs/`、`*.cache/`、`*.hw/`、`*.ip_user_files/`、`*.sim/`、`*.log`、`*.jou`、`*.str`、`*.bit.bin`。只提交源码 + `rebuild.tcl`。
