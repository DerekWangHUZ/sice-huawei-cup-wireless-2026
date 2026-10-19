/*
 * board_pins.h -- 基站扩展模块的引脚与功能开关（唯一改动点）
 *
 * ==========================================================================
 * 新增模块（2026-10-17）
 * ==========================================================================
 *
 *   OLED  1.3" 四脚 I2C 128x64（SSD1306 / SH1106）
 *     VCC -> 3V3      GND -> GND      SCL -> GPIO5      SDA -> GPIO4
 *
 *   microSD 六脚 SPI 模块
 *     VCC -> 3V3      GND -> GND      CS  -> GPIO10
 *     SCK -> GPIO12   MOSI -> GPIO11  MISO -> GPIO13
 *
 * 现场改线只需改本文件，其它源文件不出现裸引脚号。
 * 接线全表见 hardware/schematics/基站接线表.md。
 * ==========================================================================
 *
 * 引脚避让（ESP32-S3，接线前务必对照本机开发板丝印）：
 *   GPIO0 / 3 / 45 / 46   strapping 脚，禁用
 *   GPIO26~32             SPI flash / PSRAM，禁用
 *   GPIO19 / 20           USB D-/D+，禁用
 *   GPIO43 / 44           UART0 控制台，禁用
 *   GPIO17                UART1 TX -> FPGA（既有，勿占）
 *   GPIO18                UART1 RX <- FPGA 上行 UI 帧（fpga_link.h，勿占）
 *
 * I2C 用 GPIO4/5；SPI 用 GPIO10~13，正是 ESP32-S3 SPI2 的 IO_MUX 默认脚
 * （SDSPI_DEFAULT_HOST = SPI2_HOST），走默认脚性能最好。
 * ==========================================================================
 */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* --- 功能总开关（先关后开可二分定位问题） --- */
#define ENABLE_OLED        1
#define ENABLE_SD_LOG      1

/* ==========================================================================
 * OLED（I2C0）
 * ========================================================================== */

#define OLED_SDA_GPIO      4
#define OLED_SCL_GPIO      5
#define OLED_I2C_PORT      I2C_NUM_0
#define OLED_I2C_FREQ_HZ   400000      /* 1.3" 模块板载上拉，400kHz 稳定 */

/*
 * 控制器型号：0 = SSD1306（128 列 RAM），1 = SH1106（132 列 RAM）。
 *
 * 判据（上板现象）：显示内容整体横向错位 2 列 -> 实为 SH1106，把本宏改 1。
 * 两者在【页寻址模式】下命令兼容（0xB0+page / 0x00 / 0x10），差异只在
 * 初始化序列的电荷泵命令与列偏移，故本驱动一套代码通吃。
 * 详见 docs/CHANGELOG.md [2026-10-17] 决策 D1/D2。
 */
#define OLED_CTRL_SH1106   0

#if OLED_CTRL_SH1106
#define OLED_COL_OFFSET    2           /* SH1106：可见区从第 2 列开始 */
#else
#define OLED_COL_OFFSET    0           /* SSD1306：列 0 对齐 */
#endif

/* I2C 地址：常见 0x3C，少数模块 0x3D；启动时依次探测 */
#define OLED_ADDR_PRIMARY    0x3C
#define OLED_ADDR_SECONDARY  0x3D
#define OLED_PROBE_TIMEOUT_MS 50

#define OLED_WIDTH         128
#define OLED_HEIGHT        64
#define OLED_PAGES         (OLED_HEIGHT / 8)

/* 显示刷新：状态变化立即刷新，另外按此周期兜底刷新（掉线/计时类信息） */
#define OLED_REFRESH_PERIOD_MS  250

/* ==========================================================================
 * microSD（SPI2）
 * ========================================================================== */

#define SD_CS_GPIO         10
#define SD_SCK_GPIO        12
#define SD_MOSI_GPIO       11
#define SD_MISO_GPIO       13

#define SD_MOUNT_POINT     "/sdcard"

/* 六脚模块无卡检测（CD）与写保护（WP）脚 */
#define SD_HAS_CARD_DETECT 0
#define SD_CARD_DETECT_GPIO (-1)

/*
 * SPI 时钟上限。SDSPI 允许 400kHz~20MHz；20MHz 是默认值。
 * 若上板挂载不稳定（尤其长杜邦线），先降到 10000（10MHz）再试。
 */
#define SD_MAX_FREQ_KHZ    20000

/* 环形缓冲：256 项 x ~28B ≈ 7KB，按 50Hz 约 5s 缓冲深度 */
#define SD_LOG_QUEUE_ITEMS 256

/* 落盘周期：即使一直有数据也至少这么久 fsync 一次，限制掉电损失 */
#define SD_LOG_FLUSH_PERIOD_MS 1000

/* 落盘用的行缓冲大小（FATFS 写放大保护） */
#define SD_LOG_FILE_BUF_SIZE 2048

#endif /* BOARD_PINS_H */
