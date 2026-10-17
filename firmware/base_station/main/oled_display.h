/*
 * oled_display.h -- 1.3" 四脚 I2C OLED（SSD1306 / SH1106）状态显示
 *
 * 硬件接线见 main/board_pins.h 与 hardware/schematics/基站接线表.md。
 *
 * 为什么不用 ESP-IDF 内置的 esp_lcd_panel_ssd1306：
 *   IDF 的 SSD1306 面板驱动在 draw_bitmap 里下发 0x21/0x22（设置列/页范围），
 *   这两条是 SSD1306 的【水平寻址】命令，SH1106 不支持——而 1.3" 四脚模块
 *   两种控制器都常见（本机模块型号待定）。因此本模块自写精简驱动，统一采用
 *   【页寻址】写屏（0xB0|page + 列地址低/高半字节），两种控制器都能跑，
 *   差异只在初始化序列与列偏移，由 board_pins.h 的 OLED_CTRL_SH1106 切换。
 *   详见 docs/CHANGELOG.md [2026-10-17] 决策 D1/D2。
 *
 * 显示能力：6x8 点阵 ASCII（0x20~0x7E），128x64 屏 = 21 字符 x 8 行。
 * 不做中文（字库体积与本期收益不成比例）。
 *
 * 容错：初始化时探测 I2C 地址，无应答则 oled_ready() 恒 false，
 *       调用方跳过显示；显示失败绝不影响跌倒判定/报警主链路。
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "fall_core.h"

/* OLED 一屏所需的状态快照（由主程序在互斥保护下填充） */
typedef struct {
    fall_state_t state;         /* 当前判定状态 */
    bool     alarm;             /* 是否处于告警态 */
    uint16_t last_seq;          /* 最近一帧序号 */
    uint32_t packets;           /* 收到的 UDP 包数 */
    uint32_t frames_ok;         /* 解出的合法帧数 */
    uint32_t frames_bad;        /* 拒收的坏帧数 */
    uint32_t csi_var;           /* CSI 方差（本期占位 0） */
    bool     sd_ok;             /* SD 是否在记录 */
    uint32_t sd_dropped;        /* SD 队列丢弃条数 */
    char     sd_file[24];       /* SD 文件名（如 DATA_001.CSV），未挂载填 "-" */
    bool     fpga_ok;           /* FPGA UART 链路是否就绪 */
} oled_status_t;

/*
 * 初始化 I2C 与面板。返回 ESP_OK 表示显示可用。
 * 失败（模块未接/地址不符）不致命：打日志并让 oled_ready() 返回 false。
 */
esp_err_t oled_init(void);

/* 显示是否可用 */
bool oled_ready(void);

/* 清空帧缓存 */
void oled_clear(void);

/* 在指定页（0~7，每页 8 像素高）绘制一行 ASCII 文本，超过 21 字符截断 */
void oled_draw_row(uint8_t page, const char *text);

/* 把帧缓存中【发生变化的页】推送到面板（8 页 x 128B，页寻址） */
esp_err_t oled_flush(void);

/* 渲染并刷新一屏状态（= clear + draw rows + flush） */
esp_err_t oled_show_status(const oled_status_t *s);

#endif /* OLED_DISPLAY_H */
