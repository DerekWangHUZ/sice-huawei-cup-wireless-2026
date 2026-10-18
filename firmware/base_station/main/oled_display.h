/*
 * oled_display.h -- 1.3" 四脚 I2C OLED（SSD1306 / SH1106）状态显示
 *
 * 硬件接线见 main/board_pins.h 与 hardware/schematics/基站接线表.md。
 *
 * 为什么不用 ESP-IDF 内置的 esp_lcd_panel_ssd1306：
 *   IDF 的 SSD1306 面板驱动在 draw_bitmap 里下发 0x21/0x22（设置列/页范围），
 *   这两条是 SSD1306 的【水平寻址】命令，SH1106 不支持——而 1.3" 四脚模块
 *   两种控制器都常见（本机模块实测为 SSD1306）。因此本模块自写精简驱动，
 *   统一采用【页寻址】写屏（0xB0|page + 列地址低/高半字节），两种控制器都能
 *   跑，差异只在初始化序列与列偏移，由 board_pins.h 的 OLED_CTRL_SH1106 切换。
 *   详见 docs/CHANGELOG.md [2026-10-17] 决策 D1/D2。
 *
 * 显示能力（2026-10-18 升级为中文状态屏）：
 *   - ASCII  6x8 点阵（0x20~0x7E），128x64 = 21 字符 x 8 行；
 *   - 中文   16x16 点阵，按需收录子集（见 font_cjk16.h），可 2 倍放大成
 *            32x32 大字；
 *   - 文本接口 UTF-8 感知：同一函数内 ASCII 与中文混排，自动按字宽推进。
 *
 * 一屏布局（oled_show_status）：
 *   y0  ~ 15 : 实底反色标题栏   「跌倒检测基站」
 *   y16 ~ 47 : 32x32 大号中文状态 正常 / 行走 / 疑似跌倒 / 跌倒告警
 *              （告警时整条反色闪烁，2Hz）
 *   y48 ~ 55 : PKT n SEQ n        收包活性
 *   y56 ~ 63 : SD:xx F:xx a/b     存储 / FPGA 链路 / 坏帧 与 SD 丢弃计数
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

/* 把指定页整页填成 value（0x00 全灭 / 0xFF 全亮，用于实底标题栏、闪烁底） */
void oled_fill_page(uint8_t page, uint8_t value);

/* 在指定页（0~7，每页 8 像素高）绘制一行 ASCII 文本，超过 21 字符截断 */
void oled_draw_row(uint8_t page, const char *text);

/*
 * UTF-8 混排文本绘制（顺色：在暗底上点亮点）。
 *   ASCII 字符占 6 列 x 8 行（画在 page 这一页）；
 *   中文占 16 列 x 16 行（跨 page 与 page+1 两页），未收录的字画空心方框。
 */
void oled_draw_text(uint8_t page, uint8_t x, const char *utf8);

/*
 * 同上，但为【反色】：汉字/ASCII 的墨迹被清成暗色，用于实底标题栏。
 * 调用前需先把目标页填成 0xFF 作底。
 */
void oled_draw_text_inv(uint8_t page, uint8_t x, const char *utf8);

/*
 * 2 倍放大中文（每字 32x32，占 4 页），按字数水平居中，最多铺满 128 列。
 * 仅处理 16x16 汉字；inv=true 时反色（调用方需先把该 4 页填成 0xFF）。
 */
void oled_draw_big_text(uint8_t page, const char *utf8, bool inv);

/* 把帧缓存中【发生变化的页】推送到面板（8 页 x 128B，页寻址） */
esp_err_t oled_flush(void);

/* 渲染并刷新一屏状态（= clear + 标题/大字/遥测行 + flush） */
esp_err_t oled_show_status(const oled_status_t *s);

#endif /* OLED_DISPLAY_H */
