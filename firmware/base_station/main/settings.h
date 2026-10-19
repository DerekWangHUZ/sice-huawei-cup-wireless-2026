/*
 * settings.h -- 基站可视化设置的持久化与应用
 *
 * 设置项由 FPGA 键盘在 OLED 设置页（P3）上调节（A/B 选项 C/D 调值），
 * 每次改动立即保存 NVS 并即时生效；上电时从 NVS 恢复并下发 FPGA。
 *
 * 判定语义类参数（冲击阈值/确认窗口/解除判据）刻意不进设置页——
 * 告警"锁存 + 起身解除"是安全设计，不开放现场调节。
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* 蜂鸣器音调档位表（0x03 命令 p2 = 载波半周期；0 = FPGA 默认 2.7kHz） */
#define UI_TONE_DIV_0 9260u   /* 低 2.7kHz（默认共鸣点） */
#define UI_TONE_DIV_1 8000u   /* 中 ~3.1kHz */
#define UI_TONE_DIV_2 6250u   /* 高 4.0kHz */

#define SETTINGS_ITEM_N 9u

typedef struct {
    uint8_t  contrast;   /* OLED 对比度（0x81 命令值） */
    uint8_t  page_auto;  /* 页面自动轮播 0/1 */
    uint16_t cycle_ms;   /* 轮播周期 */
    uint16_t tel_ms;     /* 数码管遥测刷新周期（0.5/1/2s） */
    uint8_t  pend_beep;  /* 待确认提示音 0/1 */
    uint8_t  buzzer_en;  /* 蜂鸣器使能 0/1 */
    uint8_t  tone_sel;   /* 音调档 0..2 */
    uint8_t  x_shift;    /* OLED 左右边缘补偿列数 0..4（实测屏左右各遮约 2 列） */
} ui_settings_t;

/* 从 NVS 载入到 RAM（不立即 apply；外设就绪后由 apply_* 下发） */
esp_err_t settings_init(void);

void settings_get(ui_settings_t *out);

/* 蜂鸣器相关 → 0x03 命令下发 FPGA；OLED 对比度/遥测周期 → 本地生效 */
void settings_apply_audio(void);
void settings_apply_visual(void);

/* 设置页：调值（dir=+1/-1，循环或钳位按项定义），保存 NVS 并即时生效 */
void settings_adjust(uint8_t item, int dir);

/* 设置页渲染辅助：取第 item 项的名称与当前值文本（ASCII） */
void settings_get_item_text(uint8_t item, char *name, size_t nlen,
                            char *val, size_t vlen);

/* 恢复出厂：全部项回默认值，保存并生效 */
void settings_factory_reset(void);

/* 数码管遥测刷新周期（base_station.c 的 fpga_telemetry_tick 读取） */
extern volatile uint16_t g_fpga_tel_period_ms;

#endif /* SETTINGS_H */
