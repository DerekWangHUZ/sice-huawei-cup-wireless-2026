/*
 * settings.c -- 基站可视化设置的持久化与应用实现
 *
 * 存储策略：每项改动立即写 NVS（namespace "ui"）并即时生效；
 * 上电 settings_init() 从 NVS 恢复，外设就绪后由 apply_* 下发。
 * NVS 打开失败（极端情况）时全部使用编译期默认值，功能降级不阻断。
 */

#include "settings.h"

#include <stdio.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "fpga_link.h"
#include "oled_display.h"

static const char *TAG = "SET";

/* 数码管遥测刷新周期（base_station.c 读取） */
volatile uint16_t g_fpga_tel_period_ms = 1000u;

static ui_settings_t s_cfg;
static bool s_ready = false;

/* 恢复出厂两步确认：首次按 D 进入待确认，3s 内再按 D 才执行 */
static int64_t s_reset_arm_ms = 0;

#define RESET_CONFIRM_WINDOW_MS 3000LL

static bool reset_armed(void)
{
    return (s_reset_arm_ms != 0 &&
            esp_timer_get_time() / 1000LL - s_reset_arm_ms <
                RESET_CONFIRM_WINDOW_MS);
}

static const ui_settings_t s_defaults = {
    .contrast  = 0xCFu,
    .page_auto = 0u,
    .cycle_ms  = 5000u,
    .tel_ms    = 1000u,
    .pend_beep = 0u,
    .buzzer_en = 1u,
    .tone_sel  = 0u,
    .x_shift   = 2u,     /* 实测本屏左右各约 2 列被外框遮挡 */
};

/* ------------------------------------------------------------------
 * NVS 读写
 * ------------------------------------------------------------------ */

static uint8_t cfg_get_u8(nvs_handle_t h, const char *key, uint8_t def)
{
    uint8_t v = def;
    nvs_get_u8(h, key, &v);
    return v;
}

static uint16_t cfg_get_u16(nvs_handle_t h, const char *key, uint16_t def)
{
    uint16_t v = def;
    nvs_get_u16(h, key, &v);
    return v;
}

esp_err_t settings_init(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("ui", NVS_READONLY, &h);
    if (err == ESP_OK) {
        s_cfg.contrast  = cfg_get_u8(h, "contrast", s_defaults.contrast);
        s_cfg.page_auto = cfg_get_u8(h, "pageauto", s_defaults.page_auto);
        s_cfg.cycle_ms  = cfg_get_u16(h, "cyclems", s_defaults.cycle_ms);
        s_cfg.tel_ms    = cfg_get_u16(h, "telms", s_defaults.tel_ms);
        s_cfg.pend_beep = cfg_get_u8(h, "pendb", s_defaults.pend_beep);
        s_cfg.buzzer_en = cfg_get_u8(h, "buzen", s_defaults.buzzer_en);
        s_cfg.tone_sel  = cfg_get_u8(h, "tonesel", s_defaults.tone_sel);
        s_cfg.x_shift   = cfg_get_u8(h, "xshift", s_defaults.x_shift);
        nvs_close(h);
    } else {
        s_cfg = s_defaults;
        ESP_LOGW(TAG, "nvs open failed (%s), using defaults",
                 esp_err_to_name(err));
    }
    s_ready = true;
    return ESP_OK;
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open("ui", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, "contrast", s_cfg.contrast);
    nvs_set_u8(h, "pageauto", s_cfg.page_auto);
    nvs_set_u16(h, "cyclems", s_cfg.cycle_ms);
    nvs_set_u16(h, "telms", s_cfg.tel_ms);
    nvs_set_u8(h, "pendb", s_cfg.pend_beep);
    nvs_set_u8(h, "buzen", s_cfg.buzzer_en);
    nvs_set_u8(h, "tonesel", s_cfg.tone_sel);
    nvs_set_u8(h, "xshift", s_cfg.x_shift);
    nvs_commit(h);
    nvs_close(h);
}

void settings_get(ui_settings_t *out)
{
    if (out != NULL) {
        *out = s_cfg;
    }
}

/* ------------------------------------------------------------------
 * 应用（生效）
 * ------------------------------------------------------------------ */

static uint16_t tone_div_of(uint8_t sel)
{
    switch (sel) {
        case 1:  return UI_TONE_DIV_1;
        case 2:  return UI_TONE_DIV_2;
        default: return UI_TONE_DIV_0;
    }
}

void settings_apply_audio(void)
{
    /* 0x03 命令：bit0=静音（使能取反） bit1=待确认提示音；p2=载波半周期 */
    (void)fpga_link_send_audio((uint8_t)(s_cfg.buzzer_en ? 0u : 1u),
                               (uint8_t)s_cfg.pend_beep,
                               tone_div_of(s_cfg.tone_sel));
}

void settings_apply_visual(void)
{
    (void)oled_set_contrast(s_cfg.contrast);
    oled_set_x_shift(s_cfg.x_shift);
    g_fpga_tel_period_ms = s_cfg.tel_ms;
}

/* ------------------------------------------------------------------
 * 设置页调值
 * ------------------------------------------------------------------ */

static const uint16_t k_cycle_tab[] = { 2000u, 3000u, 5000u, 8000u, 10000u };
#define CYCLE_N (sizeof(k_cycle_tab) / sizeof(k_cycle_tab[0]))

static const uint16_t k_tel_tab[] = { 500u, 1000u, 2000u };
#define TEL_N (sizeof(k_tel_tab) / sizeof(k_tel_tab[0]))

static uint8_t clamp_u8(int v, uint8_t lo, uint8_t hi)
{
    if (v < (int)lo) return lo;
    if (v > (int)hi) return hi;
    return (uint8_t)v;
}

void settings_adjust(uint8_t item, int dir)
{
    if (!s_ready || dir == 0) {
        return;
    }
    switch (item) {
        case 0:     /* 对比度：步进 16，钳位 */
            s_cfg.contrast = clamp_u8((int)s_cfg.contrast + dir * 16, 16, 255);
            (void)oled_set_contrast(s_cfg.contrast);
            break;
        case 1:     /* 轮播开关 */
            s_cfg.page_auto = s_cfg.page_auto ? 0u : 1u;
            break;
        case 2: {   /* 轮播周期 */
            uint8_t i = 0;
            for (i = 0; i < CYCLE_N; i++) {
                if (k_cycle_tab[i] == s_cfg.cycle_ms) break;
            }
            i = (i == CYCLE_N) ? 0 : i;
            i = (uint8_t)((i + CYCLE_N + ((dir > 0) ? 1u : (CYCLE_N - 1u))) % CYCLE_N);
            s_cfg.cycle_ms = k_cycle_tab[i];
            break;
        }
        case 3: {   /* 遥测刷新周期 */
            uint8_t i = 0;
            for (i = 0; i < TEL_N; i++) {
                if (k_tel_tab[i] == s_cfg.tel_ms) break;
            }
            i = (i == TEL_N) ? 0 : i;
            i = (uint8_t)((i + TEL_N + ((dir > 0) ? 1u : (TEL_N - 1u))) % TEL_N);
            s_cfg.tel_ms = k_tel_tab[i];
            g_fpga_tel_period_ms = s_cfg.tel_ms;
            break;
        }
        case 4:     /* 待确认提示音 */
            s_cfg.pend_beep = s_cfg.pend_beep ? 0u : 1u;
            settings_apply_audio();
            break;
        case 5:     /* 蜂鸣器使能 */
            s_cfg.buzzer_en = s_cfg.buzzer_en ? 0u : 1u;
            settings_apply_audio();
            break;
        case 6: {   /* 音调档 */
            s_cfg.tone_sel = (uint8_t)((s_cfg.tone_sel + 3u +
                                        ((dir > 0) ? 1u : 2u)) % 3u);
            settings_apply_audio();
            break;
        }
        case 7:     /* 恢复出厂（两步确认：3s 内按两次 +） */
            if (dir > 0) {
                if (reset_armed()) {
                    s_reset_arm_ms = 0;
                    settings_factory_reset();
                } else {
                    s_reset_arm_ms = esp_timer_get_time() / 1000LL;
                }
            } else {
                s_reset_arm_ms = 0;      /* 任意取消 */
            }
            break;
        case 8:     /* OLED 边缘补偿 X-SHIFT 0..4 */
            s_cfg.x_shift = clamp_u8((int)s_cfg.x_shift + dir, 0, 4);
            oled_set_x_shift(s_cfg.x_shift);
            break;
        default:
            return;
    }
    settings_save();
}

void settings_get_item_text(uint8_t item, char *name, size_t nlen,
                            char *val, size_t vlen)
{
    static const char *const k_names[SETTINGS_ITEM_N] = {
        "CONTRAST", "AUTO PAGE", "CYCLE", "TEL RATE",
        "PEND BEEP", "BUZZER", "TONE", "FACTORY", "X-SHIFT"
    };

    if (name != NULL && nlen > 0) {
        name[0] = '\0';
    }
    if (val != NULL && vlen > 0) {
        val[0] = '\0';
    }
    if (item >= SETTINGS_ITEM_N) {
        return;
    }
    if (name != NULL) {
        snprintf(name, nlen, "%s", k_names[item]);
    }
    if (val == NULL) {
        return;
    }
    switch (item) {
        case 0: snprintf(val, vlen, "%u", (unsigned)s_cfg.contrast); break;
        case 1: snprintf(val, vlen, s_cfg.page_auto ? "ON" : "OFF"); break;
        case 2: snprintf(val, vlen, "%us", (unsigned)(s_cfg.cycle_ms / 1000u)); break;
        case 3: snprintf(val, vlen, "%ums", (unsigned)s_cfg.tel_ms); break;
        case 4: snprintf(val, vlen, s_cfg.pend_beep ? "ON" : "OFF"); break;
        case 5: snprintf(val, vlen, s_cfg.buzzer_en ? "ON" : "MUTE"); break;
        case 6: snprintf(val, vlen,
                         (s_cfg.tone_sel == 0u) ? "LOW" :
                         (s_cfg.tone_sel == 1u) ? "MID" : "HIGH"); break;
        case 7: snprintf(val, vlen, reset_armed() ? "AGAIN!" : "PRESS+"); break;
        case 8: snprintf(val, vlen, "%upx", (unsigned)s_cfg.x_shift); break;
        default: break;
    }
}

void settings_factory_reset(void)
{
    s_cfg = s_defaults;
    settings_save();
    settings_apply_audio();
    settings_apply_visual();
    ESP_LOGI(TAG, "factory reset applied");
}
