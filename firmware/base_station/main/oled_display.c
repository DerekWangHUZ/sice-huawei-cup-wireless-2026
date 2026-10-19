/*
 * oled_display.c -- 1.3" 四脚 I2C OLED 驱动实现（SSD1306 / SH1106 通吃）
 *
 * 详见 oled_display.h 的设计说明与屏面布局。
 *
 * 点阵数据格式：
 *   帧缓存 s_fb[page][col] 与 SSD1306 显存一一对应：一页 8 行 x 128 列，
 *   列内 bit0 在最上方。故 16x16 汉字 = 同列相邻两页各 16 字节，直接位运算
 *   写入，不需要任何转置（见 font_cjk16.h 的格式说明）。
 */

#include "oled_display.h"
#include "board_pins.h"
#include "font_cjk16.h"
#include "settings.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "OLED";

/* 单次 I2C 传输超时（毫秒） */
#define OLED_XFER_TIMEOUT_MS  100

/* I2C 控制字节：0x00 = 后续是命令，0x40 = 后续是显存数据 */
#define OLED_CTRL_CMD   0x00u
#define OLED_CTRL_DATA  0x40u

/* 一页 128 列 = 128 字节显存 */
#define OLED_PAGE_BYTES  ((size_t)OLED_WIDTH)

/* 6x8 单元格：5 列字模 + 1 列间隔 -> 128/6 = 21 字符/行 */
#define FONT_W        5
#define FONT_CELL     6

/* 状态大字占 4 页（32 行），起始页固定为 2（标题栏占 0~1） */
#define OLED_BIG_PAGE 2u

/* ==========================================================================
 * 6x8 ASCII 字库（0x20~0x7E，共 95 字）
 *   每字 5 字节，列优先（每字节一列，bit0 在最上方），标准 5x7 点阵。
 * ========================================================================== */
static const uint8_t s_font5x7[95][FONT_W] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /* ' ' */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 }, /* '!' */
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, /* '"' */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, /* '#' */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, /* '$' */
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, /* '%' */
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, /* '&' */
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, /* '\'' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 }, /* '(' */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 }, /* ')' */
    { 0x08, 0x2A, 0x1C, 0x2A, 0x08 }, /* '*' */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* '+' */
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, /* ',' */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* '-' */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* '.' */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* '/' */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* '0' */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* '1' */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* '2' */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* '3' */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* '4' */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* '5' */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* '6' */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* '7' */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* '8' */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* '9' */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* ':' */
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, /* ';' */
    { 0x00, 0x08, 0x14, 0x22, 0x41 }, /* '<' */
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, /* '=' */
    { 0x41, 0x22, 0x14, 0x08, 0x00 }, /* '>' */
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, /* '?' */
    { 0x32, 0x49, 0x79, 0x41, 0x3E }, /* '@' */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* 'A' */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* 'B' */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* 'C' */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* 'D' */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* 'E' */
    { 0x7F, 0x09, 0x09, 0x01, 0x01 }, /* 'F' */
    { 0x3E, 0x41, 0x41, 0x51, 0x32 }, /* 'G' */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* 'H' */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* 'I' */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* 'J' */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* 'K' */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* 'L' */
    { 0x7F, 0x02, 0x04, 0x02, 0x7F }, /* 'M' */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* 'N' */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* 'O' */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* 'P' */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* 'Q' */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* 'R' */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* 'S' */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* 'T' */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* 'U' */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* 'V' */
    { 0x7F, 0x20, 0x18, 0x20, 0x7F }, /* 'W' */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* 'X' */
    { 0x03, 0x04, 0x78, 0x04, 0x03 }, /* 'Y' */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* 'Z' */
    { 0x00, 0x00, 0x7F, 0x41, 0x41 }, /* '[' */
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, /* '\\' */
    { 0x41, 0x41, 0x7F, 0x00, 0x00 }, /* ']' */
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, /* '^' */
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, /* '_' */
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, /* '`' */
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, /* 'a' */
    { 0x7F, 0x48, 0x44, 0x44, 0x38 }, /* 'b' */
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, /* 'c' */
    { 0x38, 0x44, 0x44, 0x48, 0x7F }, /* 'd' */
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, /* 'e' */
    { 0x08, 0x7E, 0x09, 0x01, 0x02 }, /* 'f' */
    { 0x08, 0x14, 0x54, 0x54, 0x3C }, /* 'g' */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 }, /* 'h' */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 }, /* 'i' */
    { 0x20, 0x40, 0x44, 0x3D, 0x00 }, /* 'j' */
    { 0x00, 0x7F, 0x10, 0x28, 0x44 }, /* 'k' */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 }, /* 'l' */
    { 0x7C, 0x04, 0x18, 0x04, 0x78 }, /* 'm' */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 }, /* 'n' */
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, /* 'o' */
    { 0x7C, 0x14, 0x14, 0x14, 0x08 }, /* 'p' */
    { 0x08, 0x14, 0x14, 0x18, 0x7C }, /* 'q' */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 }, /* 'r' */
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, /* 's' */
    { 0x04, 0x3F, 0x44, 0x40, 0x20 }, /* 't' */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C }, /* 'u' */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C }, /* 'v' */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C }, /* 'w' */
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, /* 'x' */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C }, /* 'y' */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 }, /* 'z' */
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, /* '{' */
    { 0x00, 0x00, 0x7F, 0x00, 0x00 }, /* '|' */
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, /* '}' */
    { 0x08, 0x04, 0x08, 0x10, 0x08 }, /* '~' */
};

/* 未收录汉字的占位：16x16 空心方框 */
static const uint8_t s_cjk_missing[CJK16_BYTES] = {
    0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF,
    0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF,
};

/* ==========================================================================
 * 内部状态
 * ========================================================================== */

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static bool        s_ready = false;

/* 帧缓存与"上次已推屏"的镜像，用于只刷发生变化的页 */
static uint8_t s_fb[OLED_PAGES][OLED_WIDTH];
static uint8_t s_shadow[OLED_PAGES][OLED_WIDTH];

static uint32_t s_err_count = 0;

/* 告警闪烁相位：每次刷新翻转一次（250ms 刷新 -> 2Hz 反色闪烁） */
static bool s_blink = false;

/* 面板左侧遮挡补偿：实测本屏左侧约 2 列被外框遮挡，右侧完整。
 * 绘制安全区宽度 = 128 - 左侧遮挡列数；内容绘制在 [0, s_draw_w)，
 * 推送时整体右移 s_x_shift 列落到 [s_x_shift, 128)，因此右侧满幅可用。 */
static uint8_t s_x_shift  = 2;
static uint8_t s_draw_w   = OLED_WIDTH - 2;
static bool    s_full_refresh = false;

/* 状态大字区的临时提示（2s 内显示，调用方负责清除）。
 * 容量按最长提示词给足：「人工解除?」= 4 个汉字(12B) + 1 个 ASCII(1B) = 13B，
 * 取 24B 留余量，避免 snprintf 把末尾汉字截断。 */
static char s_hint[24];

void oled_set_hint(const char *utf8)
{
    if (utf8 == NULL || utf8[0] == '\0') {
        s_hint[0] = '\0';
    } else {
        snprintf(s_hint, sizeof(s_hint), "%s", utf8);
    }
}

/* 反色标题栏：居中 CJK 标题（页码标识已取消，各页不再显示 x/5） */
static void draw_title(uint8_t page_top, const char *cjk, uint8_t page_no)
{
    const char *p = cjk;
    uint8_t n = 0;
    uint8_t x;

    (void)page_no;                       /* 形参保留以兼容既有调用点 */

    while (*p != '\0') {
        if ((uint8_t)*p >= 0x80u) { p += 3; n++; }   /* UTF-8 中文 3 字节 */
        else                                     { p += 1; }
    }

    oled_fill_page(page_top, 0xFF);
    oled_fill_page((uint8_t)(page_top + 1u), 0xFF);
    x = (uint8_t)((s_draw_w - n * CJK16_W) / 2u);
    oled_draw_text_inv(page_top, x, cjk);
}

/* ==========================================================================
 * 底层 I2C 收发
 * ========================================================================== */

/* 下发 1~2 条命令（同一 I2C 事务，控制字节 0x00） */
static esp_err_t oled_tx_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t buf[4];

    if (!s_ready || s_dev == NULL || n == 0u || n > sizeof(buf) - 1u) {
        return ESP_ERR_INVALID_STATE;
    }
    buf[0] = OLED_CTRL_CMD;
    memcpy(&buf[1], cmds, n);
    return i2c_master_transmit(s_dev, buf, n + 1u, OLED_XFER_TIMEOUT_MS);
}

static esp_err_t oled_tx_cmd1(uint8_t c0)
{
    uint8_t cmds[1] = { c0 };
    return oled_tx_cmds(cmds, 1);
}

static esp_err_t oled_tx_cmd2(uint8_t c0, uint8_t c1)
{
    uint8_t cmds[2] = { c0, c1 };
    return oled_tx_cmds(cmds, 2);
}

/* 写一页显存：命令(页地址 + 列地址) + 128 字节数据 */
static esp_err_t oled_write_page(uint8_t page, const uint8_t *data)
{
    uint8_t  buf[1 + OLED_WIDTH];
    uint8_t  cmds[3];
    esp_err_t err;

    if (!s_ready || s_dev == NULL || page >= OLED_PAGES) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 页寻址模式：0xB0|page 设定页，随后两个半字节设定列起始。
     * OLED_COL_OFFSET 对 SSD1306 为 0、对 SH1106 为 2（132 列 RAM 的可见区）。
     */
    cmds[0] = (uint8_t)(0xB0u | page);
    cmds[1] = (uint8_t)(0x00u | (OLED_COL_OFFSET & 0x0Fu));
    cmds[2] = (uint8_t)(0x10u | ((OLED_COL_OFFSET >> 4) & 0x0Fu));

    err = oled_tx_cmds(cmds, sizeof(cmds));
    if (err != ESP_OK) {
        return err;
    }

    buf[0] = OLED_CTRL_DATA;
    memcpy(&buf[1], data, OLED_WIDTH);
    return i2c_master_transmit(s_dev, buf, sizeof(buf), OLED_XFER_TIMEOUT_MS);
}

/* 统一的错误记账：不 abort、不刷屏，只计数并按低频打日志 */
static void oled_note_error(const char *what, esp_err_t err)
{
    s_err_count++;
    if (s_err_count == 1u || (s_err_count % 100u) == 0u) {
        ESP_LOGW(TAG, "%s failed: %s (total errors=%u)",
                 what, esp_err_to_name(err), (unsigned)s_err_count);
    }
}

/* ==========================================================================
 * 面板初始化
 * ========================================================================== */

static esp_err_t oled_panel_init_sequence(void)
{
    /*
     * 逐条下发初始化命令。两种控制器的差异只有两处：
     *   1) 电荷泵：SSD1306 = 0x8D/0x14，SH1106 = 0xAD/0x8B；
     *   2) 寻址模式：SSD1306 显式设页寻址(0x20,0x02)，
     *      SH1106 只有页寻址、无 0x20/0x21/0x22 命令。
     * 其余命令两者兼容。
     */
#define OLED_TRY(expr)                       \
    do {                                     \
        esp_err_t e_ = (expr);               \
        if (e_ != ESP_OK) {                  \
            return e_;                       \
        }                                    \
    } while (0)

    OLED_TRY(oled_tx_cmd1(0xAE));            /* display off */
    OLED_TRY(oled_tx_cmd2(0xD5, 0x80));      /* clock div / osc freq */
    OLED_TRY(oled_tx_cmd2(0xA8, 0x3F));      /* multiplex = 64 */
    OLED_TRY(oled_tx_cmd2(0xD3, 0x00));      /* display offset 0 */
    OLED_TRY(oled_tx_cmd1(0x40));            /* start line 0 */

#if OLED_CTRL_SH1106
    OLED_TRY(oled_tx_cmd2(0xAD, 0x8B));      /* DC-DC charge pump ON */
#else
    OLED_TRY(oled_tx_cmd2(0x8D, 0x14));      /* charge pump ON */
    OLED_TRY(oled_tx_cmd2(0x20, 0x02));      /* memory addressing = PAGE */
#endif

    OLED_TRY(oled_tx_cmd1(0xA1));            /* segment remap */
    OLED_TRY(oled_tx_cmd1(0xC8));            /* COM scan dir remapped */
    OLED_TRY(oled_tx_cmd2(0xDA, 0x12));      /* COM pins config */
    OLED_TRY(oled_tx_cmd2(0x81, 0xCF));      /* contrast */
#if OLED_CTRL_SH1106
    OLED_TRY(oled_tx_cmd2(0xD9, 0x22));      /* pre-charge period */
#else
    OLED_TRY(oled_tx_cmd2(0xD9, 0xF1));      /* pre-charge period */
#endif
    OLED_TRY(oled_tx_cmd2(0xDB, 0x40));      /* VCOMH deselect */
    OLED_TRY(oled_tx_cmd1(0xA4));            /* resume from RAM */
    OLED_TRY(oled_tx_cmd1(0xA6));            /* normal (not inverted) */
    OLED_TRY(oled_tx_cmd1(0xAF));            /* display on */

#undef OLED_TRY
    return ESP_OK;
}

esp_err_t oled_init(void)
{
    esp_err_t err;
    uint8_t   addr = 0;

    memset(s_fb, 0, sizeof(s_fb));
    memset(s_shadow, 0xFF, sizeof(s_shadow));  /* 与清屏后不同 -> 首帧全刷 */

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = OLED_I2C_PORT,
        .sda_io_num        = (gpio_num_t)OLED_SDA_GPIO,
        .scl_io_num        = (gpio_num_t)OLED_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 依次探测常见地址；都无应答说明模块未接或接线有误 */
    if (i2c_master_probe(s_bus, OLED_ADDR_PRIMARY,
                         OLED_PROBE_TIMEOUT_MS) == ESP_OK) {
        addr = OLED_ADDR_PRIMARY;
    } else if (i2c_master_probe(s_bus, OLED_ADDR_SECONDARY,
                                OLED_PROBE_TIMEOUT_MS) == ESP_OK) {
        addr = OLED_ADDR_SECONDARY;
    } else {
        ESP_LOGW(TAG, "no panel at 0x%02X/0x%02X, display disabled",
                 OLED_ADDR_PRIMARY, OLED_ADDR_SECONDARY);
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = OLED_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return err;
    }

    s_ready = true;   /* 必须先置位，oled_tx_* 才允许发送 */

    err = oled_panel_init_sequence();
    if (err != ESP_OK) {
        oled_note_error("init sequence", err);
        s_ready = false;
        return err;
    }

    /*
     * 首帧：清屏并整屏推一次，确保上电后屏上不是随机雪花。
     * 注意 s_shadow 已被填成 0xFF，flush 会写全部 8 页。
     */
    oled_clear();
    err = oled_flush();

    ESP_LOGI(TAG, "panel ready: addr=0x%02X ctrl=%s size=%dx%d cjk=%u",
             addr, OLED_CTRL_SH1106 ? "SH1106" : "SSD1306",
             OLED_WIDTH, OLED_HEIGHT, (unsigned)CJK16_GLYPHS);
    return err;
}

bool oled_ready(void)
{
    return s_ready;
}

/* ==========================================================================
 * 帧缓存绘制
 * ========================================================================== */

void oled_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void oled_fill_page(uint8_t page, uint8_t value)
{
    if (page >= OLED_PAGES) {
        return;
    }
    memset(s_fb[page], value, OLED_WIDTH);
}

/* UTF-8 解码：返回码点并推进指针；非法序列返回 0xFFFD 且至少前进 1 字节 */
static uint32_t utf8_decode(const char **pp)
{
    const uint8_t *s = (const uint8_t *)*pp;
    uint32_t       cp;
    int            extra;
    int            i;

    if (s[0] < 0x80u) {
        *pp += 1;
        return (uint32_t)s[0];
    } else if ((s[0] & 0xE0u) == 0xC0u) {
        cp = (uint32_t)(s[0] & 0x1Fu);
        extra = 1;
    } else if ((s[0] & 0xF0u) == 0xE0u) {
        cp = (uint32_t)(s[0] & 0x0Fu);
        extra = 2;
    } else if ((s[0] & 0xF8u) == 0xF0u) {
        cp = (uint32_t)(s[0] & 0x07u);
        extra = 3;
    } else {
        *pp += 1;
        return 0xFFFDu;
    }

    for (i = 1; i <= extra; i++) {
        if ((s[i] & 0xC0u) != 0x80u) {     /* 后续字节格式错：只前进 1 字节 */
            *pp += 1;
            return 0xFFFDu;
        }
        cp = (cp << 6) | (uint32_t)(s[i] & 0x3Fu);
    }
    *pp += (extra + 1);
    return cp;
}

/* 查 16x16 字模；未收录返回 NULL */
static const uint8_t *cjk16_lookup(uint32_t cp)
{
    size_t i;

    for (i = 0; i < CJK16_GLYPHS; i++) {
        if ((uint32_t)s_cjk16[i].cp == cp) {
            return s_cjk16[i].bits;
        }
    }
    return NULL;
}

/* 把 16x16 字模写入帧缓存（跨 page/page+1）。inv=true 时清位（反色） */
static void blit16(uint8_t page, uint8_t x, const uint8_t *bits, bool inv)
{
    uint8_t h;
    uint8_t c;

    if (bits == NULL || (uint16_t)x + CJK16_W > s_draw_w ||
        (uint16_t)page + 1u >= OLED_PAGES) {
        return;
    }

    for (h = 0; h < 2u; h++) {
        uint8_t *dst = s_fb[page + h];
        for (c = 0; c < CJK16_W; c++) {
            uint8_t b = bits[(size_t)h * CJK16_W + c];
            if (inv) {
                dst[x + c] &= (uint8_t)~b;
            } else {
                dst[x + c] |= b;
            }
        }
    }
}

/* 把 16x16 字模放大 2 倍（32x32，占 4 页）写入帧缓存。
 * clip_w：允许绘制的最大列数（右边缘裁剪）。传 2*CJK16_W 表示不裁剪。
 * 裁剪的目的是让「4 个汉字 = 128 列」这类超出可用宽度的文本也能全部画出来，
 * 只损失右边缘少量列，而不是把末尾整字丢弃。 */
static void blit16_2x_clip(uint8_t page, uint8_t x, const uint8_t *bits,
                           bool inv, uint8_t clip_w)
{
    uint8_t h;
    uint8_t c;
    uint8_t r;
    uint8_t limit;

    if (bits == NULL || (uint16_t)page + 3u >= OLED_PAGES) {
        return;
    }
    if (x >= s_draw_w) {
        return;
    }
    /* 实际可绘制的列数 = min(clip_w, 剩余宽度) */
    limit = (uint8_t)(s_draw_w - x);
    if (clip_w < limit) {
        limit = clip_w;
    }
    if (limit == 0u) {
        return;
    }

    for (h = 0; h < 2u; h++) {                 /* 源上半页 / 下半页 */
        for (c = 0; c < CJK16_W; c++) {
            uint8_t src = bits[(size_t)h * CJK16_W + c];
            uint8_t hi  = 0;                   /* 放大后该半页的上 8 行 */
            uint8_t lo  = 0;                   /* 放大后该半页的下 8 行 */

            for (r = 0; r < 4u; r++) {
                if (src & (uint8_t)(1u << r)) {
                    hi |= (uint8_t)(3u << (2u * r));
                }
            }
            for (r = 4u; r < 8u; r++) {
                if (src & (uint8_t)(1u << r)) {
                    lo |= (uint8_t)(3u << (2u * (r - 4u)));
                }
            }

            /* 横向同样放大 2 倍：相邻两列写同样的字节 */
            for (r = 0; r < 2u; r++) {
                uint8_t col = (uint8_t)(c * 2u + r);
                uint8_t *d0;
                uint8_t *d1;

                if (col >= limit) {
                    break;                     /* 超出裁剪宽度，停止本字剩余列 */
                }
                d0 = &s_fb[page + h * 2u][x + col];
                d1 = &s_fb[page + h * 2u + 1u][x + col];
                if (inv) {
                    *d0 &= (uint8_t)~hi;
                    *d1 &= (uint8_t)~lo;
                } else {
                    *d0 |= hi;
                    *d1 |= lo;
                }
            }
        }
    }
}

/* 在页内按列绘制一个 ASCII 字符；超出右边界的列直接丢弃（截断） */
static void draw_char_ex(uint8_t page, uint8_t x, char ch, bool inv)
{
    uint8_t       idx;
    const uint8_t *glyph;
    uint8_t       i;

    if (x + FONT_CELL > s_draw_w) {
        return;                       /* 越界整字丢弃 */
    }

    if ((uint8_t)ch < 0x20u || (uint8_t)ch > 0x7Eu) {
        ch = '?';                     /* 非可打印字符统一显示 ? */
    }
    idx   = (uint8_t)((uint8_t)ch - 0x20u);
    glyph = s_font5x7[idx];

    for (i = 0; i < FONT_W; i++) {
        if (inv) {
            s_fb[page][x + i] &= (uint8_t)~glyph[i];
        } else {
            s_fb[page][x + i] |= glyph[i];
        }
    }
    if (!inv) {
        s_fb[page][x + FONT_W] = 0x00;    /* 字符间隔列（顺色时清掉） */
    }
}

void oled_draw_row(uint8_t page, const char *text)
{
    uint8_t x = 0;

    if (page >= OLED_PAGES || text == NULL) {
        return;
    }
    while (*text != '\0' && (uint8_t)(x + FONT_CELL) <= s_draw_w) {
        draw_char_ex(page, x, *text, false);
        x = (uint8_t)(x + FONT_CELL);
        text++;
    }
}

/* 混排文本的公共实现 */
static void draw_text_ex(uint8_t page, uint8_t x, const char *utf8, bool inv)
{
    if (utf8 == NULL || page >= OLED_PAGES) {
        return;
    }

    while (*utf8 != '\0') {
        uint32_t cp = utf8_decode(&utf8);
        if (cp == 0u || cp == 0xFFFDu) {
            continue;                       /* 非法字节已前进，直接跳过 */
        }

        if (cp < 0x80u) {                   /* ASCII：6x8 单元格 */
            if ((uint16_t)x + FONT_CELL > s_draw_w) {
                break;
            }
            draw_char_ex(page, x, (char)cp, inv);
            x = (uint8_t)(x + FONT_CELL);
        } else {                            /* 汉字：16x16，跨两页 */
            const uint8_t *bits = cjk16_lookup(cp);
            if (bits == NULL) {
                bits = s_cjk_missing;
            }
            if ((uint16_t)x + CJK16_W > s_draw_w) {
                break;
            }
            blit16(page, x, bits, inv);
            x = (uint8_t)(x + CJK16_W);
        }
    }
}

void oled_draw_text(uint8_t page, uint8_t x, const char *utf8)
{
    draw_text_ex(page, x, utf8, false);
}

void oled_draw_text_inv(uint8_t page, uint8_t x, const char *utf8)
{
    draw_text_ex(page, x, utf8, true);
}

void oled_draw_big_text(uint8_t page, const char *utf8, bool inv)
{
    const char *p;
    uint16_t    width = 0;
    uint8_t     n = 0;
    uint8_t     x;

    if (utf8 == NULL) {
        return;
    }

    /* 第一遍：量宽度（每字 32 列） */
    p = utf8;
    while (*p != '\0') {
        uint32_t cp = utf8_decode(&p);
        if (cp == 0u || cp == 0xFFFDu || cp < 0x80u) {
            continue;
        }
        width = (uint16_t)(width + 2u * CJK16_W);
        n++;
    }
    if (n == 0u) {
        return;
    }

    /*
     * 单字步进：正常为 32 列（16x16 放大 2 倍）。
     * 当 n 个字排不下（如 4 个汉字需 128 列，而 X-SHIFT 让可用宽度只剩 126）时，
     * 按可用宽度均分步进，让所有字都画得出来；最后一个字的右边缘会被裁剪
     * 少量列（通常 2 列），但字形主体完整可辨，不会整字消失。
     * 步进下限 24 列，再小字会互相压叠、不可辨认。
     */
    uint16_t step = (uint16_t)(2u * CJK16_W);
    if ((uint16_t)(n * step) > s_draw_w) {
        uint16_t fit = (uint16_t)(s_draw_w / n);
        if (fit >= 24u) {
            step = fit;
        }
    }

    uint16_t total = (uint16_t)(n * step);
    x = (total >= s_draw_w) ? 0u : (uint8_t)((s_draw_w - total) / 2u);

    /* 第二遍：绘制（用裁剪版本，保证末尾字不被整字丢弃） */
    p = utf8;
    while (*p != '\0') {
        uint32_t cp = utf8_decode(&p);
        if (cp == 0u || cp == 0xFFFDu || cp < 0x80u) {
            continue;
        }
        const uint8_t *bits = cjk16_lookup(cp);
        if (bits == NULL) {
            bits = s_cjk_missing;
        }
        if (x >= s_draw_w) {
            break;
        }
        blit16_2x_clip(page, x, bits, inv, (uint8_t)(2u * CJK16_W));
        x = (uint8_t)(x + step);
    }
}

/* 面板左侧遮挡补偿：由 oled_set_x_shift() 调整 s_x_shift / s_draw_w */
void oled_set_x_shift(uint8_t shift)
{
    if (shift > 6u) {
        shift = 6u;
    }
    if (shift != s_x_shift) {
        s_x_shift      = shift;
        s_draw_w       = (uint8_t)(OLED_WIDTH - shift);
        s_full_refresh = true;      /* 偏移变化后必须整屏重刷 */
    }
}

void oled_force_full_refresh(void)
{
    s_full_refresh = true;
}

/* 把 16x16 字模按带符号 x 写入帧缓存（允许 x<0：左侧越界列整列跳过）。
 * 供横向滚动文本使用；不改变既有 blit16 的行为。 */
static void blit16_clip(uint8_t page, int16_t x, const uint8_t *bits, bool inv)
{
    uint8_t h;
    uint8_t c;

    if (bits == NULL || (uint16_t)page + 1u >= OLED_PAGES) {
        return;
    }
    for (h = 0; h < 2u; h++) {
        uint8_t *dst = s_fb[page + h];
        for (c = 0; c < CJK16_W; c++) {
            int16_t dx = (int16_t)(x + (int16_t)c);
            uint8_t b  = bits[(size_t)h * CJK16_W + c];
            if (dx < 0 || dx >= (int16_t)s_draw_w) {
                continue;                    /* 越界列丢弃（裁剪） */
            }
            if (inv) {
                dst[dx] &= (uint8_t)~b;
            } else {
                dst[dx] |= b;
            }
        }
    }
}


/* 在页内绘制单个字符，允许带符号 x（越界列丢弃）。供滚动文本使用。 */
static void draw_char_clip(uint8_t page, int16_t x, char ch)
{
    uint8_t idx = (uint8_t)ch;
    uint8_t i;
    const uint8_t *glyph;

    if (idx < 0x20u || idx > 0x7Eu) {
        idx = (uint8_t)'?';
    }
    glyph = s_font5x7[idx - 0x20u];
    for (i = 0; i < FONT_W; i++) {
        int16_t dx = (int16_t)(x + (int16_t)i);
        if (dx < 0 || dx >= (int16_t)s_draw_w) {
            continue;
        }
        s_fb[page][dx] |= glyph[i];
    }
}

/* 在指定页横向滚动绘制中英混排文本。
 *   x_off  当前滚动偏移（可为负，负值表示文本已滚出左侧）
 * 返回文本像素总宽度。
 *
 * 注意：ASCII 必须真正绘制（6 列/字），不能只占位——否则 " | " 分隔符
 * 与 "FPGA:" / "GPIO18" 这类英文片段在屏幕上会凭空消失。 */
static uint16_t marquee_cjk(uint8_t page, int16_t x_off, const char *utf8)
{
    int16_t  x = x_off;
    uint16_t w = 0;

    if (utf8 == NULL) {
        return 0;
    }
    while (*utf8 != '\0') {
        uint32_t cp = utf8_decode(&utf8);
        if (cp == 0u || cp == 0xFFFDu) {
            continue;
        }
        if (cp < 0x80u) {
            draw_char_clip(page, x, (char)cp);      /* 真正绘制 6x8 ASCII */
            x = (int16_t)(x + FONT_CELL);
            w = (uint16_t)(w + FONT_CELL);
            continue;
        }
        {
            const uint8_t *bits = cjk16_lookup(cp);
            if (bits == NULL) {
                bits = s_cjk_missing;
            }
            blit16_clip(page, x, bits, false);
            x = (int16_t)(x + CJK16_W);
            w = (uint16_t)(w + CJK16_W);
        }
    }
    return w;
}


/* 追加一段文本；若缓冲区已有内容，先用中文分隔符「、」隔开。
 *
 * 为什么不用 " | "：状态区是 16x16 中文行（占 2 页），而 ASCII 只画在
 * 上页 8 行，混排时竖线只出现在上半页、看起来"上下断开错位"；
 * 且 ASCII 与中文宽度/基线不一致会让第二项起字形错乱。
 * 纯中文顿号在同一字模体系内，宽度与基线天然对齐。 */
static void cat_sep(char *buf, size_t len, const char *item)
{
    size_t n = strlen(buf);

    if (n != 0u && n + 3u < len) {
        strncat(buf, "丨", len - n - 1u);   /* 全高竖杠，与中文同高 */
        n += 3u;
    }
    if (item != NULL && n < len) {
        strncat(buf, item, len - n - 1u);
    }
}


/* 单行文本自适应绘制：放得下就静态居中/左对齐，放不下就在该页横向滚动。
 * off 由调用方持有（static），row 为 0..7 的页号。
 * 返回值无意义（保持调用点简洁）。 */
static void draw_row_autoscroll(uint8_t page, int16_t *off, const char *text)
{
    uint16_t w = (uint16_t)(strlen(text) * 6u);   /* 6x8 等宽 */

    if (w <= s_draw_w) {
        *off = 0;
        oled_draw_row(page, text);
        return;
    }
    *off = (int16_t)(*off - 5);
    if (*off < -(int16_t)w) {
        *off = (int16_t)s_draw_w;
    }
    {
        /* 逐字符按列偏移绘制（负偏移时跳过左侧越界字符） */
        int16_t x = *off;
        const char *q = text;
        while (*q != '\0') {
            if (x >= -(int16_t)FONT_CELL && x < (int16_t)s_draw_w) {
                draw_char_clip(page, x, *q);
            }
            x = (int16_t)(x + FONT_CELL);
            q++;
        }
    }
}

/* 中文/ASCII 混排文本的像素宽度 */
static uint16_t text_width(const char *utf8)
{
    uint16_t w = 0;

    if (utf8 == NULL) {
        return 0;
    }
    while (*utf8 != '\0') {
        if ((uint8_t)*utf8 >= 0x80u) { utf8 += 3; w = (uint16_t)(w + CJK16_W); }
        else                         { utf8 += 1; w = (uint16_t)(w + 6u); }
    }
    return w;
}

/* 推进滚动偏移：未超宽恒为 0（静止显示），超宽则每帧左移 1 列，
 * 滚完一轮后从右端重新进场。 */
static int16_t marquee_next(int16_t off, const char *utf8)
{
    uint16_t need = text_width(utf8);

    if (need <= s_draw_w) {
        return 0;
    }
    off = (int16_t)(off - 5);   /* 250ms 一帧滚 5 列（20 列/秒） */
    if (off < -(int16_t)need) {
        off = (int16_t)s_draw_w;
    }
    return off;
}

esp_err_t oled_flush(void)
{
    uint8_t   page;
    esp_err_t first_err = ESP_OK;
    uint8_t   out[OLED_WIDTH];
    uint8_t   x;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 边缘补偿在下面的逐页循环里完成：安全区内容右移 s_x_shift 列，
     * 被遮挡的左边缘与右边多出的列留黑，确保没有任何笔画落在不可见区。 */
    if (s_full_refresh) {
        memset(s_shadow, 0xFF, sizeof(s_shadow));   /* 强制与内容不同 */
        s_full_refresh = false;
    }

    for (page = 0; page < OLED_PAGES; page++) {
        const uint8_t *src;

        if (s_x_shift == 0u) {
            src = s_fb[page];
        } else {
            memset(out, 0x00, s_x_shift);
            for (x = 0; x + s_x_shift < OLED_WIDTH; x++) {
                out[x + s_x_shift] = s_fb[page][x];
            }
            src = out;
        }

        /* 内容未变的页不占 I2C 带宽（状态屏多数页是静态的） */
        if (memcmp(src, s_shadow[page], OLED_WIDTH) == 0) {
            continue;
        }
        esp_err_t err = oled_write_page(page, src);
        if (err != ESP_OK) {
            oled_note_error("write_page", err);
            if (first_err == ESP_OK) {
                first_err = err;
            }
            continue;                /* 其余页继续尝试 */
        }
        memcpy(s_shadow[page], src, OLED_WIDTH);
    }
    return first_err;
}

/* ==========================================================================
 * 状态屏渲染
 *
 * 遥测行图例：
 *   PKT n SEQ m      收包总数 / 最近一帧序号（判断穿戴端是否在推流）
 *   SD:xx F:xx a/b   SD 是否在记录 / FPGA 链路是否就绪 / 坏帧数 与 SD 丢弃数
 * ========================================================================== */

esp_err_t oled_show_status(const oled_status_t *s)
{
    /* 大号状态词：索引 = fall_state_t（0 静止 / 1 走动 / 2 待确认 / 3 已确认） */
    static const char *const STATE_TXT[4] = {
        "正常", "行走", "疑似跌倒", "跌倒告警"
    };

    /*
     * row 容量按"最长可能输出"给足（最长为 "PKT 4294967295 SEQ 65535" = 25），
     * 取 48 留裕量，避免 -Wformat-truncation 报错；
     * 实际每行只画 21 个字符，超出部分由 oled_draw_row 截断。
     */
    char        row[48];
    const char *big;
    bool        blink_inv = false;
    uint8_t     st;

    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    oled_clear();

    /* --- y0~15：实底反色标题栏「跌倒检测基站」（主页不画页码） --- */
    draw_title(0, "跌倒检测基站", 0);

    /* --- y16~47：32x32 大号状态；告警时整条反色闪烁（2Hz）；
     *     解除提示期间（s_hint）以单倍字号居中显示提示词 --- */
    if (s_hint[0] != '\0') {
        const char *p = s_hint;
        uint8_t n = 0, x;
        while (*p != '\0') {
            if ((uint8_t)*p >= 0x80u) { p += 3; n++; }
            else                      { p += 1; }
        }
        x = (uint8_t)((s_draw_w - n * CJK16_W) / 2u);
        oled_draw_text(3, x, s_hint);          /* 单倍字号，pages 3~4 */
    } else {
        if (s->alarm) {
            s_blink   = !s_blink;          /* 每次刷新翻转 -> 250ms 刷新时为 2Hz */
            blink_inv = s_blink;
            big       = STATE_TXT[3];
        } else {
            st  = ((uint8_t)s->state < 4u) ? (uint8_t)s->state : 0u;
            big = STATE_TXT[st];
        }
        if (blink_inv) {
            /* 反色阶段：先把 4 页铺成实底，再清出笔画 -> 白底黑字 */
            oled_fill_page(OLED_BIG_PAGE + 0u, 0xFF);
            oled_fill_page(OLED_BIG_PAGE + 1u, 0xFF);
            oled_fill_page(OLED_BIG_PAGE + 2u, 0xFF);
            oled_fill_page(OLED_BIG_PAGE + 3u, 0xFF);
        }
        oled_draw_big_text(OLED_BIG_PAGE, big, blink_inv);
    }

    /* --- y48~63：硬件接入状态区（中文，异常必须讲清原因）
     *   全部正常：page6「接入正常」+ page7「全部设备已就绪」（居中不滚动）
     *   有异常  ：page6 具体原因（滚动）+ page7 处置建议（滚动）
     * 收包计数与 SD 统计已移到 P2 系统页。 --- */
    {
        static int16_t s_off6 = 0, s_off7 = 0;
        bool  bad_udp  = !s->data_fresh;
        bool  bad_fpga = !s->fpga_ok;
        bool  bad_ui   = !s->ui_alive;
        bool  bad_sd   = !s->sd_ok;
        uint8_t nbad = (uint8_t)((bad_udp ? 1u : 0u) + (bad_fpga ? 1u : 0u) +
                                 (bad_ui ? 1u : 0u) + (bad_sd ? 1u : 0u));

        if (s->alarm && s->has_impact) {
            goto status_rows_done;
        }

        if (nbad == 0u) {
            s_off6 = s_off7 = 0;
            oled_draw_text(6, (uint8_t)((s_draw_w - 4u * CJK16_W) / 2u),
                           "全部正常");
            oled_draw_text(7, (uint8_t)((s_draw_w - 5u * CJK16_W) / 2u),
                           "设备就绪");
        } else {
            /* 两行滚动：上行=哪个设备坏了，下行=怎么办 */
            char reason[64];
            char advice[64];

            /* 文案规则：只用中文与竖杠「丨」，不使用任何其它标点，
             * 不使用 ASCII（ASCII 只占 8 行高，与 16 行中文字模不匹配）。 */
            reason[0] = '\0';
            advice[0] = '\0';
            if (bad_udp) {
                strncat(reason, "穿戴端无数据",
                        sizeof(reason) - strlen(reason) - 1u);
                strncat(advice, "查穿戴端电源",
                        sizeof(advice) - strlen(advice) - 1u);
            }
            if (bad_fpga) {
                cat_sep(reason, sizeof(reason), "串口断开");
                cat_sep(advice, sizeof(advice), "查两板连线");
            }
            if (bad_ui) {
                cat_sep(reason, sizeof(reason), "键盘未连接");
                cat_sep(advice, sizeof(advice), "查键盘排线");
            }
            if (bad_sd) {
                cat_sep(reason, sizeof(reason), "存储卡未插入");
                cat_sep(advice, sizeof(advice), "插好卡后上电");
            }

            s_off6 = marquee_next(s_off6, reason);
            marquee_cjk(6, s_off6, reason);
            s_off7 = marquee_next(s_off7, advice);
            marquee_cjk(7, s_off7, advice);
        }

status_rows_done:
        if (s->alarm && s->has_impact) {
            /* 数字/小数点/g 已收录为 16x16 全高字模（由 5x7 ASCII 放大而来），
             * 因此可与中文同高混排，"3.40g" 保持可读格式。 */
            snprintf(row, sizeof(row), "冲击%u.%02ug",
                     (unsigned)(s->impact_mag_x100 / 100u),
                     (unsigned)(s->impact_mag_x100 % 100u));
            oled_draw_text(6, 0, row);
            if (s->alarm_start_ms != 0u) {
                int64_t now_ms = esp_timer_get_time() / 1000LL;
                int64_t dur = now_ms - (int64_t)s->alarm_start_ms;
                if (dur < 0) dur = 0;
                snprintf(row, sizeof(row), "已持续%lu秒%s",
                         (unsigned long)(dur / 1000LL),
                         s->mute_effective ? "丨静音中" : "");
            } else {
                snprintf(row, sizeof(row), "%s",
                         s->mute_effective ? "静音中" : "");
            }
            oled_draw_text(7, 0, row);
        }
    }

    return oled_flush();
}

/* ==========================================================================
 * 多页面扩展（P1 遥测 / P2 系统 / P3 设置 / P4 演示）
 *
 * 全部复用既有绘制原语（页寻址差量刷新、CJK 混排、反色高亮），
 * 新增字模见 font_cjk16.h（冲击/设置/演示模式/场景/静止/灯/自检/剧本/
 * 遥测(遥)/系统(系统)/解除 等，由 tools/font_gen/gen_cjk16.ps1 生成）。
 * ========================================================================== */

/* 运行期设置面板对比度（0x81 命令） */
esp_err_t oled_set_contrast(uint8_t val)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    return oled_tx_cmd2(0x81u, val);
}

/* 带符号定点（0.01g 或 0.1dps）-> "+N.NN" / "-N.N" 文本 */
static void fmt_q(char *buf, size_t len, int32_t v, int32_t scale)
{
    int32_t a = (v < 0) ? -v : v;
    snprintf(buf, len, "%s%ld.%02ld", (v < 0) ? "-" : "+",
             (long)(a / scale), (long)(a % scale));
}

/* P1 遥测页 */
void oled_show_telemetry(const oled_status_t *s,
                         const uint8_t *mag_hist, uint8_t hist_n)
{
    char   row[24];
    char   q[12];
    uint8_t i;
    int     fresh = (int)s->data_fresh;

    oled_clear();

    /* 反色标题栏「遥测」+ 页码 */
    draw_title(0, "遥测", 2);

    /* 合加速度幅值 + 状态（数据流停摆显示 --，与 FPGA 侧 3s 老化同语义） */
    if (fresh) {
        snprintf(row, sizeof(row), "a=%2u.%02ug %s",
                 (unsigned)(s->mag_x100 / 100u),
                 (unsigned)(s->mag_x100 % 100u),
                 fall_state_name(s->state));
    } else {
        snprintf(row, sizeof(row), "a= --    %s",
                 fall_state_name(s->state));
    }
    oled_draw_row(2, row);

    /* 三轴加速度（原始值 -> 0.01g） */
    if (fresh) {
        fmt_q(q, sizeof(q), ((int32_t)s->acc[0] * 100) / 2048, 100);
        snprintf(row, sizeof(row), "X%s ", q);
    } else {
        snprintf(row, sizeof(row), "X --");
    }
    oled_draw_row(3, row);
    if (fresh) {
        fmt_q(q, sizeof(q), ((int32_t)s->acc[1] * 100) / 2048, 100);
        snprintf(row, sizeof(row), "Y%s ", q);
    } else {
        snprintf(row, sizeof(row), "Y --");
    }
    oled_draw_row(4, row);

    /* 角速度合成（0.1dps）：|w| = isqrt(gx^2+gy^2+gz^2) 整数开方 */
    if (fresh) {
        int32_t gx = s->gyro[0], gy = s->gyro[1], gz = s->gyro[2];
        int32_t g2 = gx * gx + gy * gy + gz * gz;
        int32_t r = 0, bit = 1 << 30;
        int32_t wx;
        while (bit > g2) bit >>= 2;
        while (bit != 0) {
            if (g2 >= r + bit) { g2 -= r + bit; r = (r >> 1) + bit; }
            else               { r >>= 1; }
            bit >>= 2;
        }
        wx = (int32_t)(((int64_t)r * 10000L) / 16384L);   /* 0.1dps */
        snprintf(row, sizeof(row), "w=%ld.%01lddps BAD %lu",
                 (long)(wx / 10), (long)(wx % 10),
                 (unsigned long)s->frames_bad);
    } else {
        snprintf(row, sizeof(row), "w= --      BAD %lu",
                 (unsigned long)s->frames_bad);
    }
    oled_draw_row(5, row);

    /* 帧率 + 序号 */
    snprintf(row, sizeof(row), "FPS %u.%u  SEQ %u",
             (unsigned)(s->fps_x10 / 10u), (unsigned)(s->fps_x10 % 10u),
             (unsigned)s->last_seq);
    oled_draw_row(6, row);

    /* 幅值历史条形图（2 页 16px 高，末元素最右柱）；停摆期已由 ui 清零 */
    if (mag_hist != NULL && hist_n > 0) {
        for (i = 0; i < hist_n && i < s_draw_w; i++) {
            uint8_t lvl = mag_hist[hist_n - 1u - i];
            uint8_t x   = (uint8_t)(s_draw_w - 1u - i);
            uint8_t h   = (uint8_t)(lvl * 16u / 15u);
            uint8_t mask_top    = (h <= 8u) ? 0x00 : (uint8_t)(0xFFu << (16u - h));
            uint8_t mask_bottom = (h >= 8u) ? 0xFF : (uint8_t)((1u << h) - 1u);
            s_fb[4][x] = mask_top;
            s_fb[5][x] = mask_bottom;
        }
    }

    (void)oled_flush();
}

/* P2 系统页 */
void oled_show_system(const oled_status_t *s, const char *ssid, int channel)
{
    char row[40];
    int64_t up_s = esp_timer_get_time() / 1000000LL;

    oled_clear();

    draw_title(0, "系统", 3);

    /* 每行都走自适应：放得下静态显示，放不下自动横向滚动 */
    static int16_t o2 = 0, o3 = 0, o4 = 0, o5 = 0, o6 = 0, o7 = 0;

    snprintf(row, sizeof(row), "IP192.168.4.1 CH%d", channel);
    draw_row_autoscroll(2, &o2, row);

    snprintf(row, sizeof(row), "UP %02ld:%02ld:%02lu %s",
             (long)((up_s / 3600) % 100), (long)((up_s / 60) % 60),
             (unsigned long)(up_s % 60), ssid);
    draw_row_autoscroll(3, &o3, row);

    snprintf(row, sizeof(row), "SD %s DROP %lu",
             s->sd_file, (unsigned long)s->sd_dropped);
    draw_row_autoscroll(4, &o4, row);

    snprintf(row, sizeof(row), "BAD %lu FPGA %s UI %s",
             (unsigned long)s->frames_bad,
             s->fpga_ok ? "OK" : "--", s->ui_alive ? "OK" : "--");
    draw_row_autoscroll(5, &o5, row);

    snprintf(row, sizeof(row), "LINK %s SEQ %u",
             s->data_fresh ? "OK" : "--", (unsigned)s->last_seq);
    draw_row_autoscroll(6, &o6, row);

    snprintf(row, sizeof(row), "PKT %lu SD %s",
             (unsigned long)s->packets, s->sd_ok ? "OK" : "--");
    draw_row_autoscroll(7, &o7, row);

    (void)oled_flush();
}

/* P3 设置页 */
void oled_show_settings(uint8_t cursor)
{
    char row[40];
    char name[16], val[12];
    uint8_t start, r, item;

    if (cursor >= SETTINGS_ITEM_N) {
        cursor = 0;
    }
    start = (cursor >= 5u) ? (uint8_t)(cursor - 4u) : 0u;

    oled_clear();

    draw_title(0, "设置", 4);

    for (r = 0; r < 5; r++) {
        item = (uint8_t)(start + r);
        if (item >= SETTINGS_ITEM_N) {
            break;
        }
        settings_get_item_text(item, name, sizeof(name), val, sizeof(val));
        snprintf(row, sizeof(row), "%u %-9s %s",
                 (unsigned)(item + 1u), name, val);
        if (item == cursor) {
            oled_fill_page(2u + r, 0xFF);
            oled_draw_text_inv(2u + r, 0, row);
        } else {
            oled_draw_row(2u + r, row);
        }
    }

    /* 键位提示：KEY0/1 移光标、KEY2/3 调值、KEY14 退出 */
    oled_draw_row(7, "K0/K1 sel  K2/K3 +/-  K14 exit");
    (void)oled_flush();
}

/* P4 演示页：英文紧凑行（每行占 1 页，ASCII 6x8 不跨页，绝不重叠）。
 *
 * 版面：page0~1 反色标题「演示模式」+ 页码；page2 场景编号与名称；
 *       page3 该场景动作；page4 声光表现；page5 剧本进度；page7 按键提示。
 *
 * 为什么这里用英文：16x16 中文字模高 16 行 = 2 页，相邻两行必须隔一页
 * 才能不重叠；演示页信息行多（5 行），中文只能塞 3 行且极易叠字。
 * 因此正文用英文短句（表意清楚、非缩写），标题仍用中文。
 *
 * demo_active=false（拨码 SW1 未开）时显示开启指引。 */
void oled_show_demo(uint8_t scene, int script_step, const char *script_name,
                    bool demo_mute, bool demo_active)
{
    static const char *const k_scene[8] = {
        "Script", "Idle", "Walk", "Pending",
        "Fall Alarm", "LED Walk", "Self Test", "Reserved"
    };
    static const char *const k_act[8] = {
        "Auto demo: full sequence",
        "Simulate standing still",
        "Simulate walking",
        "Simulate impact, wait",
        "Simulate fall alarm",
        "Light LEDs one by one",
        "Check display & buzzer",
        "Not used"
    };
    static const char *const k_fx[8] = {
        "SEG scroll, LED flow",
        "SEG ready, LED breathe",
        "SEG run, LED ring flow",
        "SEG value flash, LED scan",
        "SEG alarm, all strobe",
        "SEG count, single LED",
        "Glyphs sweep, 3 tones",
        "Not used"
    };
    char row[40];

    if (scene >= 8u) {
        scene = 0u;
    }

    oled_clear();

    draw_title(0, "演示模式", 5);

    if (!demo_active) {
        oled_draw_text(2, 0, "拨码一开启演示");
        oled_draw_row(4, "SW1=ON  -> demo");
        oled_draw_row(5, "KEY0-7  select scene");
        oled_draw_row(6, "KEY8 next, KEY9/10");
        oled_draw_row(7, "SW1=OFF -> exit");
        (void)oled_flush();
        return;
    }

    /* 场景编号 + 名称（ASCII 单页） */
    snprintf(row, sizeof(row), "Scene %u: %s",
             (unsigned)scene, k_scene[scene]);
    oled_draw_row(2, row);

    /* 该场景在做什么 / 声光表现：长句自动滚动，避免右侧超出显示区 */
    static int16_t oa = 0, of = 0, ok = 0;
    draw_row_autoscroll(3, &oa, k_act[scene]);
    draw_row_autoscroll(4, &of, k_fx[scene]);

    /* 剧本进度（场景 0） */
    if (scene == 0u) {
        if (script_step >= 0) {
            snprintf(row, sizeof(row), "Step %d of 5", script_step + 1);
        } else {
            snprintf(row, sizeof(row), "Step - (not started)");
        }
        oled_draw_row(5, row);
    }

    /* 按键提示 */
    snprintf(row, sizeof(row), "%sKEY0-7 sel, KEY8 next",
             demo_mute ? "MUTE | " : "");
    draw_row_autoscroll(7, &ok, row);

    (void)oled_flush();
}
