/*
 * oled_display.c -- 1.3" 四脚 I2C OLED 驱动实现（SSD1306 / SH1106 通吃）
 *
 * 详见 oled_display.h 的设计说明。
 */

#include "oled_display.h"
#include "board_pins.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"

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

    ESP_LOGI(TAG, "panel ready: addr=0x%02X ctrl=%s size=%dx%d",
             addr, OLED_CTRL_SH1106 ? "SH1106" : "SSD1306",
             OLED_WIDTH, OLED_HEIGHT);
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

/* 在页内按列绘制一个字符；超出右边界的列直接丢弃（截断） */
static void draw_char(uint8_t page, uint8_t x, char ch)
{
    uint8_t       idx;
    const uint8_t *glyph;
    uint8_t       i;

    if (x + FONT_CELL > OLED_WIDTH) {
        return;                       /* 越界整字丢弃 */
    }

    if ((uint8_t)ch < 0x20u || (uint8_t)ch > 0x7Eu) {
        ch = '?';                     /* 非可打印字符统一显示 ? */
    }
    idx   = (uint8_t)((uint8_t)ch - 0x20u);
    glyph = s_font5x7[idx];

    for (i = 0; i < FONT_W; i++) {
        s_fb[page][x + i] = glyph[i];
    }
    s_fb[page][x + FONT_W] = 0x00;    /* 字符间隔列 */
}

void oled_draw_row(uint8_t page, const char *text)
{
    uint8_t x = 0;

    if (page >= OLED_PAGES || text == NULL) {
        return;
    }
    while (*text != '\0' && (uint8_t)(x + FONT_CELL) <= OLED_WIDTH) {
        draw_char(page, x, *text);
        x = (uint8_t)(x + FONT_CELL);
        text++;
    }
}

esp_err_t oled_flush(void)
{
    uint8_t   page;
    esp_err_t first_err = ESP_OK;

    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    for (page = 0; page < OLED_PAGES; page++) {
        /* 内容未变的页不占 I2C 带宽（状态屏多数页是静态的） */
        if (memcmp(s_fb[page], s_shadow[page], OLED_WIDTH) == 0) {
            continue;
        }
        esp_err_t err = oled_write_page(page, s_fb[page]);
        if (err != ESP_OK) {
            oled_note_error("write_page", err);
            if (first_err == ESP_OK) {
                first_err = err;
            }
            continue;                /* 其余页继续尝试 */
        }
        memcpy(s_shadow[page], s_fb[page], OLED_WIDTH);
    }
    return first_err;
}

/* ==========================================================================
 * 状态屏渲染
 * ========================================================================== */

esp_err_t oled_show_status(const oled_status_t *s)
{
    /*
     * row 容量按"最长可能输出"给足（最长为 "PKT 4294967295 SEQ 65535" = 25），
     * 取 48 留裕量，避免 -Wformat-truncation 报错；
     * 实际每行只画 21 个字符，超出部分由 oled_draw_row 截断。
     */
    char row[48];

    if (s == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    oled_clear();

    oled_draw_row(0, "AP CSI_FALL_AP");
    oled_draw_row(1, "IP 192.168.4.1");

    snprintf(row, sizeof(row), "STATE %s", fall_state_name(s->state));
    oled_draw_row(2, row);

    snprintf(row, sizeof(row), "PKT %u SEQ %u",
             (unsigned)s->packets, (unsigned)s->last_seq);
    oled_draw_row(3, row);

    snprintf(row, sizeof(row), "OK %u BAD %u",
             (unsigned)s->frames_ok, (unsigned)s->frames_bad);
    oled_draw_row(4, row);

    snprintf(row, sizeof(row), "SD %s", s->sd_ok ? s->sd_file : "FAIL");
    oled_draw_row(5, row);

    snprintf(row, sizeof(row), "DROP %u CSI %u",
             (unsigned)s->sd_dropped, (unsigned)s->csi_var);
    oled_draw_row(6, row);

    snprintf(row, sizeof(row), "ALARM %s FPGA %s",
             s->alarm ? "ON" : "OFF", s->fpga_ok ? "OK" : "--");
    oled_draw_row(7, row);

    return oled_flush();
}
