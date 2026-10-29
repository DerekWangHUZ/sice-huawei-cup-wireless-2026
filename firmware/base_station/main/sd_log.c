/*
 * sd_log.c -- microSD CSV 记录实现（SPI + FATFS + 环形缓冲 + 独立任务）
 *
 * 详见 sd_log.h。
 */

#include "sd_log.h"
#include "board_pins.h"
#include "log_fmt.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "SDLOG";

/* SDSPI 使用 SPI2（S3 上 GPIO10~13 即该外设 IO_MUX 默认脚） */
#define SD_SPI_HOST   SPI2_HOST

/* 记录任务栈与优先级：低于 udp_pose(5)，让步于收包与判定 */
#define SD_TASK_STACK   4096
#define SD_TASK_PRIO    3

/* 出队等待上限：即使数据停摆也要周期性醒来做 flush */
#define SD_RECV_TIMEOUT_MS 200

static RingbufHandle_t  s_rb = NULL;
static TaskHandle_t     s_task = NULL;
static sdmmc_card_t    *s_card = NULL;
static FILE            *s_fp = NULL;

static bool     s_ready = false;
static uint32_t s_dropped = 0;
static char     s_fname[24] = "-";

/* ==========================================================================
 * 文件名编号：扫描挂载点，取 DATA_nnn.CSV 的最大编号 +1（跨断电递增）
 * ========================================================================== */

static char lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

static bool ci_eq(const char *a, const char *b)
{
    while (*a != '\0' && *b != '\0') {
        if (lower(*a) != lower(*b)) {
            return false;
        }
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

/* 解析 "DATA_007.CSV" -> 7；不匹配返回 false */
static bool parse_index(const char *name, uint32_t *out)
{
    const char *p = name;
    uint32_t    v = 0;
    int         digits = 0;

    if (name == NULL || strlen(name) < 5 || lower(p[0]) != 'd' ||
        lower(p[1]) != 'a' || lower(p[2]) != 't' || lower(p[3]) != 'a' ||
        p[4] != '_') {
        return false;
    }
    p += 5;

    while (*p >= '0' && *p <= '9' && digits < 6) {
        v = v * 10u + (uint32_t)(*p - '0');
        p++;
        digits++;
    }
    if (digits == 0 || !ci_eq(p, ".CSV")) {
        return false;
    }
    *out = v;
    return true;
}

static uint32_t find_next_index(void)
{
    DIR           *d;
    struct dirent *e;
    uint32_t       max_idx = 0;

    d = opendir(SD_MOUNT_POINT);
    if (d == NULL) {
        ESP_LOGW(TAG, "opendir(%s) failed, start from 001", SD_MOUNT_POINT);
        return 1u;
    }
    while ((e = readdir(d)) != NULL) {
        uint32_t idx;
        if (parse_index(e->d_name, &idx) && idx > max_idx && idx < 1000000u) {
            max_idx = idx;
        }
    }
    closedir(d);

    ESP_LOGI(TAG, "existing max index = %u", (unsigned)max_idx);
    return max_idx + 1u;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

static esp_err_t sd_mount(void)
{
    esp_err_t err;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = SD_MOSI_GPIO,
        .miso_io_num     = SD_MISO_GPIO,
        .sclk_io_num     = SD_SCK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };

    err = spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot         = SD_SPI_HOST;
    host.max_freq_khz = SD_MAX_FREQ_KHZ;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = (gpio_num_t)SD_CS_GPIO;
    slot_cfg.host_id = SD_SPI_HOST;
#if SD_HAS_CARD_DETECT
    slot_cfg.gpio_cd = (gpio_num_t)SD_CARD_DETECT_GPIO;
#else
    slot_cfg.gpio_cd = SDSPI_SLOT_NO_CD;
#endif
    slot_cfg.gpio_wp = SDSPI_SLOT_NO_WP;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,   /* 绝不格式化用户卡 */
        .max_files              = 3,
        .allocation_unit_size   = 0,       /* 0 = FATFS 自动选择 */
        .disk_status_check_enable = false,
    };

    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_cfg,
                                  &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount %s failed: %s (card inserted? power? CS=%d)",
                 SD_MOUNT_POINT, esp_err_to_name(err), SD_CS_GPIO);
        spi_bus_free(SD_SPI_HOST);
        s_card = NULL;
        return err;
    }

    return ESP_OK;
}

static esp_err_t sd_open_file(void)
{
    char path[48];
    uint32_t idx = find_next_index();
    int      n;

    n = snprintf(s_fname, sizeof(s_fname), "DATA_%03u.CSV", (unsigned)idx);
    if (n <= 0 || (size_t)n >= sizeof(s_fname)) {
        return ESP_ERR_INVALID_SIZE;
    }
    n = snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, s_fname);
    if (n <= 0 || (size_t)n >= sizeof(path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    s_fp = fopen(path, "w");
    if (s_fp == NULL) {
        ESP_LOGE(TAG, "fopen(%s) failed", path);
        s_fname[0] = '-';
        s_fname[1] = '\0';
        return ESP_FAIL;
    }

    /* 行缓冲：由 FATFS 侧聚合成大块写，避免每行一次扇区操作 */
    static char file_buf[SD_LOG_FILE_BUF_SIZE];
    setvbuf(s_fp, file_buf, _IOFBF, sizeof(file_buf));

    if (fputs(log_fmt_header(), s_fp) < 0) {
        ESP_LOGE(TAG, "write header failed");
        fclose(s_fp);
        s_fp = NULL;
        s_fname[0] = '-';
        s_fname[1] = '\0';
        return ESP_FAIL;
    }
    fflush(s_fp);

    ESP_LOGI(TAG, "logging to %s", path);
    return ESP_OK;
}

/* ==========================================================================
 * 记录任务
 * ========================================================================== */

static void sd_log_task(void *arg)
{
    char     line[LOG_FMT_LINE_MAX];
    int64_t  last_flush_us = esp_timer_get_time();
    uint8_t  last_state    = 0xFFu;
    bool     first         = true;

    (void)arg;

    while (1) {
        size_t      item_size = 0;
        sd_log_item_t *it = (sd_log_item_t *)xRingbufferReceive(
            s_rb, &item_size, pdMS_TO_TICKS(SD_RECV_TIMEOUT_MS));

        if (it != NULL) {
            if (item_size == sizeof(sd_log_item_t)) {
                int len = log_fmt_row(line, sizeof(line),
                                      it->ts_ms, it->seq,
                                      it->acc, it->gyro,
                                      it->csi_var, it->state);
                if (len > 0 && (size_t)len < sizeof(line)) {
                    if (fwrite(line, 1, (size_t)len, s_fp) != (size_t)len) {
                        ESP_LOGE(TAG, "fwrite failed (card removed/full?), "
                                      "logging stopped");
                        goto fail;
                    }
                    if (first || it->state != last_state) {
                        last_state = it->state;
                        first = false;
                        /* 状态变化立即落盘：保证告警前后的证据不丢 */
                        fflush(s_fp);
                        fsync(fileno(s_fp));
                        last_flush_us = esp_timer_get_time();
                    }
                }
            }
            vRingbufferReturnItem(s_rb, it);
        }

        /* 周期落盘：限制掉电时的数据损失（默认 1s） */
        if ((esp_timer_get_time() - last_flush_us) >=
            ((int64_t)SD_LOG_FLUSH_PERIOD_MS * 1000)) {
            fflush(s_fp);
            fsync(fileno(s_fp));
            last_flush_us = esp_timer_get_time();
        }
    }

fail:
    /* 只关文件、停止记录，绝不影响收包/判定；剩余记录由生产者计入 dropped */
    if (s_fp != NULL) {
        fclose(s_fp);
        s_fp = NULL;
    }
    s_ready = false;
    s_task  = NULL;
    vTaskDelete(NULL);
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

esp_err_t sd_log_init(void)
{
    esp_err_t err;

    /*
     * 可重入保护：本函数会被 sd_log_probe() 在"卡恢复"时再次调用。
     * 先把上一次残留的资源收干净，避免重复建 ringbuffer / 多任务。
     */
    if (s_task != NULL) {
        /* 旧任务已自行退出（fail 路径会置 s_task=NULL），此处防御性处理 */
        s_task = NULL;
    }
    if (s_fp != NULL) {
        fclose(s_fp);
        s_fp = NULL;
    }
    if (s_rb != NULL) {
        vRingbufferDelete(s_rb);
        s_rb = NULL;
    }
    if (s_card != NULL) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
        /* 卸载后必须释放 SPI 总线，否则下一次 sd_mount() 的
         * spi_bus_initialize 会返回 ESP_ERR_INVALID_STATE 而永久失败 */
        spi_bus_free(SD_SPI_HOST);
    }
    s_ready = false;

    s_rb = xRingbufferCreateNoSplit(sizeof(sd_log_item_t), SD_LOG_QUEUE_ITEMS);
    if (s_rb == NULL) {
        ESP_LOGE(TAG, "ringbuffer create failed");
        return ESP_ERR_NO_MEM;
    }

    err = sd_mount();
    if (err != ESP_OK) {
        vRingbufferDelete(s_rb);
        s_rb = NULL;
        return err;
    }

    err = sd_open_file();
    if (err != ESP_OK) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
        vRingbufferDelete(s_rb);
        s_rb = NULL;
        return err;
    }

    s_ready = true;

    if (xTaskCreate(sd_log_task, "sd_log", SD_TASK_STACK, NULL,
                    SD_TASK_PRIO, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        s_ready = false;
        fclose(s_fp);
        s_fp = NULL;
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        s_card = NULL;
        vRingbufferDelete(s_rb);
        s_rb = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ready: file=%s queue=%d x %uB",
             s_fname, SD_LOG_QUEUE_ITEMS, (unsigned)sizeof(sd_log_item_t));
    return ESP_OK;
}

bool sd_log_ready(void)
{
    return s_ready && s_fp != NULL;
}

/*
 * 卡在位探活 + 自动恢复。
 *
 * 判据：**直接读卡的引导扇区（LBA 0）**。
 *   这是真实的硬件访问，不经过 VFS 缓冲/FATFS 缓存：
 *   卡在位 -> CMD17 正常返回；卡拔出 -> 超时/CRC 错，立即失败。
 *   之前用 fflush() 推断是错的——flush 只碰内存缓冲，卡拔了也可能"成功"，
 *   卡在着也可能因缓冲满而失败，两个方向都会误判。
 *
 * 稳定性约束：
 *   1) 连续 SD_PROBE_FAIL_MAX 次失败才降级（防单次 SPI 抖动误判）；
 *   2) 降级后周期性尝试重新挂载，卡插回即自动恢复；
 *   3) 降级后的重挂载带指数退避：全量 sd_log_init()（SPI 总线初始化 +
 *      挂载尝试）失败一次要耗几十至上百毫秒，无卡时若每个探活周期
 *      （约 5s）都做一遍，会周期性阻塞调用方（UI 任务）。前 3 次仍
 *      逐周期快速重试（应对"刚降级就插回"），之后按 4/8/16 个周期
 *      退避封顶（最长约 80s 一次），任何一次成功立即恢复逐周期探活。
 *
 * 由 UI 任务低频（约 5s）调用。返回 true = 记录功能当前可用。
 */
#define SD_PROBE_FAIL_MAX   3

static bool sd_card_alive(void)
{
    uint8_t buf[512];

    if (s_card == NULL) {
        return false;
    }
    /* 读引导扇区：卡不在位时返回非 ESP_OK */
    return (sdmmc_read_sectors(s_card, buf, 0, 1) == ESP_OK);
}

bool sd_log_probe(void)
{
    static int fail_cnt   = 0;   /* 连续扇区读失败计数（降级判据） */
    static int init_fails = 0;   /* 降级后连续重挂载失败次数 */
    static int defer_cnt  = 0;   /* 退避期：还需跳过的探活周期数 */

    /* --- 已降级：指数退避的重新挂载尝试（卡插回后自动恢复） --- */
    if (!s_ready || s_fp == NULL) {
        int shift;

        if (defer_cnt > 0) {
            defer_cnt--;
            return false;
        }
        if (sd_log_init() == ESP_OK) {
            fail_cnt   = 0;
            init_fails = 0;
            defer_cnt  = 0;
            ESP_LOGI(TAG, "SD recovered: logging re-enabled");
            return true;
        }
        init_fails++;
        if (init_fails <= 3) {
            defer_cnt = 0;                       /* 前 3 次逐周期重试 */
        } else {
            shift = init_fails - 4;
            if (shift > 2) {
                shift = 2;                       /* 4/8/16 周期封顶 */
            }
            defer_cnt = 4 << shift;
        }
        return false;
    }

    /* --- 在线：真实读扇区判断卡是否还在 --- */
    if (!sd_card_alive()) {
        if (++fail_cnt < SD_PROBE_FAIL_MAX) {
            ESP_LOGW(TAG, "SD probe fail %d/%d (retrying)",
                     fail_cnt, SD_PROBE_FAIL_MAX);
            return true;                     /* 未达阈值，仍视为可用 */
        }
        ESP_LOGW(TAG, "SD removed (probe failed %d times)", fail_cnt);
        fclose(s_fp);
        s_fp    = NULL;
        s_ready = false;
        init_fails = 0;                      /* 重挂载从快速重试开始 */
        defer_cnt  = 0;
        return false;
    }

    fail_cnt = 0;                            /* 成功即清零 */
    return true;
}

void sd_log_push(const sd_log_item_t *item)
{
    if (!s_ready || s_rb == NULL || item == NULL) {
        return;
    }
    /*
     * 超时 0 = 绝不阻塞收包路径。队列满说明卡写不过来，
     * 丢本条并计数（OLED 以 DROP 显示，与 UDP 丢包区分）。
     */
    if (xRingbufferSend(s_rb, item, sizeof(*item), 0) != pdTRUE) {
        s_dropped++;
    }
}

uint32_t sd_log_dropped(void)
{
    return s_dropped;
}

const char *sd_log_filename(void)
{
    return s_fname;
}
