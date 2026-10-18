/*
 * base_station.c -- ESP32-S3 固定基站主控
 *
 * ==========================================================================
 * 系统拓扑（本期主链路）
 * ==========================================================================
 *
 *   穿戴端 ESP32-CAM         基站 ESP32-S3                      Artix-7 FPGA
 *   +--------------+        +----------------------+          +-------------+
 *   | MPU6050      | UDP    | SoftAP CSI_FALL_AP   |  UART1   | cmd_decoder |
 *   | 18B 姿态帧   | =====> | UDP:8888 收包        | =======> | seg_display |
 *   | 50Hz 推流    |  8888  | fall_core 解析+判定  | 115200   | buzzer_ctrl |
 *   +--------------+        | fpga_link 发控制帧   |  8N1     | 16 LED      |
 *                           +--+--------------+----+          +-------------+
 *                     I2C0 ----+              +---- SPI2
 *                 OLED 状态屏              microSD CSV 记录
 *                                    |
 *                                    | UDP:8889 抓拍指令 55 AA 01 FF
 *                                    v
 *                              穿戴端 ESP32-CAM
 *
 * 范围：
 *   [做]   SoftAP、UDP 收包、18B 帧解析、跌倒判定状态机、向 FPGA 下发报警命令
 *   [做]   microSD CSV 记录（sd_log，契约 §2.1）、OLED 状态显示（oled_display）
 *   [占位] CSI 提取（csi_stub，接口已留好；CSV 的 CSI_Variance 列恒 0）
 *   [不做] 真实 CSI 提取、穿戴端联调（需穿戴端在场）
 *
 * 协议：
 *   姿态帧 18B  见 README.md / docs/团队协作规范与接口契约.md §1.2
 *   控制帧  8B  见 docs/团队协作规范与接口契约.md §4
 * ==========================================================================
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "board_pins.h"
#include "fall_core.h"
#include "csi_stub.h"
#include "fpga_link.h"
#if ENABLE_OLED
#include "oled_display.h"
#endif
#if ENABLE_SD_LOG
#include "sd_log.h"
#endif

static const char *TAG = "BASE";

/* --- 网络参数（契约 §1.1 / README） --- */
#define AP_SSID          "CSI_FALL_AP"
#define AP_PASSWORD      "12345678"
#define AP_CHANNEL       6
#define AP_MAX_CONN      4

#define UDP_PORT_POSE    8888      /* 收穿戴端姿态推流 */
#define UDP_PORT_SNAP    8889      /* 发抓拍指令给穿戴端 */
#define WEARABLE_IP      "192.168.4.2"

/* 抓拍指令 4 字节（契约 §1.3） */
static const uint8_t SNAP_CMD[4] = { 0x55, 0xAA, 0x01, 0xFF };

/* 统计打印周期 */
#define STATS_PERIOD_MS  5000u

/* --- 全局统计（原始日志用） --- */
static uint32_t g_frames_ok   = 0;
static uint32_t g_frames_bad  = 0;
static uint32_t g_packets     = 0;

/* FPGA 链路是否就绪（app_main 初始化后不再变化） */
static bool g_fpga_ok = false;

/* ==========================================================================
 * 状态快照（供 OLED 任务读取）
 *
 * 单一写入者：udp_pose_task（唯一接触 fall_detector 的任务）。
 * 读取者：oled_task。用互斥量保护结构体整体拷贝，避免读到半更新状态。
 * 注意：快照只用于显示，判定逻辑始终只依赖 udp_pose_task 内的 det，
 *       因此即使 OLED 任务异常也不会影响判定结果。
 * ========================================================================== */
#if ENABLE_OLED
static oled_status_t    g_status;
static SemaphoreHandle_t g_status_mtx = NULL;

static void status_publish(const fall_detector_t *det, uint16_t seq)
{
    if (g_status_mtx == NULL) {
        return;
    }
    xSemaphoreTake(g_status_mtx, portMAX_DELAY);
    g_status.state      = det->state;
    g_status.alarm      = fall_detector_alarm_active(det);
    g_status.last_seq   = seq;
    g_status.packets    = g_packets;
    g_status.frames_ok  = g_frames_ok;
    g_status.frames_bad = g_frames_bad;
    g_status.csi_var    = csi_get_variance();
    xSemaphoreGive(g_status_mtx);
}
#endif /* ENABLE_OLED */

/* ==========================================================================
 * SoftAP 初始化
 * ========================================================================== */
static void wifi_init_softap(void)
{
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s",
                 esp_err_to_name(err));
        return;
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == NULL) {
        ESP_LOGE(TAG, "create_default_wifi_ap failed");
        return;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_mode failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_config_t cfg = { 0 };
    strncpy((char *)cfg.ap.ssid, AP_SSID, sizeof(cfg.ap.ssid) - 1);
    strncpy((char *)cfg.ap.password, AP_PASSWORD, sizeof(cfg.ap.password) - 1);
    cfg.ap.ssid_len       = (uint8_t)strlen(AP_SSID);
    cfg.ap.channel        = AP_CHANNEL;        /* 固定信道，禁止自动切频 */
    cfg.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;
    cfg.ap.max_connection = AP_MAX_CONN;
    cfg.ap.beacon_interval = 100;

    err = esp_wifi_set_config(WIFI_IF_AP, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_config failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_start failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "SoftAP started: ssid=%s channel=%d ip=192.168.4.1",
             AP_SSID, AP_CHANNEL);
}

/* ==========================================================================
 * 抓拍指令下发（契约 §1.3）
 * ========================================================================== */
static void send_snapshot_cmd(int sock)
{
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(UDP_PORT_SNAP);
    dest.sin_addr.s_addr = inet_addr(WEARABLE_IP);

    int sent = sendto(sock, SNAP_CMD, sizeof(SNAP_CMD), 0,
                      (struct sockaddr *)&dest, sizeof(dest));
    if (sent < 0) {
        ESP_LOGW(TAG, "snap cmd send failed (errno=%d)", errno);
    } else {
        ESP_LOGI(TAG, "snap cmd sent to %s:%d", WEARABLE_IP, UDP_PORT_SNAP);
    }
}

/* ==========================================================================
 * 状态变化沿 -> 下发控制帧（契约 §4.5）。feed 与 tick 共用。
 *   seq：feed 路径传帧序号；tick 路径无帧传 0。
 * ========================================================================== */
static void handle_state_change(int sock,
                                fall_detector_t *det,
                                fall_state_t *last_state,
                                fall_state_t st,
                                uint16_t seq,
                                uint32_t now_ms)
{
    if (st == *last_state) {
        return;
    }

    ESP_LOGI(TAG, "STATE %s -> %s | seq=%u mag2=%u gyro2=%u t=%ums",
             fall_state_name(*last_state),
             fall_state_name(st),
             (unsigned)seq,
             (unsigned)det->last_mag2,
             (unsigned)det->last_gyro2,
             (unsigned)now_ms);

    if (st == FALL_STATE_CONFIRMED) {
        /* 确认跌倒：告警 + 抓拍 */
        fpga_link_send_alarm();
        send_snapshot_cmd(sock);
    } else if (*last_state == FALL_STATE_CONFIRMED) {
        /* 离开告警态：解除 */
        fpga_link_send_clear();
    } else {
        /* 其它迁移：同步状态显示 */
        fpga_link_send_state(st);
    }
    *last_state = st;
}

/* ==========================================================================
 * UDP 收包 + 判定主任务
 * ========================================================================== */
static void udp_pose_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family      = AF_INET;
    bind_addr.sin_port        = htons(UDP_PORT_POSE);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        ESP_LOGE(TAG, "bind :%d failed", UDP_PORT_POSE);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UDP listening on :%d", UDP_PORT_POSE);

    /*
     * 收包超时 100ms：数据流停摆时任务也要周期醒来跑 fall_detector_tick，
     * 保证告警超时解除/掉线保护不依赖"下一帧到达"（实测缺陷修复）。
     */
    struct timeval rx_timeout = { .tv_sec = 0, .tv_usec = 100 * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));

    fall_detector_t det;
    fall_detector_init(&det);
    posture_parser_reset();

    fall_state_t last_state   = det.state;
    uint32_t     last_stats_ms = 0;
#if ENABLE_OLED
    /* 仅 OLED 状态快照需要记住最近序号；模块关闭时不声明，避免未使用告警 */
    uint16_t     last_seq      = 0;
#endif

    /* 上电首帧：同步初始状态到 FPGA（契约 §4.5 第 3 条） */
    fpga_link_send_state(last_state);

    uint8_t rxbuf[128];

    while (1) {
        int len = recvfrom(sock, rxbuf, sizeof(rxbuf), 0, NULL, NULL);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        if (len <= 0) {
            /*
             * 超时/无数据：也要推进告警超时与掉线保护（每 100ms 一次）。
             * 状态机解除 -> 照常走状态变化沿下发 CLEAR/SET_STATE。
             */
            fall_state_t st = fall_detector_tick(&det, now_ms);
            handle_state_change(sock, &det, &last_state, st, 0, now_ms);
#if ENABLE_OLED
            status_publish(&det, last_seq);
#endif
            continue;
        }
        g_packets++;

        /* 逐字节喂给流式解析器（天然兼容粘包/半包/错位） */
        int i;
        for (i = 0; i < len; i++) {
            posture_frame_t frame;
            posture_result_t r = posture_parser_feed(rxbuf[i], &frame);

            if (r == POSTURE_BAD) {
                g_frames_bad++;
                continue;
            }
            if (r != POSTURE_OK) {
                continue;   /* POSTURE_WAIT */
            }

            g_frames_ok++;

            /* CSI 占位：真实实现在此更新统计量 */
            csi_feed_packet();

            /* 跌倒判定 */
            fall_state_t st = fall_detector_feed(&det,
                                                 frame.acc,
                                                 frame.gyro,
                                                 now_ms);

            /* 状态变化沿 -> 下发控制帧（契约 §4.5） */
            handle_state_change(sock, &det, &last_state, st, frame.seq, now_ms);

#if ENABLE_OLED
            last_seq = frame.seq;
#endif

            /*
             * SD 记录：非阻塞入队，绝不在此写卡。
             * 时序约束见 sd_log.h（写卡耗时可打断 100ms 收包节拍）。
             */
#if ENABLE_SD_LOG
            {
                sd_log_item_t rec;
                rec.ts_ms   = now_ms;
                rec.seq     = frame.seq;
                rec.acc[0]  = frame.acc[0];
                rec.acc[1]  = frame.acc[1];
                rec.acc[2]  = frame.acc[2];
                rec.gyro[0] = frame.gyro[0];
                rec.gyro[1] = frame.gyro[1];
                rec.gyro[2] = frame.gyro[2];
                rec.csi_var = csi_get_variance();
                rec.state   = (uint8_t)st;
                sd_log_push(&rec);
            }
#endif

#if ENABLE_OLED
            status_publish(&det, last_seq);
#endif
        }

        /* 周期统计（原始日志，供验收查） */
        if (last_stats_ms == 0) {
            last_stats_ms = now_ms;
        } else if (now_ms - last_stats_ms >= STATS_PERIOD_MS) {
            last_stats_ms = now_ms;
#if ENABLE_SD_LOG
            ESP_LOGI(TAG, "STATS packets=%u frames_ok=%u frames_bad=%u "
                          "state=%s sd_drop=%u",
                     (unsigned)g_packets,
                     (unsigned)g_frames_ok,
                     (unsigned)g_frames_bad,
                     fall_state_name(det.state),
                     (unsigned)sd_log_dropped());
#else
            ESP_LOGI(TAG, "STATS packets=%u frames_ok=%u frames_bad=%u state=%s",
                     (unsigned)g_packets,
                     (unsigned)g_frames_ok,
                     (unsigned)g_frames_bad,
                     fall_state_name(det.state));
#endif
        }
    }
}

/* ==========================================================================
 * OLED 显示任务
 *
 * 独立低优先级任务：I2C 整屏刷新（8 页 x 128B @400kHz 约 25ms）不能放在
 * 收包任务里，否则同样会打乱 100ms 收包节拍。此处只读快照，不碰判定状态。
 * ========================================================================== */
#if ENABLE_OLED
static void oled_task(void *arg)
{
    (void)arg;

    while (1) {
        oled_status_t snap;

        if (g_status_mtx != NULL) {
            xSemaphoreTake(g_status_mtx, portMAX_DELAY);
            snap = g_status;
            xSemaphoreGive(g_status_mtx);
        } else {
            memset(&snap, 0, sizeof(snap));
        }

        /* 这几个量不属于判定状态，直接取（写者仅在初始化期写文件名） */
        snap.fpga_ok    = g_fpga_ok;
#if ENABLE_SD_LOG
        snap.sd_ok      = sd_log_ready();
        snap.sd_dropped = sd_log_dropped();
        snprintf(snap.sd_file, sizeof(snap.sd_file), "%s", sd_log_filename());
#else
        snap.sd_ok      = false;
        snap.sd_dropped = 0;
        snprintf(snap.sd_file, sizeof(snap.sd_file), "%s", "-");
#endif

        /* 失败已在驱动内计数限流，这里不重复刷屏 */
        (void)oled_show_status(&snap);

        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_PERIOD_MS));
    }
}
#endif /* ENABLE_OLED */

/* ==========================================================================
 * 入口
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "base_station (ESP32-S3) starting");

    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "nvs erase & retry");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
    }

    /* 状态快照互斥量（先于任何使用它的任务创建） */
#if ENABLE_OLED
    g_status_mtx = xSemaphoreCreateMutex();
    if (g_status_mtx == NULL) {
        ESP_LOGE(TAG, "status mutex create failed, OLED disabled");
    }
    memset(&g_status, 0, sizeof(g_status));
    snprintf(g_status.sd_file, sizeof(g_status.sd_file), "%s", "-");
#endif

    /*
     * 扩展模块初始化顺序：SD -> OLED -> FPGA 链路 -> Wi-Fi。
     * 两者失败都只是"该功能不可用"，绝不阻断主链路（与 fpga_link 同策略）。
     * SD 放在 Wi-Fi 之前：卡初始化/挂载较慢，避免与射频启动互相干扰。
     */
#if ENABLE_SD_LOG
    if (sd_log_init() != ESP_OK) {
        ESP_LOGE(TAG, "sd log init failed, data recording disabled");
    }
#else
    ESP_LOGW(TAG, "sd log disabled by board_pins.h");
#endif

#if ENABLE_OLED
    if (oled_init() != ESP_OK) {
        ESP_LOGE(TAG, "oled init failed, status display disabled");
    }
#else
    ESP_LOGW(TAG, "oled disabled by board_pins.h");
#endif

    /* UART -> FPGA */
    g_fpga_ok = (fpga_link_init() == ESP_OK);
    if (!g_fpga_ok) {
        ESP_LOGE(TAG, "fpga link init failed, alarm output disabled");
    }

    /* Wi-Fi SoftAP */
    wifi_init_softap();

    /* UDP 收包 + 判定任务 */
    xTaskCreate(udp_pose_task, "udp_pose", 4096, NULL, 5, NULL);

    /* OLED 状态显示任务（优先级 2，低于收包） */
#if ENABLE_OLED
    if (g_status_mtx != NULL && oled_ready()) {
        xTaskCreate(oled_task, "oled", 3072, NULL, 2, NULL);
    }
#endif

    ESP_LOGI(TAG, "main init done");
}
