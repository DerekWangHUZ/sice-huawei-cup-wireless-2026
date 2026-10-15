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
 *                           +----------------------+          +-------------+
 *                                    | UDP:8889
 *                                    v 抓拍指令 55 AA 01 FF
 *                              穿戴端 ESP32-CAM
 *
 * 本期范围：
 *   [做]   SoftAP、UDP 收包、18B 帧解析、跌倒判定状态机、向 FPGA 下发报警命令
 *   [占位] CSI 提取（csi_stub，接口已留好）
 *   [不做] MicroSD 记录、OLED 显示、真实 CSI 提取、穿戴端联调
 *          （模块未到货或需穿戴端在场）
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
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "fall_core.h"
#include "csi_stub.h"
#include "fpga_link.h"

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
        }

        /* 周期统计（原始日志，供验收查） */
        if (last_stats_ms == 0) {
            last_stats_ms = now_ms;
        } else if (now_ms - last_stats_ms >= STATS_PERIOD_MS) {
            last_stats_ms = now_ms;
            ESP_LOGI(TAG, "STATS packets=%u frames_ok=%u frames_bad=%u state=%s",
                     (unsigned)g_packets,
                     (unsigned)g_frames_ok,
                     (unsigned)g_frames_bad,
                     fall_state_name(det.state));
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

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

    /* UART -> FPGA */
    if (fpga_link_init() != ESP_OK) {
        ESP_LOGE(TAG, "fpga link init failed, alarm output disabled");
    }

    /* Wi-Fi SoftAP */
    wifi_init_softap();

    /* UDP 收包 + 判定任务 */
    xTaskCreate(udp_pose_task, "udp_pose", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "main init done");
}
