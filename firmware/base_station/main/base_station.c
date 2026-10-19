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
#include "dhcpserver/dhcpserver.h"   /* dhcps_lease_t（DHCP 地址池） */

#include "board_pins.h"
#include "fall_core.h"
#include "csi_stub.h"
#include "fpga_link.h"
#include "settings.h"
#include "ui_app.h"
#if ENABLE_OLED
#include "oled_display.h"
#endif
#if ENABLE_SD_LOG
#include "sd_log.h"
#endif

static const char *TAG = "BASE";

/* --- 网络参数（契约 §1.1 / README）；系统页显示经 k_ap_* 同源导出 --- */
#define AP_SSID          "CSI_FALL_AP"
#define AP_PASSWORD      "12345678"
#define AP_CHANNEL       6
#define AP_MAX_CONN      4

const char *const k_ap_ssid    = AP_SSID;
const int        k_ap_channel = AP_CHANNEL;

#define UDP_PORT_POSE    8888      /* 收穿戴端姿态推流 */
#define UDP_PORT_SNAP    8889      /* 发抓拍指令给穿戴端 */
#define WEARABLE_IP      "192.168.4.2"

/* 抓拍指令 4 字节（契约 §1.3） */
static const uint8_t SNAP_CMD[4] = { 0x55, 0xAA, 0x01, 0xFF };

/* 统计打印周期 */
#define STATS_PERIOD_MS  5000u

/* 数码管遥测刷新周期：由设置模块管理（settings.c 的 g_fpga_tel_period_ms，
 * 默认 1s，设置页 P3 可调 0.5/1/2s）。周期重发 SET_STATE 对 FPGA 完全
 * 幂等：cmd_decoder 只做锁存，不产生任何动作（契约 §4.5）。 */

/* --- 全局统计（原始日志用） --- */
static uint32_t g_frames_ok   = 0;
static uint32_t g_frames_bad  = 0;
static uint32_t g_packets     = 0;

/* FPGA 链路是否就绪（app_main 初始化后不再变化） */
static bool g_fpga_ok = false;

/* 最近一次遥测（UI 快照显示用；写入者=udp_pose_task，读取者=快照发布） */
#if ENABLE_OLED
static uint16_t g_last_mag_x100 = 0;
static int16_t  g_last_acc[3]  = { 0, 0, 0 };
static int16_t  g_last_gyro[3] = { 0, 0, 0 };
static uint32_t g_last_frame_ms = 0;    /* 最近一帧到达时刻（UI 新鲜度判据） */
static uint32_t g_alarm_start_ms = 0;   /* 进入真实告警的时刻（0=不在告警） */
#endif

/* 告警冲击幅值（进入 CONFIRMED 时捕获，告警屏定格显示；解除即清） */
static uint16_t g_impact_mag = 0;
static bool     g_has_impact = false;

/* 人工解除告警请求（UI 任务置位；udp_pose_task 执行并清除） */
volatile bool g_manual_clear_req = false;

/* ==========================================================================
 * 人工解除告警：在主判定任务里执行（保持单一写入者纪律）。
 * 返回 true 表示本次执行了解除（调用方随后照常走状态变化沿下发 CLEAR）。
 * ========================================================================== */
static bool manual_clear_poll(fall_detector_t *det)
{
    if (!g_manual_clear_req) {
        return false;
    }
    g_manual_clear_req = false;
    if (fall_detector_force_clear(det)) {
        ESP_LOGW(TAG, "MANUAL CLEAR: alarm released by operator");
        return true;
    }
    return false;
}

/* ==========================================================================
 * 状态快照（供 UI 任务读取）
 *
 * 单一写入者：udp_pose_task（唯一接触 fall_detector 的任务）。
 * 读取者：ui 任务（经 base_station_get_status 访问器）。用互斥量保护
 * 结构体整体拷贝，避免读到半更新状态。
 * 注意：快照只用于显示，判定逻辑始终只依赖 udp_pose_task 内的 det，
 *       因此即使 UI 任务异常也不会影响判定结果。
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
    g_status.mag_x100   = g_last_mag_x100;
    g_status.acc[0]     = g_last_acc[0];
    g_status.acc[1]     = g_last_acc[1];
    g_status.acc[2]     = g_last_acc[2];
    g_status.gyro[0]    = g_last_gyro[0];
    g_status.gyro[1]    = g_last_gyro[1];
    g_status.gyro[2]    = g_last_gyro[2];
    g_status.has_impact = g_has_impact;
    g_status.impact_mag_x100 = g_impact_mag;
    g_status.last_frame_ms = g_last_frame_ms;
    g_status.alarm_start_ms = g_alarm_start_ms;
    xSemaphoreGive(g_status_mtx);
}

void base_station_get_status(oled_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (g_status_mtx == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(g_status_mtx, portMAX_DELAY);
    *out = g_status;
    xSemaphoreGive(g_status_mtx);
}
#endif /* ENABLE_OLED */

bool base_station_fpga_ok(void)
{
    return g_fpga_ok;
}

/* 真实判定状态（演示剧本退出时向 FPGA 重同步用） */
fall_state_t base_station_real_state(void)
{
#if ENABLE_OLED
    fall_state_t st;
    if (g_status_mtx == NULL) {
        return FALL_STATE_IDLE;
    }
    xSemaphoreTake(g_status_mtx, portMAX_DELAY);
    st = g_status.state;
    xSemaphoreGive(g_status_mtx);
    return st;
#else
    return FALL_STATE_IDLE;
#endif
}

bool base_station_alarm_active(void)
{
#if ENABLE_OLED
    bool ret;
    if (g_status_mtx == NULL) {
        return false;
    }
    xSemaphoreTake(g_status_mtx, portMAX_DELAY);
    ret = g_status.alarm;
    xSemaphoreGive(g_status_mtx);
    return ret;
#else
    return false;
#endif
}

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

    /*
     * DHCP 地址池避开 192.168.4.2（契约 §1.1 归穿戴端静态地址）。
     * 否则后接入的电脑可能先拿到 .2，导致抓拍指令发错设备。
     * 失败只告警：地址池维持默认，不影响收包主链路。
     */
    {
        dhcps_lease_t lease;
        memset(&lease, 0, sizeof(lease));
        lease.enable      = true;
        lease.start_ip.addr = htonl(0xC0A8040A);   /* 192.168.4.10 */
        lease.end_ip.addr   = htonl(0xC0A80464);   /* 192.168.4.100 */
        err = esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET,
                                     ESP_NETIF_REQUESTED_IP_ADDRESS,
                                     &lease, sizeof(lease));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "dhcps lease option failed: %s (keep default pool)",
                     esp_err_to_name(err));
        }
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
                                uint16_t mag_x100,
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
        /* 确认跌倒：告警 + 抓拍。ALARM/CLEAR 永不因演示模式被抑制
         * （真实告警必须直通 FPGA，安全优先）。
         * 契约 §4.5 第 2 条：确认跌倒后只发 ALARM，绝不发 SET_STATE(3)。 */
        g_impact_mag = mag_x100;          /* 冲击幅值定格（告警屏显示） */
        g_has_impact = true;
#if ENABLE_OLED
        g_alarm_start_ms = now_ms;        /* 告警计时起点（OLED T+NNs） */
#endif
        fpga_link_send_alarm();
        send_snapshot_cmd(sock);
    } else if (*last_state == FALL_STATE_CONFIRMED) {
        /* 离开告警态：解除 */
        g_has_impact = false;
#if ENABLE_OLED
        g_alarm_start_ms = 0;
#endif
        fpga_link_send_clear();
    } else {
        /* 演示剧本运行期间抑制真实状态同步帧（遥测同步停发由
         * fpga_telemetry_tick 同规则处理）；ALARM/CLEAR 不受影响。 */
        if (g_ui_demo_owns_state) {
            *last_state = st;
            return;
        }
        /* 其它迁移：同步状态显示（携带当前幅值，避免读数滞后一次刷新）。
         * 注意用 verbose 版本：状态变化沿是验收信号，日志里必须有
         * `TX cmd=0x10 p1=...`（见 docs/成员A任务状态.md 的"状态同步"行）；
         * 静默只用于 1Hz 周期刷新，避免刷屏。 */
        fpga_link_send_state_change(st, mag_x100);
    }
    *last_state = st;
}

/* ==========================================================================
 * 周期遥测刷新（契约 §4.5）：周期重发 SET_STATE 携带合加速度幅值，
 * 让数码管读数持续反映实时运动。周期由设置页 P3 调节（0.5/1/2s）。
 *
 * 约束：
 *   1) 演示剧本运行期间停发 —— 显示归剧本所有（ALARM/CLEAR 不受影响）；
 *   2) CONFIRMED 期间不发 —— 保持"确认跌倒后只发 ALARM"的既有语义；
 *   3) 无有效帧（data timeout 分支）时发 0xFFFF 哨兵；
 *   4) FPGA 链路未就绪时直接跳过。
 * ========================================================================== */
static void fpga_telemetry_tick(const fall_detector_t *det,
                                uint16_t mag_x100,
                                bool data_fresh,
                                uint32_t now_ms,
                                uint32_t *last_tel_ms)
{
    if (!g_fpga_ok || det->state == FALL_STATE_CONFIRMED) {
        return;
    }
    if (g_ui_demo_owns_state) {
        /* 演示剧本拥有显示：真实遥测停发，防止覆盖剧本画面 */
        *last_tel_ms = now_ms;
        return;
    }
    if (*last_tel_ms == 0) {
        *last_tel_ms = now_ms;
        return;
    }
    if (now_ms - *last_tel_ms < g_fpga_tel_period_ms) {
        return;
    }
    *last_tel_ms = now_ms;

    fpga_link_send_state_telemetry(det->state,
                                   data_fresh ? mag_x100 : FPGA_TEL_NO_DATA);
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
     *
     * 2026-10-18 实测注记（未改代码，仅记录）：本 IDF 构建下该超时实测**不生效**
     * ——25s 无数据时该分支一次都没跑过（真机日志只在"下一帧到达"时才判定掉线，
     * 与设计的 3s 超时不符）。曾试用 select() 显式超时替代，实机表现相同，
     * 因无法在无数据场景下验证而**回退**，以免改动已实测通过的主链路。
     * 影响范围：仅"完全没有数据到达"时的空闲路径（掉线保护 tick、遥测哨兵）。
     * 数码管读数不会因此显示过期值——FPGA 侧已自行做 3s 遥测老化（见
     * rtl/seg_display.v 的 TELEM_STALE_DIV），不依赖 S3 的这条路径。
     */
    struct timeval rx_timeout = { .tv_sec = 0, .tv_usec = 100 * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rx_timeout, sizeof(rx_timeout));

    fall_detector_t det;
    fall_detector_init(&det);
    posture_parser_reset();

    fall_state_t last_state   = det.state;
    uint32_t     last_stats_ms = 0;
    uint16_t     last_mag_x100 = 0;      /* 最近一次合加速度幅值（0.01g） */
    uint32_t     last_tel_ms   = 0;      /* 上次遥测刷新时刻 */
#if ENABLE_OLED
    /* 仅 OLED 状态快照需要记住最近序号；模块关闭时不声明，避免未使用告警 */
    uint16_t     last_seq      = 0;
#endif

    /* 上电首帧：同步初始状态到 FPGA（契约 §4.5 第 3 条）。
     * 此时还没有任何传感器数据，遥测用哨兵 -> 数码管读数显示 "----"。
     * 这一帧走 verbose 版本，让日志里能看到上电首帧同步（验收信号）；
     * 后续 1Hz 遥测刷新走静默版本，不刷屏。 */
    fpga_link_send_cmd(0x10u, (uint8_t)last_state, FPGA_TEL_NO_DATA);

    uint8_t rxbuf[128];

    while (1) {
        int len = recvfrom(sock, rxbuf, sizeof(rxbuf), 0, NULL, NULL);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        if (len <= 0) {
            /*
             * 超时/无数据：也要推进告警超时与掉线保护（每 100ms 一次）。
             * 状态机解除 -> 照常走状态变化沿下发 CLEAR/SET_STATE。
             */
            /* 人工解除请求优先于 tick（操作者显式命令） */
            if (manual_clear_poll(&det)) {
                handle_state_change(sock, &det, &last_state, det.state, 0,
                                    FPGA_TEL_NO_DATA, now_ms);
            }
            fall_state_t st = fall_detector_tick(&det, now_ms);
            handle_state_change(sock, &det, &last_state, st, 0,
                                FPGA_TEL_NO_DATA, now_ms);
            fpga_telemetry_tick(&det, last_mag_x100, false, now_ms, &last_tel_ms);
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
#if ENABLE_OLED
            g_last_frame_ms = now_ms;    /* 数据流存活判据（UI P1/P2 LINK） */
#endif

            /* 合加速度幅值（0.01g）——遥测用，纯整数换算，无浮点 */
            last_mag_x100 = fall_acc_mag_centi_g(frame.acc);
#if ENABLE_OLED
            g_last_mag_x100 = last_mag_x100;
            g_last_acc[0] = frame.acc[0];
            g_last_acc[1] = frame.acc[1];
            g_last_acc[2] = frame.acc[2];
            g_last_gyro[0] = frame.gyro[0];
            g_last_gyro[1] = frame.gyro[1];
            g_last_gyro[2] = frame.gyro[2];
#endif

            /* CSI 占位：真实实现在此更新统计量 */
            csi_feed_packet();

            /* 跌倒判定（先处理人工解除请求，保证按键即刻生效） */
            (void)manual_clear_poll(&det);
            fall_state_t st = fall_detector_feed(&det,
                                                 frame.acc,
                                                 frame.gyro,
                                                 now_ms);

            /* 人工解除后状态已变，走状态变化沿下发 CLEAR */
            if (st != last_state && st == FALL_STATE_IDLE &&
                last_state == FALL_STATE_CONFIRMED) {
                ESP_LOGW(TAG, "MANUAL CLEAR applied (state edge)");
            }

            /* 状态变化沿 -> 下发控制帧（契约 §4.5） */
            handle_state_change(sock, &det, &last_state, st, frame.seq,
                                last_mag_x100, now_ms);

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

        /* 1Hz 遥测刷新：让 FPGA 数码管读数持续反映实时合加速度（契约 §4.5） */
        fpga_telemetry_tick(&det, last_mag_x100, true, now_ms, &last_tel_ms);

        /* 周期统计（原始日志，供验收查） */
        if (last_stats_ms == 0) {
            last_stats_ms = now_ms;
        } else if (now_ms - last_stats_ms >= STATS_PERIOD_MS) {
            last_stats_ms = now_ms;
#if ENABLE_SD_LOG
            ESP_LOGI(TAG, "STATS packets=%u frames_ok=%u frames_bad=%u "
                          "state=%s mag=%u.%02ug sd_drop=%u",
                     (unsigned)g_packets,
                     (unsigned)g_frames_ok,
                     (unsigned)g_frames_bad,
                     fall_state_name(det.state),
                     (unsigned)(last_mag_x100 / 100u),
                     (unsigned)(last_mag_x100 % 100u),
                     (unsigned)sd_log_dropped());
#else
            ESP_LOGI(TAG, "STATS packets=%u frames_ok=%u frames_bad=%u "
                          "state=%s mag=%u.%02ug",
                     (unsigned)g_packets,
                     (unsigned)g_frames_ok,
                     (unsigned)g_frames_bad,
                     fall_state_name(det.state),
                     (unsigned)(last_mag_x100 / 100u),
                     (unsigned)(last_mag_x100 % 100u));
#endif
        }
    }
}

/* OLED 单页状态渲染任务已由 ui_app.c 的 ui 任务取代（多页面 + 键盘导航 +
 * 演示剧本）；快照填充逻辑同样移至 ui 任务。 */

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

    /* 可视化设置：NVS 恢复后把蜂鸣器配置（0x03）下发 FPGA。
     * OLED 对比度/遥测周期在 UI 任务中等面板就绪后应用。 */
    (void)settings_init();
    if (g_fpga_ok) {
        settings_apply_audio();
    }

    /* Wi-Fi SoftAP */
    wifi_init_softap();

    /* UDP 收包 + 判定任务 */
    xTaskCreate(udp_pose_task, "udp_pose", 4096, NULL, 5, NULL);

    /* 多页面 UI + 键盘导航 + 演示剧本（OLED 就绪时启动；内部含
     * fpga_uplink 接收任务，优先级 2，低于收包任务） */
#if ENABLE_OLED
    ui_app_start();
#endif

    ESP_LOGI(TAG, "main init done");
}
