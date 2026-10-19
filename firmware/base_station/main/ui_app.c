/*
 * ui_app.c -- OLED 多页面 UI / 键盘导航 / 演示剧本实现
 *
 * 任务划分（延续既有实时性纪律：I2C 刷屏不能进收包任务）：
 *   fpga_uplink 任务（fpga_uplink.c）  : UART RX + 解帧，事件入队
 *   ui 任务（本文件，优先级 2）        : 事件消费 + 250ms 页面解析渲染
 *   demo 剧本任务（本文件，优先级 2）  : 场景 0 时按剧本下发控制帧
 *
 * 真实判定主链路（udp_pose_task）不感知 UI：唯一交互点是
 * g_ui_demo_owns_state —— 演示剧本运行期间抑制真实 SET_STATE/遥测下发
 * （ALARM/CLEAR 永不抑制），保证"演示不破坏告警安全语义"。
 */

#include "ui_app.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_pins.h"
#include "fall_core.h"
#include "fpga_link.h"
#include "oled_display.h"
#include "settings.h"
#include "fpga_uplink.h"
#include "ui_proto.h"
#if ENABLE_SD_LOG
#include "sd_log.h"
#endif

#if ENABLE_OLED

static const char *TAG = "UI";

/* ------------------------------------------------------------------
 * 状态
 * ------------------------------------------------------------------ */

static QueueHandle_t s_evt_q = NULL;

typedef enum {
    PAGE_STATUS = 0,   /* P0 状态页（默认） */
    PAGE_TELEM  = 1,   /* P1 遥测页 */
    PAGE_SYSTEM = 2,   /* P2 系统页 */
    PAGE_SET    = 3,   /* P3 设置页 */
    PAGE_DEMO   = 4    /* P4 演示页 */
} ui_page_t;

static volatile bool   s_settings_mode = false;
static volatile uint8_t s_cursor = 0;
static volatile uint8_t s_page_lock = 0;      /* 0=自动 1..5=P0..P4 */
static volatile bool   s_demo = false;
static volatile bool   s_demo_mute = false;
static volatile uint8_t s_demo_scene = 0;

/* 遥测条形图历史（P1 渲染用，ui 任务私有） */
#define HIST_N 16u
static uint8_t s_hist[HIST_N];

/* 演示剧本步表（场景 0；demo_task 执行、P4 渲染步名） */
typedef struct {
    const char *name;
    uint32_t    dur_ms;
} script_step_t;

static const script_step_t k_script[] = {
    { "静止",   4000u },
    { "行走",   4000u },
    { "待确认", 5000u },
    { "跌倒告警", 6000u },
    { "解除",   3000u },
};
#define SCRIPT_N (sizeof(k_script) / sizeof(k_script[0]))

/* 演示剧本 */
static volatile int s_script_step = -1;

volatile bool g_ui_demo_owns_state = false;

int ui_demo_script_step(void)
{
    return s_script_step;
}

/* ------------------------------------------------------------------
 * 帧率统计（ui 任务从相邻快照的包数差分计算）
 * ------------------------------------------------------------------ */

/* ------------------------------------------------------------------
 * 键盘处理（键编号 = 板上丝印 KEY0~KEY15）
 *
 *   正常模式：KEY13 循环翻页  KEY14 进设置  KEY15 蜂鸣器静音切换
 *   设置模式：KEY0/KEY1 移光标  KEY2/KEY3 调值  KEY14 退出
 *   演示模式：场景选择由 FPGA 本地完成（KEY0~7 选场景 / KEY8 下一个 /
 *             KEY9·10 调参），S3 仅同步显示。
 * ------------------------------------------------------------------ */

/* 页面数（P0~P4）；正常模式 KEY13 在所有页面间循环 */
#define UI_PAGE_N 5u

/* 进入设置模式的时刻：由"进入"动作显式写入，退出时清零。
 * 用途仅是屏蔽"同一次按下产生的重复 KEY14 事件"，
 * 不得吞掉后续按键——屏蔽条件严格限定为 KEY14 且仅在 300ms 窗口内。 */
static int64_t s_settings_enter_ms = 0;

static void handle_key_normal(uint8_t key)
{
    static int64_t clear_arm_ms = 0;

    /* 告警锁存期间：KEY0 = 人工解除（3s 内按两次确认，防误触） */
    if (base_station_alarm_active()) {
        if (key == UI_KEY_0) {
            int64_t now = esp_timer_get_time() / 1000LL;
            if (clear_arm_ms != 0 && now - clear_arm_ms < 3000LL) {
                clear_arm_ms = 0;
                g_manual_clear_req = true;      /* 由判定任务执行解除 */
            } else {
                clear_arm_ms = now;
                oled_set_hint("人工解除?");
            }
        }
        return;                                  /* 告警期间其它键不翻页 */
    }
    clear_arm_ms = 0;

    switch (key) {
        case UI_KEY_13:                       /* 循环翻页 P0->P1->...->P4->P0 */
            /* 注意 lock 的取值语义：0 = 自动（画面即 P0），1..5 = P0..P4。
             * 因此从 0 出发应直接跳到 2（=P1 遥测），而不是 1（仍是 P0）——
             * 否则第一次按键画面不变，表现为"要按两次"。
             * 统一处理：先把 lock 折算成"当前页索引 0..4"，再 +1 循环回写。 */
            {
                uint8_t cur = (s_page_lock == 0u) ? 0u
                                                  : (uint8_t)(s_page_lock - 1u);
                cur = (uint8_t)((cur + 1u) % UI_PAGE_N);
                s_page_lock = (uint8_t)(cur + 1u);
            }
            oled_force_full_refresh();
            break;
        case UI_KEY_14:                       /* 进入设置页 */
            s_settings_mode = true;
            s_cursor = 0;
            s_page_lock = 0;                  /* 清掉页面锁定，退出时回默认页 */
            s_settings_enter_ms = esp_timer_get_time() / 1000LL;
            oled_force_full_refresh();
            break;
        case UI_KEY_15:                       /* 蜂鸣器使能切换 */
            settings_adjust(5, +1);
            break;
        case UI_KEY_0:                        /* 回默认状态页 */
            s_page_lock = 0;
            oled_force_full_refresh();
            break;
        default: break;
    }
}

/* 进入设置模式的时刻由 handle_key_normal 写入（见上方静态变量） */
static void handle_key_settings(uint8_t key)
{
    int64_t now = esp_timer_get_time() / 1000LL;

    /* 只屏蔽进入后极短窗口内的同名键（KEY14），其余键立即生效 */
    if (s_settings_enter_ms != 0 && now - s_settings_enter_ms < 300LL &&
        key == UI_KEY_14) {
        return;
    }

    switch (key) {
        case UI_KEY_0:                        /* 光标上一项 */
            s_cursor = (uint8_t)((s_cursor + SETTINGS_ITEM_N - 1u) %
                                 SETTINGS_ITEM_N);
            break;
        case UI_KEY_1:                        /* 光标下一项 */
            s_cursor = (uint8_t)((s_cursor + 1u) % SETTINGS_ITEM_N);
            break;
        case UI_KEY_2: settings_adjust(s_cursor, -1); break;   /* 调值 - */
        case UI_KEY_3: settings_adjust(s_cursor, +1); break;   /* 调值 + */
        case UI_KEY_14:                       /* 退出设置 */
            s_settings_mode = false;
            s_page_lock = 0;
            s_settings_enter_ms = 0;
            oled_set_hint(NULL);
            oled_force_full_refresh();
            break;
        default: break;
    }
}

/* ------------------------------------------------------------------
 * 页面解析
 * ------------------------------------------------------------------ */

static ui_page_t resolve_page(const oled_status_t *snap)
{
    static uint8_t  rot_idx = 0;
    static int64_t  rot_ms  = 0;
    ui_settings_t   cfg;

    settings_get(&cfg);

    /* 设置页优先级最高：这是操作者主动进入的交互态，任何后台状态
     * （演示快照、告警快照、轮播）都不得把它顶掉，否则按键会"没反应"、
     * 或刚进设置就退回主页。退出设置只能由 KEY14 触发。 */
    if (s_settings_mode) {
        return PAGE_SET;
    }

    /* 演示模式（拨码 SW1=1）= 操作者主动选择的独占演练通道：
     * 真实状态（含真实告警）不抢屏、不打断剧本；真实告警的声光由 FPGA
     * 物理输出穿透承担，演示退出（拨回 SW1）后 OLED 立即回到真实状态。 */
    if (s_demo) {
        return PAGE_DEMO;
    }
    if (snap->alarm) {
        return PAGE_STATUS;               /* 真实告警强制状态屏 */
    }
    if (s_page_lock != 0) {
        rot_ms = 0;                       /* 手动指定页面时复位轮播相位，
                                           * 避免退出锁定后立刻跳到旧相位 */
        return (ui_page_t)(s_page_lock - 1);
    }
    if (cfg.page_auto) {
        int64_t now = esp_timer_get_time() / 1000LL;
        if (rot_ms == 0 || now - rot_ms >= (int64_t)cfg.cycle_ms) {
            rot_ms = now;
            rot_idx = (uint8_t)((rot_idx + 1u) % 3u);   /* P0->P1->P2 */
        }
        return (ui_page_t)rot_idx;
    }
    rot_ms = 0;
    return PAGE_STATUS;
}

/* ------------------------------------------------------------------
 * 渲染
 * ------------------------------------------------------------------ */

static void render(ui_page_t page, const oled_status_t *snap)
{
    switch (page) {
        case PAGE_TELEM:  oled_show_telemetry(snap, s_hist, HIST_N); break;
        case PAGE_SYSTEM: oled_show_system(snap, k_ap_ssid, k_ap_channel); break;
        case PAGE_SET:    oled_show_settings(s_cursor); break;
        case PAGE_DEMO:   oled_show_demo(s_demo_scene, s_script_step,
                                         (s_script_step >= 0)
                                             ? k_script[s_script_step].name
                                             : NULL,
                                         s_demo_mute, s_demo);
                          break;
        default:          oled_show_status(snap); break;
    }
}

/* ------------------------------------------------------------------
 * 事件消费
 * ------------------------------------------------------------------ */

static void handle_event(const ui_event_t *ev)
{
    if (ev->type == UI_EVT_SNAP) {
        s_demo      = (ev->demo != 0);
        s_demo_mute = (ev->mute != 0);
        s_demo_scene = ev->scene;
        /* 注意：演示快照不得清掉设置模式——设置页优先级最高，
         * 操作者主动进入后只能由 KEY14 退出。 */
    } else if (ev->type == UI_EVT_KEY) {
        /* 每个按键都打日志：键位映射有问题时，直接读串口即可定位
         * （"按下丝印 KEYn，收到 KEYm" 一目了然，不必靠行为反推）。 */
        ESP_LOGI(TAG, "KEY = %u (%s mode)",
                 (unsigned)ev->key,
                 s_demo ? "demo"
                        : (s_settings_mode ? "setting" : "normal"));
        if (s_settings_mode) {
            handle_key_settings(ev->key);
        } else {
            handle_key_normal(ev->key);
        }
    }
}

/* ------------------------------------------------------------------
 * ui 任务：事件消费 + 快照 + 页面渲染（250ms）
 * ------------------------------------------------------------------ */

static void ui_task(void *arg)
{
    oled_status_t snap;
    uint32_t prev_packets = 0;
    bool     visual_done  = false;
    bool     have_prev    = false;
    bool     prev_alarm   = false;
    int64_t  clear_ms     = 0;

    (void)arg;
    memset(s_hist, 0, sizeof(s_hist));

    for (;;) {
        ui_event_t ev;
        while (xQueueReceive(s_evt_q, &ev, 0) == pdTRUE) {
            handle_event(&ev);
        }

        /* OLED 对比度与边缘补偿需面板就绪后下发一次（X-SHIFT 变化时
         * 设置模块会置整屏重刷标志，这里无需重复处理） */
        if (!visual_done && oled_ready()) {
            settings_apply_visual();
            visual_done = true;
        }

        /* 快照：判定状态取自共享结构（互斥保护），外设状态直取 */
        base_station_get_status(&snap);

        snap.fpga_ok = base_station_fpga_ok();
#if ENABLE_SD_LOG
        /* SD 在位探活（约 2s 一次）：直接读卡引导扇区判断，真实硬件访问，
         * 不依赖 fflush 之类的间接推断。拔卡 -> 显示"存储卡未插入"；
         * 插回 -> 自动重新挂载并恢复显示。 */
        {
            static int64_t probe_ms = 0;
            int64_t now_ms = esp_timer_get_time() / 1000LL;
            if (probe_ms == 0 || now_ms - probe_ms >= 2000LL) {
                probe_ms = now_ms;
                (void)sd_log_probe();
            }
        }
        snap.sd_ok      = sd_log_ready();
        snap.sd_dropped = sd_log_dropped();
        snprintf(snap.sd_file, sizeof(snap.sd_file), "%s", sd_log_filename());
#else
        snap.sd_ok      = false;
        snap.sd_dropped = 0;
        snprintf(snap.sd_file, sizeof(snap.sd_file), "%s", "-");
#endif
        snap.ui_alive = fpga_uplink_alive();

        /* 数据新鲜度（与 FPGA 3s 老化同口径：DATA_TIMEOUT_MS 无帧即停摆） */
        {
            int64_t now_ms = esp_timer_get_time() / 1000LL;
            snap.data_fresh =
                (snap.last_frame_ms != 0u &&
                 (now_ms - (int64_t)snap.last_frame_ms) < (int64_t)3000u);
        }

        /* 有效静音 = FPGA 拨码上报位 | 设置项蜂鸣器使能取反 */
        {
            ui_settings_t cfg;
            settings_get(&cfg);
            snap.mute_effective =
                (uint8_t)(s_demo_mute || (cfg.buzzer_en == 0u));
        }

        /* 告警解除提示（P0 大字区显示「解除」2s；重新告警立即清除） */
        {
            int64_t now_ms = esp_timer_get_time() / 1000LL;
            if (snap.alarm) {
                prev_alarm = true;
                if (clear_ms != 0) {
                    oled_set_hint(NULL);
                    clear_ms = 0;
                }
            } else if (prev_alarm) {
                prev_alarm = false;
                clear_ms = now_ms;
                oled_set_hint("解除");
            } else if (clear_ms != 0 && now_ms - clear_ms > 2000) {
                oled_set_hint(NULL);
                clear_ms = 0;
            }
        }

        /* 实测帧率：包数差分（250ms 周期 -> fps_x10 = delta * 40） */
        if (have_prev && snap.packets >= prev_packets) {
            uint32_t d = snap.packets - prev_packets;
            snap.fps_x10 = (uint8_t)((d > 25u) ? 250u : d * 40u);
        } else {
            snap.fps_x10 = 0;
        }
        prev_packets = snap.packets;
        have_prev = true;

        /* 幅值历史（0..15，供 P1 条形图；线性移位，末元素最新；
         * 数据流停摆期清零，条形图随之熄灭） */
        {
            uint8_t lvl = (uint8_t)(snap.data_fresh
                                        ? ((snap.mag_x100 > 3000u)
                                               ? 15u
                                               : (snap.mag_x100 / 200u))
                                        : 0u);
            memmove(s_hist, s_hist + 1, HIST_N - 1u);
            s_hist[HIST_N - 1u] = lvl;
        }

        render(resolve_page(&snap), &snap);

        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------
 * 演示剧本任务（场景 0：全链路自动演示）
 * ------------------------------------------------------------------ */

static void script_enter(int step)
{
    switch (step) {
        case 0: fpga_link_send_state_telemetry(FALL_STATE_IDLE, 100u); break;
        case 1: fpga_link_send_state_telemetry(FALL_STATE_WALK, 124u); break;
        case 2: fpga_link_send_state_telemetry(FALL_STATE_PENDING, 340u); break;
        case 3: fpga_link_send_alarm(); break;
        case 4:
            /* 唯一的真实状态守卫：真实告警锁存期间不得掐断其蜂鸣
             * （人还在地上）。重叠期 FPGA 本就被真实告警覆盖，
             * 该抑制对演示画面不可见。 */
            if (!base_station_alarm_active()) {
                fpga_link_send_clear();
            }
            fpga_link_send_state_telemetry(FALL_STATE_IDLE, 100u);
            break;
        default: break;
    }
    s_script_step = step;
    ESP_LOGI(TAG, "DEMO step %d %s", step, k_script[step].name);
}

static void demo_task(void *arg)
{
    int step = -1;
    int64_t t_enter = 0;

    (void)arg;
    for (;;) {
        bool active = (s_demo && s_demo_scene == 0);

        if (!active) {
            if (step >= 0) {
                s_script_step = -1;
                step = -1;
                g_ui_demo_owns_state = false;
                /* 退出演示：向 FPGA 重同步真实状态（含哨兵语义由真实
                 * 遥测刷新恢复；此处先恢复状态码显示） */
                fpga_link_send_state_telemetry(base_station_real_state(),
                                               0u);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        g_ui_demo_owns_state = true;

        /* 演示豁免：真实状态（含真实告警）不打断、不暂停剧本。
         * 真实 ALARM/CLEAR 仍直通 FPGA 物理输出（安全穿透）；
         * 剧本 CLEAR 的安全守卫在 script_enter() 内。 */
        int64_t now = esp_timer_get_time() / 1000LL;
        if (step < 0) {
            step = 0;
            t_enter = now;
            script_enter(step);
        } else if (now - t_enter >= (int64_t)k_script[step].dur_ms) {
            step = (step + 1 >= (int)SCRIPT_N) ? 0 : step + 1;
            t_enter = now;
            script_enter(step);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ------------------------------------------------------------------
 * 启动
 * ------------------------------------------------------------------ */

void ui_app_start(void)
{
    if (!oled_ready()) {
        ESP_LOGW(TAG, "OLED not ready, UI tasks not started");
        return;
    }
    if (s_evt_q != NULL) {
        return;                     /* 已启动 */
    }
    s_evt_q = xQueueCreate(8, sizeof(ui_event_t));
    if (s_evt_q == NULL) {
        ESP_LOGE(TAG, "event queue create failed");
        return;
    }

    (void)fpga_uplink_init(s_evt_q);
    xTaskCreate(ui_task, "ui", 4096, NULL, 2, NULL);
    xTaskCreate(demo_task, "demo", 3072, NULL, 2, NULL);
    ESP_LOGI(TAG, "ui tasks started");
}

#else /* !ENABLE_OLED */

void ui_app_start(void)
{
    /* OLED 关闭：UI 任务不启动；键盘场景选择等 FPGA 本地功能不受影响 */
}

int ui_demo_script_step(void)
{
    return -1;
}

#endif /* ENABLE_OLED */
