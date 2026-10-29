/*
 * fpga_uplink.c -- FPGA -> S3 上行 UI 帧接收实现
 *
 * UART1 驱动由 fpga_link_init() 安装（RX 缓冲 256）；本模块只负责读取、
 * 解帧、投递事件。收包任务优先级低（2），50ms 空闲超时轮询——上行流量
 * 只有按键/拨码事件与 1s 心跳，带宽占用可忽略。
 *
 * 读取方式：一次最多读 32 字节再逐字节喂解析器。UART 事件以突发到达
 * （8 字节一帧），逐字节读会让每帧产生 8 次驱动调用；批量读不改变
 * 解析语义（ui_proto 本来就是流式逐字节状态机）。
 *
 * s_last_ms 为 64 位时间戳，在 32 位 Xtensa 上跨任务读写存在撕裂
 * 可能（仅存活判据使用，风险低），用短临界区收敛掉。
 */

#include "fpga_uplink.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "fpga_link.h"
#include "ui_proto.h"

static const char *TAG = "UIRX";

/* 单次批量读取的最大字节数：一个 8 字节帧的 4 倍，够摊薄驱动调用，
 * 又不至于在极端背压下长时间占用本任务。 */
#define UPLINK_READ_CHUNK  32

static QueueHandle_t s_evt_q = NULL;
static volatile int64_t s_last_ms = 0;
static volatile uint32_t s_drop = 0;
static portMUX_TYPE s_ts_lock = portMUX_INITIALIZER_UNLOCKED;

int64_t fpga_uplink_last_ms(void)
{
    int64_t v;

    portENTER_CRITICAL(&s_ts_lock);
    v = s_last_ms;
    portEXIT_CRITICAL(&s_ts_lock);
    return v;
}

bool fpga_uplink_alive(void)
{
    int64_t last = fpga_uplink_last_ms();

    if (last == 0) {
        return false;
    }
    return (esp_timer_get_time() / 1000LL - last) < 3000LL;
}

static void uplink_post(const ui_frame_t *f)
{
    ui_event_t ev;

    if (s_evt_q == NULL) {
        return;
    }
    memset(&ev, 0, sizeof(ev));
    if (f->cmd == UI_CMD_SNAP) {
        ev.type  = UI_EVT_SNAP;
        ev.demo  = UI_SNAP_DEMO(f->p2);
        ev.mute  = UI_SNAP_MUTE(f->p2);
        ev.scene = UI_SNAP_SCENE(f->p2);
    } else {
        ev.type = UI_EVT_KEY;
        ev.key  = UI_KEY_CODE(f->p2);
    }
    if (xQueueSend(s_evt_q, &ev, 0) != pdTRUE) {
        s_drop++;
        if ((s_drop % 20u) == 1u) {
            ESP_LOGW(TAG, "event queue full, dropped=%u", (unsigned)s_drop);
        }
    }
}

static void uplink_task(void *arg)
{
    uint8_t buf[UPLINK_READ_CHUNK];

    (void)arg;
    for (;;) {
        int n = uart_read_bytes(FPGA_UART_PORT, buf, sizeof(buf),
                                pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            ui_frame_t f;
            ui_proto_result_t r = ui_proto_feed(buf[i], &f);
            if (r == UI_PROTO_OK) {
                int64_t now = esp_timer_get_time() / 1000LL;
                portENTER_CRITICAL(&s_ts_lock);
                s_last_ms = now;
                portEXIT_CRITICAL(&s_ts_lock);
                uplink_post(&f);
            } else if (r == UI_PROTO_BAD) {
                /* 坏帧计数仅日志限流，滑动窗口自动重同步 */
                ESP_LOGD(TAG, "bad uplink frame");
            }
        }
    }
}

esp_err_t fpga_uplink_init(QueueHandle_t evt_q)
{
    if (evt_q == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_evt_q = evt_q;

    if (xTaskCreate(uplink_task, "fpga_up", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "create uplink task failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "uplink reader started (rx gpio=%d)",
             FPGA_UART_RX_GPIO);
    return ESP_OK;
}
