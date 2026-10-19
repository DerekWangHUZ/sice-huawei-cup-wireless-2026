/*
 * fpga_uplink.c -- FPGA -> S3 上行 UI 帧接收实现
 *
 * UART1 驱动由 fpga_link_init() 安装（RX 缓冲 256）；本模块只负责读取、
 * 解帧、投递事件。收包任务优先级低（2），50ms 空闲超时轮询——上行流量
 * 只有按键/拨码事件与 1s 心跳，带宽占用可忽略。
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

static QueueHandle_t s_evt_q = NULL;
static volatile int64_t s_last_ms = 0;
static volatile uint32_t s_drop = 0;

int64_t fpga_uplink_last_ms(void)
{
    return s_last_ms;
}

bool fpga_uplink_alive(void)
{
    if (s_last_ms == 0) {
        return false;
    }
    return (esp_timer_get_time() / 1000LL - s_last_ms) < 3000LL;
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
    uint8_t byte;

    (void)arg;
    for (;;) {
        int n = uart_read_bytes(FPGA_UART_PORT, &byte, 1,
                                pdMS_TO_TICKS(50));
        if (n == 1) {
            ui_frame_t f;
            ui_proto_result_t r = ui_proto_feed(byte, &f);
            if (r == UI_PROTO_OK) {
                s_last_ms = esp_timer_get_time() / 1000LL;
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
