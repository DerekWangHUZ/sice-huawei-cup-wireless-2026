/*
 * fpga_link.c -- S3 -> FPGA UART 控制帧发送实现
 *
 * 协议见 docs/团队协作规范与接口契约.md §4。
 * 关键点：
 *   1) 实际只用 TX，但 legacy UART 驱动要求 rx_buffer_size > HW FIFO
 *      （S3 FIFO=128，传 0 安装直接失败，实测报
 *      "uart rx buffer length error"）——分配最小合法值、永不读取。
 *   2) 帧间隔保护：两次发送至少间隔 FPGA_TX_GAP_MS，防止 FPGA 端
 *      8 字节组帧窗口被连续帧打乱（契约 §4.5 明确要求）。
 *   3) XOR = cmd ^ p1 ^ p2_lo ^ p2_hi（Byte2..Byte5 四字节）。
 */

#include "fpga_link.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "FPGA";

/* 上一次发送的微秒时间戳，用于帧间隔保护 */
static int64_t s_last_tx_us = 0;

/* RX 缓冲：仅满足驱动安装的合法性检查（须 > HW FIFO 128），永不读取 */
#define FPGA_UART_RX_BUF   256

/* 协议常量（与 docs/团队协作规范与接口契约.md §4 严格一致） */
#define FRAME_HDR0   0xAAu
#define FRAME_HDR1   0x55u
#define FRAME_TAIL   0x0Du

#define CMD_ALARM     0x01u
#define CMD_CLEAR     0x02u
#define CMD_SET_AUDIO 0x03u
#define CMD_SET_STATE 0x10u

/* 发送互斥：收包判定任务 / 演示剧本任务 / 设置模块都会下发帧 */
static SemaphoreHandle_t s_tx_mtx = NULL;

esp_err_t fpga_link_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = FPGA_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(FPGA_UART_PORT, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(FPGA_UART_PORT,
                       FPGA_UART_TX_GPIO,   /* tx -> FPGA rxd */
                       FPGA_UART_RX_GPIO,   /* rx <- FPGA 上行 UI 帧 */
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    /* TX + RX：RX 缓冲 > HW FIFO（驱动合法性要求），由 fpga_uplink 任务读取 */
    err = uart_driver_install(FPGA_UART_PORT, FPGA_UART_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    if (s_tx_mtx == NULL) {
        s_tx_mtx = xSemaphoreCreateMutex();
        if (s_tx_mtx == NULL) {
            ESP_LOGE(TAG, "tx mutex create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "UART link ready: port=%d tx=%d rx=%d baud=%d",
             FPGA_UART_PORT, FPGA_UART_TX_GPIO, FPGA_UART_RX_GPIO,
             FPGA_UART_BAUDRATE);
    return ESP_OK;
}

esp_err_t fpga_link_send_cmd(uint8_t cmd, uint8_t p1, uint16_t p2)
{
    return fpga_link_send_cmd_ex(cmd, p1, p2, true);
}

esp_err_t fpga_link_send_cmd_ex(uint8_t cmd, uint8_t p1, uint16_t p2, bool verbose)
{
    uint8_t  p2_lo = (uint8_t)(p2 & 0xFFu);
    uint8_t  p2_hi = (uint8_t)((p2 >> 8) & 0xFFu);
    uint8_t  x     = (uint8_t)(cmd ^ p1 ^ p2_lo ^ p2_hi);
    uint8_t  frame[8];
    int      written;

    frame[0] = FRAME_HDR0;
    frame[1] = FRAME_HDR1;
    frame[2] = cmd;
    frame[3] = p1;
    frame[4] = p2_lo;
    frame[5] = p2_hi;
    frame[6] = x;
    frame[7] = FRAME_TAIL;

    if (s_tx_mtx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /*
     * 用有限等待而非 portMAX_DELAY：若持锁任务异常卡住（例如 UART 驱动
     * 阻塞），无限等待会让所有 FPGA 发送永久挂起，告警帧发不出去。
     * 200ms 足够覆盖正常竞争（一帧 8 字节 @115200 约 0.7ms），
     * 超时则放弃本帧——告警语义由状态机重发兜底。
     */
    if (xSemaphoreTake(s_tx_mtx, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /*
     * 帧间隔保护：距上次发送不足 FPGA_TX_GAP_MS 则补足等待。
     * 等待仍在互斥量内进行——帧间隔是多任务共享的全局约束，不能让
     * 别的任务插队；与旧实现的差别只是把"持锁自旋烧 CPU"换成
     * 每轮 vTaskDelay(1) 睡 1 tick 后重算剩余时间（8 字节 @115200
     * 约 0.7ms + 2ms 间隔，等待本身极短，最多多睡一个 tick，
     * 对 FPGA 侧"至少间隔 gap"的语义无影响——只会更长不会更短）。
     */
    if (s_last_tx_us != 0) {
        int64_t gap_us     = (int64_t)FPGA_TX_GAP_MS * 1000;
        int64_t remain_us  = s_last_tx_us + gap_us - esp_timer_get_time();
        while (remain_us > 0) {
            vTaskDelay(1);
            remain_us = s_last_tx_us + gap_us - esp_timer_get_time();
        }
    }

    written = uart_write_bytes(FPGA_UART_PORT, frame, sizeof(frame));
    s_last_tx_us = esp_timer_get_time();
    xSemaphoreGive(s_tx_mtx);

    if (written != (int)sizeof(frame)) {
        ESP_LOGW(TAG, "short write: %d bytes", written);
        return ESP_FAIL;
    }

    if (verbose) {
        ESP_LOGI(TAG, "TX cmd=0x%02X p1=0x%02X p2=0x%04X xor=0x%02X "
                      "[%02X %02X %02X %02X %02X %02X %02X %02X]",
                 cmd, p1, p2, x,
                 frame[0], frame[1], frame[2], frame[3],
                 frame[4], frame[5], frame[6], frame[7]);
    } else {
        /* 1Hz 周期遥测刷新：降到 DEBUG，避免刷屏 */
        ESP_LOGD(TAG, "TX(tel) p1=0x%02X p2=0x%04X xor=0x%02X",
                 p1, p2, x);
    }
    return ESP_OK;
}

esp_err_t fpga_link_send_alarm(void)
{
    return fpga_link_send_cmd(CMD_ALARM, 0x00, 0x0000);
}

esp_err_t fpga_link_send_clear(void)
{
    return fpga_link_send_cmd(CMD_CLEAR, 0x00, 0x0000);
}

esp_err_t fpga_link_send_state(fall_state_t st)
{
    return fpga_link_send_cmd(CMD_SET_STATE, (uint8_t)st, 0x0000);
}

esp_err_t fpga_link_send_state_telemetry(fall_state_t st, uint16_t mag_x100)
{
    /* 周期遥测刷新（1Hz）：与状态变化边沿发的是同一种帧，p2 携带合加速度
     * 幅值（0.01g）。FPGA 侧对重复的 SET_STATE 完全幂等，只做锁存。
     * 契约 §4.5。日志降到 DEBUG，避免 1Hz 刷屏。 */
    return fpga_link_send_cmd_ex(CMD_SET_STATE, (uint8_t)st, mag_x100, false);
}

esp_err_t fpga_link_send_state_change(fall_state_t st, uint16_t mag_x100)
{
    /* 状态变化沿（契约 §4.5 第 1 条）：保留逐帧 INFO 日志，
     * 作为"状态同步已下发"的验收信号。 */
    return fpga_link_send_cmd_ex(CMD_SET_STATE, (uint8_t)st, mag_x100, true);
}

esp_err_t fpga_link_send_audio(uint8_t mute, uint8_t pend_beep, uint16_t tone_half_div)
{
    /* 0x03 SET_AUDIO：bit0=静音 bit1=待确认提示音，p2=载波半周期（0=默认） */
    return fpga_link_send_cmd(CMD_SET_AUDIO,
                              (uint8_t)((mute ? 0x01u : 0u) |
                                        (pend_beep ? 0x02u : 0u)),
                              tone_half_div);
}
