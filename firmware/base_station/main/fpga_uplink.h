/*
 * fpga_uplink.h -- FPGA -> S3 上行 UI 帧接收
 *
 * 从 UART1（与下行共用外设，RX=FPGA_UART_RX_GPIO）读取字节流，
 * 经 ui_proto 滑动窗口解帧后把事件投入 ui_app 的事件队列。
 * 快照帧兼作上行链路心跳：fpga_uplink_alive_ms() 供 UI 显示链路状态。
 */

#ifndef FPGA_UPLINK_H
#define FPGA_UPLINK_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

#define UI_EVT_SNAP 1u   /* p2 快照：demo/mute/scene */
#define UI_EVT_KEY  2u   /* 键按下事件 */

typedef struct {
    uint8_t type;      /* UI_EVT_SNAP / UI_EVT_KEY */
    uint8_t key;       /* UI_EVT_KEY：键号 0..15（ui_proto.h 键表） */
    uint8_t demo;      /* UI_EVT_SNAP：演示模式使能 */
    uint8_t mute;      /* UI_EVT_SNAP：FPGA 本地静音 */
    uint8_t scene;     /* UI_EVT_SNAP：当前演示场景 0..7 */
} ui_event_t;

/* 启动接收任务。evt_q 由 ui_app 创建并传入；队列满时事件丢弃（计数）。 */
esp_err_t fpga_uplink_init(QueueHandle_t evt_q);

/* 最近一次收到合法上行帧的毫秒时刻（esp_timer 口径），0 = 尚未收到 */
int64_t fpga_uplink_last_ms(void);

/* 上行链路是否存活（3s 内有快照心跳） */
bool fpga_uplink_alive(void);

#endif /* FPGA_UPLINK_H */
