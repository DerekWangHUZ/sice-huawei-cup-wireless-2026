/*
 * fpga_link.h -- ESP32-S3 到 Artix-7 FPGA 的 UART 控制链路
 *
 * 协议权威定义见 docs/团队协作规范与接口契约.md §4，本模块逐字段实现：
 *   8 字节定长帧：AA 55 | cmd | p1 | p2_lo p2_hi | xor | 0D
 *   xor = cmd ^ p1 ^ p2_lo ^ p2_hi
 *   115200 8N1，S3 -> FPGA 单向
 *
 * 命令：
 *   0x01 ALARM      告警触发（数码管闪 FALL--、蜂鸣器响、LED 全亮）
 *   0x02 CLEAR      告警解除
 *   0x10 SET_STATE  同步判定状态（p1 = 状态码 0..3）
 */

#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>
#include "esp_err.h"
#include "fall_core.h"

/* --- 硬件连接（改板子接线时只改这里） --- */
#define FPGA_UART_PORT      UART_NUM_1
#define FPGA_UART_TX_GPIO   17        /* S3 的 TX -> FPGA rxd(F13) */
#define FPGA_UART_RX_GPIO   (-1)      /* 本期不接收，不分配 RX 引脚 */
#define FPGA_UART_BAUDRATE  115200

/* 帧间隔保护：协议要求 >= 1ms，这里留裕量取 2ms */
#define FPGA_TX_GAP_MS      2u

/* 初始化 UART（TX only）。返回 ESP_OK 表示成功。 */
esp_err_t fpga_link_init(void);

/* 发送一条命令帧。内部自动算 XOR，并保证帧间隔。 */
esp_err_t fpga_link_send_cmd(uint8_t cmd, uint8_t p1, uint16_t p2);

/* 便捷封装 */
esp_err_t fpga_link_send_alarm(void);                    /* 0x01 */
esp_err_t fpga_link_send_clear(void);                    /* 0x02 */
esp_err_t fpga_link_send_state(fall_state_t st);         /* 0x10 */

#endif /* FPGA_LINK_H */
