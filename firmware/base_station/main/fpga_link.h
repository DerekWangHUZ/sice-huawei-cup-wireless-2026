/*
 * fpga_link.h -- ESP32-S3 到 Artix-7 FPGA 的 UART 控制链路
 *
 * 协议权威定义见 docs/团队协作规范与接口契约.md §4，本模块逐字段实现：
 *   8 字节定长帧：AA 55 | cmd | p1 | p2_lo p2_hi | xor | 0D
 *   xor = cmd ^ p1 ^ p2_lo ^ p2_hi
 *   115200 8N1，S3 -> FPGA 单向
 *
 * 命令：
 *   0x01 ALARM      告警触发（数码管 "AL" 整屏急闪、蜂鸣器响、LED 全亮）
 *   0x02 CLEAR      告警解除
 *   0x10 SET_STATE  同步判定状态（p1 = 状态码 0..3）
 *                   p2 = 合加速度幅值，单位 0.01g（1.00g -> 100），
 *                        0 表示无遥测。数码管读数位即显示该值，
 *                        见 docs/团队协作规范与接口契约.md §4.3/§4.8。
 */

#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "fall_core.h"

/* --- 硬件连接（改板子接线时只改这里） --- */
#define FPGA_UART_PORT      UART_NUM_1
#define FPGA_UART_TX_GPIO   17        /* S3 的 TX -> FPGA rxd(B15，右扩展排母上排第 1 脚，契约 §4.1；共地) */
#define FPGA_UART_RX_GPIO   (-1)      /* 本期不接收，不分配 RX 引脚 */
#define FPGA_UART_BAUDRATE  115200

/* 帧间隔保护：协议要求 >= 1ms，这里留裕量取 2ms */
#define FPGA_TX_GAP_MS      2u

/* SET_STATE 的 p2 哨兵：表示"暂无有效遥测"，FPGA 数码管读数位显示 "----" */
#define FPGA_TEL_NO_DATA    0xFFFFu

/* 初始化 UART（TX only）。返回 ESP_OK 表示成功。 */
esp_err_t fpga_link_init(void);

/* 发送一条命令帧。内部自动算 XOR，并保证帧间隔。 */
esp_err_t fpga_link_send_cmd(uint8_t cmd, uint8_t p1, uint16_t p2);

/* 同上，但可关闭逐帧 INFO 日志（周期遥测刷新用，避免刷屏） */
esp_err_t fpga_link_send_cmd_ex(uint8_t cmd, uint8_t p1, uint16_t p2, bool verbose);

/* 便捷封装 */
esp_err_t fpga_link_send_alarm(void);                    /* 0x01 */
esp_err_t fpga_link_send_clear(void);                    /* 0x02 */
esp_err_t fpga_link_send_state(fall_state_t st);         /* 0x10，p2=0 */

/* 0x10 + 遥测：p2 = 合加速度幅值（0.01g）。
 * 以 1Hz 周期调用，让数码管读数持续刷新（契约 §4.5）。日志为 DEBUG。 */
esp_err_t fpga_link_send_state_telemetry(fall_state_t st, uint16_t mag_x100);

/* 0x10 + 遥测，日志为 INFO。状态变化沿用这个，
 * 保留既有验收信号 `TX cmd=0x10 p1=...`。 */
esp_err_t fpga_link_send_state_change(fall_state_t st, uint16_t mag_x100);

#endif /* FPGA_LINK_H */
