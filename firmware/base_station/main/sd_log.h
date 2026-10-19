/*
 * sd_log.h -- microSD（六脚 SPI 模块）CSV 数据记录
 *
 * 契约依据：docs/团队协作规范与接口契约.md §2.1
 *   文件名  /DATA_001.CSV、/DATA_002.CSV（每次上电新建一个，递增编号）
 *   表头    由 log_fmt.h 集中定义
 *
 * 设计要点（实时性，重要）：
 *   写卡（SPI + FATFS）单次耗时可达数十甚至上百毫秒，若直接在 50Hz 收包任务里
 *   f_write，会阻塞 UDP 收包与 fall_detector_tick 的 100ms 节拍，破坏跌倒判定
 *   时序。因此采用【生产者-消费者】：
 *     udp_pose_task  -> sd_log_push() 非阻塞入队（超时 0）
 *     sd_log_task    -> 低优先级出队落盘
 *   队列满即丢并累加 sd_dropped()，绝不反压收包/判定路径。
 *   详见 docs/CHANGELOG.md [2026-10-17] 决策 D3。
 *
 * 容错：未插卡/挂载失败 -> 记录禁用，主链路照常；
 *       运行中写失败 -> 关闭文件并停止记录（不做紧循环重试）。
 */

#ifndef SD_LOG_H
#define SD_LOG_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/* 一条待落盘记录（固定长度，供无分裂环形缓冲使用） */
typedef struct {
    uint32_t ts_ms;      /* 上电起毫秒 */
    uint16_t seq;        /* 穿戴端包序号 */
    int16_t  acc[3];     /* 加速度原始值 */
    int16_t  gyro[3];    /* 角速度原始值 */
    uint32_t csi_var;    /* CSI 方差（本期占位 0） */
    uint8_t  state;      /* fall_state_t 原始值 0..3 */
} sd_log_item_t;

/*
 * 挂载 SD 卡、建文件、写表头、建队列与记录任务。
 * 返回 ESP_OK 表示记录已启动；失败仅告警，不致命。
 */
esp_err_t sd_log_init(void);

/* 记录是否处于可用（已挂载且未发生写错误）状态 */
bool sd_log_ready(void);

/*
 * 卡在位探活：六脚模块无卡检测脚，拔卡后固件原本无从得知。
 * 用一次廉价的 f_sync 判断，失败即降级（sd_log_ready() 随之返回 false，
 * OLED 主页显示"内存卡未插入"）。由 UI 任务低频（约 5s）调用。
 * 返回 true = 仍可用。
 */
bool sd_log_probe(void);

/* 生产者接口：非阻塞入队。队列满则丢弃并计数。可从任务上下文调用。 */
void sd_log_push(const sd_log_item_t *item);

/* 因队列满而丢弃的记录条数（与 UDP 丢包无关） */
uint32_t sd_log_dropped(void);

/* 当前文件名（如 "DATA_001.CSV"）；未挂载时返回 "-" */
const char *sd_log_filename(void);

#endif /* SD_LOG_H */
