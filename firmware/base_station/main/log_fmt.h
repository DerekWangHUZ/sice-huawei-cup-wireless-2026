/*
 * log_fmt.h -- SD 卡 CSV 记录格式（纯标准 C99，无平台依赖）
 *
 * 设计约束（重要）：
 *   本模块是【纯标准 C99】，只允许 <stdint.h> <stddef.h> <stdio.h>，
 *   不得包含任何 ESP-IDF / FreeRTOS 头文件。
 *   原因与 fall_core 相同：宿主机离线自测程序直接 #include 本文件编译，
 *   用它逐字节锁定 docs/团队协作规范与接口契约.md §2.1 定义的 CSV 格式，
 *   避免"文档一套、固件一套"的接口漂移。
 *
 * 契约 §2.1 原文：
 *   文件名 /DATA_001.CSV、/DATA_002.CSV（每次上电新建一个，递增编号）；
 *   首行表头 Timestamp_ms,Seq,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,CSI_Variance,State
 *
 * 字段取值约定（2026-10-17 与契约同步订正，见 CHANGELOG）：
 *   Timestamp_ms   上电起毫秒（esp_timer_get_time()/1000），uint32
 *   Seq            穿戴端包序号 uint16（丢包可直接看出来）
 *   AccX~GyroZ     原始 int16 ADC 值，不转 float（契约 §1.2 链路即原始值）
 *   CSI_Variance   当前 CSI 特征；本期 CSI 为占位，恒 0
 *   State          fall_state_t 原始值 0..3（与契约 §4.3 状态码一致）：
 *                  0=IDLE 1=WALK 2=FALL_PENDING 3=FALL_CONFIRMED
 */

#ifndef LOG_FMT_H
#define LOG_FMT_H

#include <stdint.h>
#include <stddef.h>

/*
 * 单行最大长度（含 NUL）。
 * 最坏情况实测上界 73 字节（ts 10 位 + seq 5 位 + 6 个 -32768 + csi 10 位
 * + state 1 位 + 9 个逗号 + '\n' + NUL），取 96 留裕量。
 */
#define LOG_FMT_LINE_MAX 96

/* 契约 §2.1 规定的表头行（含行尾 '\n'，不含 '\r'）。 */
const char *log_fmt_header(void);

/*
 * 按契约 §2.1 格式化一行数据（含行尾 '\n'，以 '\0' 结尾）。
 *
 *   buf/n   输出缓冲与容量（建议 LOG_FMT_LINE_MAX）
 *   ts_ms   上电起毫秒
 *   seq     穿戴端包序号
 *   acc     int16[3] 加速度原始值
 *   gyro    int16[3] 角速度原始值
 *   csi_var CSI 方差（本期占位 0）
 *   state   fall_state_t 原始值 0..3
 *
 * 返回 snprintf 语义的"本应写入长度"（不含 NUL）：
 *   返回值 >= n 表示缓冲不足、内容已被截断，调用方应视为失败。
 */
int log_fmt_row(char *buf, size_t n,
                uint32_t ts_ms, uint16_t seq,
                const int16_t acc[3], const int16_t gyro[3],
                uint32_t csi_var, uint8_t state);

#endif /* LOG_FMT_H */
