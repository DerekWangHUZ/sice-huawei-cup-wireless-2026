/*
 * csi_stats.h -- CSI 子载波幅度方差统计（纯 C99，可离线单测）
 *
 * 设计约束（与 fall_core 相同的纪律）：
 *   本模块是【纯标准 C99】，不得包含任何 ESP-IDF / FreeRTOS / 平台头文件，
 *   只允许 <stdint.h> <stdbool.h>。同一份源码被宿主机离线测试程序直接
 *   #include 编译运行（tools/offline_test/test_harness.c），防止两份实现漂移。
 *
 * 统计口径（契约 §2.2 第三段，csi_stub.h 承诺的接口语义）：
 *   单帧 CSI 的所有子载波幅度 |h_i| = sqrt(re_i^2 + im_i^2)，
 *   方差 = (N*Σ|h|^2 - (Σ|h|)^2) / N^2（子载波间空间频率维方差）。
 *   人在穿戴端附近活动 -> 多径扰动 -> 子载波间幅度起伏大 -> 方差高；
 *   人跌倒后静止 -> 扰动消失 -> 方差回落到 sigma_csi 以下（辅助确认判据）。
 *
 * 并发约定（真实实现的锁由 csi_real.c 负责，本模块不含平台原语）：
 *   csi_stats_accumulate() 在 WiFi 驱动 CSI 回调上下文调用（只累加，
 *   不做除法，每子载波常数次整数运算）；csi_stats_commit() 与两个
 *   getter 在任务上下文调用（除法只发生在这两处）。调用方需保证
 *   accumulate 与 commit 互斥（csi_real.c 用 portMUX 短临界区实现）。
 */

#ifndef CSI_STATS_H
#define CSI_STATS_H

#include <stdint.h>
#include <stdbool.h>

/* 支持的最大子载波数：LLTF 52/64、HT-LTF 128 均覆盖（按 csi_info->len 动态取） */
#define CSI_STATS_MAX_SUB  128

/* 方差滑动窗口帧数：对单帧方差做窗口平均去抖（50Hz 上行 ~1 帧/包） */
#define CSI_STATS_WINDOW   10

/*
 * 喂入一帧 CSI 的子载波 IQ 对（WiFi 驱动回调上下文）。
 *   iq    : 交错 [re0, im0, re1, im1, ...] 的 int8 数组（wifi_csi_info_t->buf）
 *   n_sub : 子载波对数（csi_info->len / 2）
 * 返回 false = 参数非法被拒绝（n_sub 为 0 或超上限）。
 */
bool csi_stats_accumulate(const int8_t *iq, uint16_t n_sub);

/*
 * 把最近一次 accumulate 的快照结算为单帧方差并推入滑动窗口
 * （任务上下文；csi_feed_packet 的真实实现调用本函数）。
 * 自上次 commit 后没有新的 accumulate 时为空操作，不重复入窗
 * ——保证"UDP 包率 = 入窗率"的节流语义。
 */
void csi_stats_commit(void);

/* 窗口内方差的整数平均；无任何数据时返回 0。 */
uint32_t csi_stats_variance(void);

/* 已入窗至少一帧有效 CSI 数据。 */
bool csi_stats_ready(void);

/* 全量复位（离线单测用）。 */
void csi_stats_reset(void);

#endif /* CSI_STATS_H */
