/*
 * csi_stats.c -- CSI 子载波幅度方差统计实现
 *
 * 纯 C99（约束见 csi_stats.h 头注）。整数实现细节：
 *   幅度 |h| = fall_isqrt_u32(re^2 + im^2)（复用 fall_core 的逐位开方，
 *   同算法不存两份拷贝）；方差用一阶公式
 *     var = (N*Σx^2 - (Σx)^2) / N^2
 *   避免两遍扫描（回调上下文只需一次遍历累加 Σx 与 Σx^2）。
 *
 * 溢出分析（int8 IQ、CSI_STATS_MAX_SUB=128 上限）：
 *   |re|,|im| <= 127 -> |h| <= isqrt(2*127^2) = 179
 *   Σx   <= 128 * 179 = 22912
 *   Σx^2 <= 128 * 179^2 = 4101248
 *   N*Σx^2 <= 128 * 4101248 ≈ 5.25e8，(Σx)^2 <= 5.25e8，
 *   均 < 2^32，用 uint64 承载乘加后仍余量巨大，安全。
 *   单帧方差 <= max|h|^2 = 32041，窗口和 <= 10 * 32041 = 320410。
 */

#include "csi_stats.h"

#include "fall_core.h"

/* 单帧待结算快照（accumulate 写、commit 读；互斥由调用方保证） */
static uint32_t s_pend_sum;
static uint32_t s_pend_sumsq;
static uint16_t s_pend_n;
static bool     s_pend_fresh;

/* 已入窗的单帧方差环形缓冲 */
static uint32_t s_ring[CSI_STATS_WINDOW];
static uint16_t s_ring_n;
static uint16_t s_ring_head;   /* 下一写入位置 */

bool csi_stats_accumulate(const int8_t *iq, uint16_t n_sub)
{
    uint32_t sum   = 0;
    uint32_t sumsq = 0;
    uint16_t i;

    if (iq == NULL || n_sub == 0 || n_sub > CSI_STATS_MAX_SUB) {
        return false;
    }

    for (i = 0; i < n_sub; i++) {
        int32_t re = iq[2u * i];
        int32_t im = iq[2u * i + 1u];
        uint32_t amp = fall_isqrt_u32((uint32_t)(re * re + im * im));
        sum   += amp;
        sumsq += amp * amp;
    }

    s_pend_sum   = sum;
    s_pend_sumsq = sumsq;
    s_pend_n     = n_sub;
    s_pend_fresh = true;
    return true;
}

void csi_stats_commit(void)
{
    uint64_t n      = s_pend_n;
    uint64_t var;

    if (!s_pend_fresh || s_pend_n == 0) {
        return;   /* 没有新快照：空操作，不重复入窗 */
    }

    /* var = (N*Σx^2 - (Σx)^2) / N^2，整数截断（统计判据用，精度足够） */
    var = (n * s_pend_sumsq - (uint64_t)s_pend_sum * s_pend_sum) / (n * n);

    s_ring[s_ring_head] = (uint32_t)var;
    s_ring_head = (uint16_t)((s_ring_head + 1u) % CSI_STATS_WINDOW);
    if (s_ring_n < CSI_STATS_WINDOW) {
        s_ring_n++;
    }
    s_pend_fresh = false;
}

uint32_t csi_stats_variance(void)
{
    uint32_t total = 0;
    uint16_t i;

    if (s_ring_n == 0) {
        return 0;
    }
    for (i = 0; i < s_ring_n; i++) {
        total += s_ring[i];
    }
    return total / s_ring_n;
}

bool csi_stats_ready(void)
{
    return s_ring_n != 0;
}

void csi_stats_reset(void)
{
    s_pend_sum   = 0;
    s_pend_sumsq = 0;
    s_pend_n     = 0;
    s_pend_fresh = false;
    s_ring_n     = 0;
    s_ring_head  = 0;
}
