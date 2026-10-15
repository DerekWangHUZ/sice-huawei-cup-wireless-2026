/*
 * csi_stub.c -- CSI 占位实现
 *
 * 详见 csi_stub.h 的接入点说明。本期不实现任何真实 CSI 逻辑：
 *   - csi_feed_packet()   空操作
 *   - csi_get_variance()  返回常数 0
 *   - csi_ready()         恒 false
 *
 * 主程序通过 csi_ready() 判断，不会把占位值当作真实判据使用，
 * 因此不会出现"占位数据导致误判"的隐患。
 */

#include "csi_stub.h"

/* 占位统计计数器：保留字段以便真实实现直接替换，
   同时让日志能显示"已调用次数"，便于验证调用链确实接通。 */
static uint32_t s_packet_count = 0;

void csi_feed_packet(void)
{
    s_packet_count++;
    /* TODO(真实 CSI)：在此读取 Wi-Fi CSI 回调累积的统计量 */
}

uint32_t csi_get_variance(void)
{
    /* TODO(真实 CSI)：返回 64 子载波幅度方差 */
    return 0u;
}

bool csi_ready(void)
{
    /* TODO(真实 CSI)：回调注册成功且已收到数据后返回 true */
    return false;
}
