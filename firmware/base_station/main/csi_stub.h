/*
 * csi_stub.h -- CSI（Channel State Information）对外接口
 *
 * 实现状态（2026-10-25 更新）：
 *   真实提取【通路已实现、默认未启用】——csi_real.c + csi_stats.c 完成
 *   了"WiFi CSI 回调 -> 子载波幅度方差滑窗 -> csi_get_variance()"的完整
 *   数据通路，并由 main/CMakeLists.txt 的 BS_CSI_IMPL 在编译期选择：
 *     stub（默认）：本文件配套的 csi_stub.c，行为与历史版本完全一致；
 *     real        ：csi_real.c，SoftAP 侧采集 LLTF CSI（穿戴端 STA 的
 *                   50Hz 上行包即采集源，回调只对本机收发的帧触发）。
 *
 * 剩余为上机事项（本仓库纯离线验证，未上机）：
 *   1) 确认 S3 AP 模式下 CSI 回调确实触发（csi_ready() 变 true）；
 *   2) 标定 sigma_csi：采集"静止/活动/跌倒躺平"三种状态的
 *      CSI_Variance 序列，取分界值填 fall_core.h 的 CSI_FALL_VAR_MAX；
 *   3) 把 fall_core.h 的 CSI_JUDGE_ENABLE 置 1 启用辅助判据
 *      （契约 §2.2 第三段：跌倒后扰动方差 < sigma_csi）。
 *
 * 历史接入点说明（原 6 步 TODO，现由 csi_real.c/csi_stats.c 落实）：
 *   1) esp_wifi_set_csi(true) 开启采集            -> csi_init()
 *   2) esp_wifi_set_csi_config() 配置(LLTF only)   -> csi_init()
 *   3) esp_wifi_set_csi_rx_cb() 注册回调           -> csi_init()
 *   4) csi_info->buf 取子载波复数对算幅度          -> csi_stats_accumulate()
 *   5) 64 子载波幅度方差                            -> csi_stats_commit()
 *   6) 方差喂给 fall_detector 作辅助判据           -> fall_detector_set_csi()
 *
 * 注意：CSI 回调运行在 Wi-Fi 驱动上下文，只能做轻量计算，
 *       方差统计只累加（csi_stats_accumulate），除法在 getter 里做
 *       （csi_stats_commit / csi_stats_variance）。
 */

#ifndef CSI_STUB_H
#define CSI_STUB_H

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

/*
 * 初始化 CSI 采集（esp_wifi_start() 成功后调用一次）。
 * stub：空操作返回 ESP_OK；real：配置并注册 WiFi CSI 回调。
 * 失败时主程序仅记日志降级（CSI 缺失不影响 IMU 主判据）。
 */
esp_err_t csi_init(void);

/* 每收到一个姿态 UDP 包调用一次。
   真实实现：在此触发一次 CSI 快照结算 / 更新统计量。 */
void csi_feed_packet(void);

/* 获取当前 CSI 特征（64 子载波幅度方差）。
   真实实现：返回最近一次统计结果。
   占位实现：返回常数 0（表示"无 CSI 数据"）。 */
uint32_t csi_get_variance(void);

/* CSI 子系统是否已就绪（真实实现：回调注册成功且已收到过数据）。
   占位实现：恒 false，调用方据此跳过 CSI 相关判据。 */
bool csi_ready(void);

#endif /* CSI_STUB_H */
