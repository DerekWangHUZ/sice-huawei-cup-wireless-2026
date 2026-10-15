/*
 * csi_stub.h -- CSI（Channel State Information）占位接口
 *
 * 本期说明：
 *   真实 CSI 提取本期【不做】——需要穿戴端在场持续发包才能采集到有意义
 *   的空间多径特征，且基站侧 CSI 回调要与 Wi-Fi 驱动深度耦合。
 *   因此这里只提供稳定接口 + 占位实现，保证 S3 主程序可以完整编译、
 *   完整跑通主链路，后续接入真实 CSI 时只需替换本文件实现。
 *
 * 真实实现接入点（TODO，留给有穿戴端在场的阶段）：
 *   1) 在 wifi 初始化时 esp_wifi_set_csi(true) 开启 CSI 采集；
 *   2) 用 esp_wifi_set_csi_config() 配置采集参数（LLTF/HT-LTF/STBC 等）；
 *   3) 注册 esp_wifi_set_csi_rx_cb() 回调，在回调里拿到 wifi_csi_info_t；
 *   4) 从 csi_info->buf 取出子载波复数对（实部+虚部），算幅度；
 *   5) 对 64 个子载波求幅度方差，即本接口承诺的 csi_get_variance()；
 *   6) 把方差喂给 fall_detector 作为"跌倒后环境扰动消失"的辅助判据
 *      （对应契约 §2.2 第三段：CSI 扰动方差 < sigma_csi）。
 *
 * 注意：CSI 回调运行在 Wi-Fi 驱动上下文，只能做轻量计算，
 *       方差统计建议只累加，取用在本接口的 getter 里做除法。
 */

#ifndef CSI_STUB_H
#define CSI_STUB_H

#include <stdint.h>
#include <stdbool.h>

/* 每收到一个姿态 UDP 包调用一次。
   真实实现：在此触发一次 CSI 快照读取 / 更新统计量。 */
void csi_feed_packet(void);

/* 获取当前 CSI 特征（64 子载波幅度方差）。
   真实实现：返回最近一次统计结果。
   占位实现：返回常数 0（表示"无 CSI 数据"）。 */
uint32_t csi_get_variance(void);

/* CSI 子系统是否已就绪（真实实现：回调注册成功且已收到过数据）。
   占位实现：恒 false，调用方据此跳过 CSI 相关判据。 */
bool csi_ready(void);

#endif /* CSI_STUB_H */
