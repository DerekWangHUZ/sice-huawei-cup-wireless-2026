/*
 * ui_app.h -- OLED 多页面 UI 与演示剧本的任务层
 *
 * 页面（优先级从高到低，250ms 解析一次）：
 *   真实告警强制 P0 告警屏 > 演示模式 P4 > 设置模式 P3 >
 *   键盘锁定页 > 自动轮播(可选) > 默认 P0 状态页
 *
 * 输入：FPGA 键盘/拨码经 fpga_uplink 投入事件队列；正常模式下键盘负责
 * 翻页与设置导航，演示模式下场景由 FPGA 本地接管、S3 只同步显示。
 *
 * 演示剧本（场景 0）：S3 按剧本下发与真实事件逐字节相同的控制帧
 * （静止->行走->待确认->告警->解除），真实判定继续后台运行；真实告警
 * 期间剧本暂停且真实 ALARM/CLEAR 永远直通 FPGA（安全优先）。
 */

#ifndef UI_APP_H
#define UI_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "oled_display.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 网络参数（base_station.c 的 SoftAP 配置同一来源，供系统页显示） */
extern const char *const k_ap_ssid;
extern const int        k_ap_channel;

/* 演示剧本占用状态显示：true 时抑制真实 SET_STATE/遥测下发
 * （ALARM/CLEAR 永不抑制，保证真实告警直通）。base_station.c 读取。 */
extern volatile bool g_ui_demo_owns_state;

/* ---- 由 base_station.c 提供的访问器 ---- */

/* 判定状态快照（互斥保护拷贝；含 mag/impact 字段） */
void base_station_get_status(oled_status_t *out);

/* FPGA 下行链路是否就绪（app_main 初始化后不变） */
bool base_station_fpga_ok(void);

/* 是否处于真实告警锁存（演示剧本据此挂起） */
bool base_station_alarm_active(void);

/* 真实判定状态（演示剧本退出时向 FPGA 重同步用） */
fall_state_t base_station_real_state(void);

/* 人工解除告警请求：UI 任务置位，udp_pose_task 在下一轮循环执行并清除。
 * 单一写入者纪律：只有 udp_pose_task 触碰 fall_detector，跨任务只传这一位。 */
extern volatile bool g_manual_clear_req;

/* 创建事件队列并启动 uplink/ui/demo 三个任务（OLED 就绪后调用一次）。
 * OLED 未就绪时直接返回（FPGA 本地演示不受影响）。 */
void ui_app_start(void);

/* 当前演示剧本步（0..4），-1 = 剧本未运行；演示页渲染用 */
int ui_demo_script_step(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_APP_H */
