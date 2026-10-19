/*
 * ui_proto.h -- FPGA -> S3 上行 UI 帧解析（纯 C99，零平台依赖）
 *
 * 上行链路与下行链路（fpga_link / 契约 §4）使用同一种 8 字节定长帧：
 *   AA 55 | cmd | p1 | p2_lo | p2_hi | xor | 0D
 *   xor = cmd ^ p1 ^ p2_lo ^ p2_hi
 *
 * 本模块与 rtl/cmd_decoder.v 逐字段对应（同一协议的两个方向）：
 *   0x80 输入快照 : p1=0x01，p2[0]=演示模式 p2[1]=本地静音 p2[4:2]=演示场景
 *                   触发：任一位变化 + 周期心跳（兼作上行链路存活探测）
 *   0x81 键事件   : p1=0x01，p2[3:0]=键号 0..15（消抖后的按下沿）
 *
 * 设计纪律与 fall_core 相同：纯 C99、零平台头文件，宿主机离线自测
 * 直接 #include 本源文件编译断言，杜绝"文档/RTL/固件三份逻辑漂移"。
 */

#ifndef UI_PROTO_H
#define UI_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UI_PROTO_FRAME_LEN 8

#define UI_CMD_SNAP 0x80u
#define UI_CMD_KEY  0x81u

/* 键号 = 板上丝印 KEY0~KEY15（矩阵行主序，与 rtl/input_ctrl.v 的
 * key_number() 一致；若丝印顺序不同，改 FPGA 侧一处即可，本头文件不变）。 */
#define UI_KEY_0  0u
#define UI_KEY_1  1u
#define UI_KEY_2  2u
#define UI_KEY_3  3u
#define UI_KEY_4  4u
#define UI_KEY_5  5u
#define UI_KEY_6  6u
#define UI_KEY_7  7u
#define UI_KEY_8  8u
#define UI_KEY_9  9u
#define UI_KEY_10 10u
#define UI_KEY_11 11u
#define UI_KEY_12 12u
#define UI_KEY_13 13u
#define UI_KEY_14 14u
#define UI_KEY_15 15u

typedef struct {
    uint8_t  cmd;
    uint8_t  p1;
    uint16_t p2;
} ui_frame_t;

typedef enum {
    UI_PROTO_WAIT = 0,   /* 收集中，本次未产出帧 */
    UI_PROTO_OK   = 1,   /* 本字节收完，成功产出一帧，out 已填充 */
    UI_PROTO_BAD  = 2    /* 帧头命中但校验/帧尾失败（已按滑动窗口继续） */
} ui_proto_result_t;

/* 快照位域提取 */
#define UI_SNAP_DEMO(p2)   ((uint8_t)((p2) & 0x01u))
#define UI_SNAP_MUTE(p2)   ((uint8_t)(((p2) >> 1) & 0x01u))
#define UI_SNAP_SCENE(p2)  ((uint8_t)(((p2) >> 2) & 0x07u))
#define UI_KEY_CODE(p2)    ((uint8_t)((p2) & 0x0Fu))

/* 复位解析器状态（流/超时时调用） */
void ui_proto_reset(void);

/*
 * 逐字节喂入。返回 UI_PROTO_OK 时 *out 有效。
 * 滑动窗口语义与 cmd_decoder 一致：任意垃圾流后紧跟合法帧必然被解出；
 * 校验失败报 UI_PROTO_BAD 但不复位窗口。
 */
ui_proto_result_t ui_proto_feed(uint8_t byte, ui_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UI_PROTO_H */
