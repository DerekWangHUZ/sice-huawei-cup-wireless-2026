/*
 * ui_proto.c -- FPGA -> S3 上行 UI 帧解析实现（纯 C99）
 * 协议见 ui_proto.h；与 rtl/cmd_decoder.v 逐字段对应。
 */

#include "ui_proto.h"

static uint64_t s_shift;

/* 新字节移入后的 8 字节窗口 */
static uint64_t ui_win(uint8_t byte)
{
    return (uint64_t)(((uint64_t)s_shift << 8) | byte);
}

void ui_proto_reset(void)
{
    s_shift = 0;
}

ui_proto_result_t ui_proto_feed(uint8_t byte, ui_frame_t *out)
{
    uint64_t w = ui_win(byte);
    s_shift = w;

    if ((uint8_t)(w >> 56) != 0xAAu || (uint8_t)((w >> 48) & 0xFFu) != 0x55u) {
        return UI_PROTO_WAIT;
    }

    {
        uint8_t cmd   = (uint8_t)((w >> 40) & 0xFFu);
        uint8_t p1    = (uint8_t)((w >> 32) & 0xFFu);
        uint8_t p2lo  = (uint8_t)((w >> 24) & 0xFFu);
        uint8_t p2hi  = (uint8_t)((w >> 16) & 0xFFu);
        uint8_t xors  = (uint8_t)((w >> 8) & 0xFFu);
        uint8_t tail  = (uint8_t)(w & 0xFFu);

        if (xors != (uint8_t)(cmd ^ p1 ^ p2lo ^ p2hi) || tail != 0x0Du) {
            return UI_PROTO_BAD;
        }
        if (cmd != UI_CMD_SNAP && cmd != UI_CMD_KEY) {
            return UI_PROTO_WAIT;   /* 未知命令：静默丢弃 */
        }

        if (out != 0) {
            out->cmd = cmd;
            out->p1  = p1;
            out->p2  = (uint16_t)((uint16_t)p2hi << 8 | p2lo);
        }
        return UI_PROTO_OK;
    }
}
