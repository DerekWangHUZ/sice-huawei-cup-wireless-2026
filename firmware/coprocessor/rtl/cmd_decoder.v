`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : cmd_decoder
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : S3 -> FPGA 短控制帧解码器。
//            协议见 docs/团队协作规范与接口契约.md §4。
//
// 帧格式（8 字节定长，小端）:
//   Byte0 = 0xAA 帧头1
//   Byte1 = 0x55 帧头2
//   Byte2 = cmd       命令码
//   Byte3 = p1        参数1（SET_STATE 时为状态码 0..3）
//   Byte4 = p2[7:0]   参数2 低字节
//   Byte5 = p2[15:8]  参数2 高字节
//   Byte6 = xor = cmd ^ p1 ^ p2_lo ^ p2_hi   （Byte2..Byte5 四字节异或）
//   Byte7 = 0x0D 帧尾
//
// 命令语义:
//   0x01 ALARM     -> alarm_on  = 1
//   0x02 CLEAR     -> alarm_on  = 0
//   0x10 SET_STATE -> state_code = p1[1:0]
//   其它命令码     -> 静默丢弃（无 cmd_valid、无 cmd_error、状态不变）
//
// 接口说明 :
//   byte_valid / byte_data   来自 uart_rx 的字节流（每个字节一拍脉冲）
//   alarm_on    持续电平：告警中（驱动蜂鸣器 / 数码管 / LED）
//   state_code  S3 判定状态编号 0..3，锁存直到下次 SET_STATE
//   cmd_valid   单周期脉冲：识别出一个合法且已知的命令帧
//   cmd_error   单周期脉冲：帧头命中但校验/帧尾失败
//
// ==================================================================
// 【滑动重同步的实现思路】
//   采用 8 字节移位寄存器：每来一个字节，窗口左移、新字节进入尾部。
//   判定用的是"新字节移入后"的窗口 shift_now（组合逻辑拼出），
//   因此命中判定与寄存器更新在同一拍完成，单 always 块无多驱动。
//
//   每个字节都对整窗做一次完整校验（帧头 + XOR + 帧尾 + 命令白名单），
//   命中即执行。这个结构天然满足协议 §4.4 的容错要求：
//     1) 任意长度垃圾字节流后紧跟合法帧 —— 合法帧收满时窗口正好对齐，
//        一定会被检出（垃圾只是从窗口左端滑出，不影响）。
//     2) 帧头错位 / 中途插入新帧头 —— 无需特殊处理，窗口滑动会自然
//        在正确对齐的那一拍命中。
//     3) 校验失败 —— 仅报一次 cmd_error，不复位窗口，后续合法帧照常解出。
//
//   残余风险：垃圾字节偶然凑出合法帧（约 2^-16 量级），协议层已有
//   XOR + 固定帧尾 + 命令白名单三重过滤，可接受。
// ==================================================================
//////////////////////////////////////////////////////////////////////////////////

module cmd_decoder (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       byte_valid,
    input  wire [7:0] byte_data,
    output reg        alarm_on,
    output reg  [1:0] state_code,
    output reg        cmd_valid,
    output reg        cmd_error
);

    //------------------------------------------------------------------
    // 协议常量（与 docs/团队协作规范与接口契约.md §4 严格一致）
    //------------------------------------------------------------------
    localparam [7:0] HDR0           = 8'hAA;
    localparam [7:0] HDR1           = 8'h55;
    localparam [7:0] TAIL           = 8'h0D;
    localparam [7:0] CMD_ALARM      = 8'h01;
    localparam [7:0] CMD_CLEAR      = 8'h02;
    localparam [7:0] CMD_SET_STATE  = 8'h10;

    //------------------------------------------------------------------
    // 8 字节滑动窗口（历史字节）
    //   shift[63:56] 为最早字节，shift[7:0] 为最近字节。
    //   新字节移入前的历史；本帧判定用下方拼好的 shift_now。
    //------------------------------------------------------------------
    reg [63:0] shift;

    // 新字节移入后的"最近 8 字节"窗口（组合逻辑）
    wire [63:0] shift_now = {shift[55:0], byte_data};

    wire [7:0] n0 = shift_now[63:56];   // 应为 0xAA
    wire [7:0] n1 = shift_now[55:48];   // 应为 0x55
    wire [7:0] n2 = shift_now[47:40];   // cmd
    wire [7:0] n3 = shift_now[39:32];   // p1
    wire [7:0] n4 = shift_now[31:24];   // p2 低字节
    wire [7:0] n5 = shift_now[23:16];   // p2 高字节
    wire [7:0] n6 = shift_now[15:8];    // xor
    wire [7:0] n7 = shift_now[7:0];     // 应为 0x0D

    // 帧头命中
    wire hdr_now = (n0 == HDR0) && (n1 == HDR1);

    // XOR 校验：Byte2..Byte5
    wire [7:0] xor_now = n2 ^ n3 ^ n4 ^ n5;

    // 整帧合法：帧头 + 校验和 + 帧尾
    wire frame_now = hdr_now && (n6 == xor_now) && (n7 == TAIL);

    // 命令白名单
    wire known_now = (n2 == CMD_ALARM) ||
                      (n2 == CMD_CLEAR) ||
                      (n2 == CMD_SET_STATE);

    //------------------------------------------------------------------
    // 单一 always 块：窗口移位 + 命令执行（无多驱动风险）
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift      <= 64'h0;
            alarm_on   <= 1'b0;
            state_code <= 2'd0;
            cmd_valid  <= 1'b0;
            cmd_error  <= 1'b0;
        end else begin
            // 脉冲信号默认拉低，仅命中当拍为高
            cmd_valid <= 1'b0;
            cmd_error <= 1'b0;

            if (byte_valid) begin
                // 新字节从低位进入窗口，最早字节从高位滑出
                shift <= {shift[55:0], byte_data};

                if (frame_now && known_now) begin
                    cmd_valid <= 1'b1;
                    case (n2)
                        CMD_ALARM:     alarm_on   <= 1'b1;
                        CMD_CLEAR:     alarm_on   <= 1'b0;
                        CMD_SET_STATE: state_code <= n3[1:0];
                        default: ;
                    endcase
                end else if (hdr_now && !frame_now) begin
                    // 帧头对上了但校验/帧尾错：报错一次，窗口继续滑动
                    cmd_error <= 1'b1;
                end
            end
        end
    end

endmodule
