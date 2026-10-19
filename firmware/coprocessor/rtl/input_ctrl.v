`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : input_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 人机输入子系统 + FPGA->S3 上行链路。
//
//   1) 4x4 矩阵键盘扫描：行输出逐行驱动（低有效），列输入+板上拉。
//      约 1kHz 全矩阵采样，每键独立消抖（默认 20ms），按下沿产生键事件。
//      **键编号 = 板上丝印 KEY0~KEY15**（矩阵位置 pos = 行*4+列，行主序）。
//      若丝印编号与行主序不符，只改 key_number() 一处置换即可
//      （上行帧与本地演示场景选择共用同一编号）。
//   2) 拨码开关 sw_demo（SW1）/ sw_mute（SW2）：消抖后作为持久电平输出。
//   3) 演示模式本地控制（不依赖 S3 在场）：
//      KEY0~KEY7 直选场景 0~7；KEY8 场景+1 循环；
//      KEY9/KEY10 场景内调参（合成读数 ±0.01g；场景 6 音调相换档）。
//   4) 上行 UART 帧（8 字节定长，与下行帧格式对称，115200 8N1）：
//      0x80 输入快照 : AA 55 80 01 p2lo p2hi xor 0D
//                      p2[0]=demo_en p2[1]=mute_local p2[4:2]=demo_scene
//                      触发：任一位变化 + 周期心跳（S3 兼作链路存活探测）
//      0x81 键事件   : AA 55 81 01 00 <键号0..15> xor 0D
//                      触发：任一按键消抖后的按下沿（深度 8 的键队列）
//
// 接口说明 :
//   clk/rst_n     50MHz / 异步低有效复位
//   key_row       行扫描输出（低电平选中行）
//   key_col       列输入（板上拉，按下=被选中行拉低该列）
//   sw_demo/mute  拨码原始电平（T8/T7）
//   demo_en       消抖后的演示模式使能
//   mute_local    消抖后的本地静音（与 0x03 命令静音在 top 相"或"）
//   demo_scene    当前演示场景 0..7
//   demo_mag      演示合成读数（0.01g，'C'/'D' 调节，默认 340=3.40g）
//   walk_idx      场景 5 走灯序号（0..WALK_LEDS-1，SCENE_SLOT 步进）
//   self_tone_phase 场景 6 阶段：0=段码巡检 1=音调试听（self_done 上升沿切换）
//   self_tone     音调档 0..2（场景 6 音调相，'C'/'D' 调节）
//   self_done     来自 seg_display：段码巡检一轮完成（单拍）
//   uplink_txd    上行串行输出
////////////////////////////////////////////////////////////////////////////////

module input_ctrl #(
    parameter SAMPLE_DIV   = 2000,        // 行相位 clk 数（2000=40us，全矩阵 160us）
    parameter DEBOUNCE_CNT = 125,         // 消抖采样数（125 x 160us = 20ms）
    parameter SNAP_PERIOD  = 50_000_000,  // 快照心跳 1s
    parameter SCENE_SLOT   = 25_000_000,  // 演示场景步进 0.5s
    parameter WALK_LEDS    = 12,          // 走灯 LED 数（与 led_ctrl.LED_COUNT 一致）
    parameter TONE_ON_DIV  = 10_000_000,  // 音调试听：响 200ms
    parameter TONE_OFF_DIV = 20_000_000   // 音调试听：停 400ms
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg  [3:0]  key_row,
    input  wire [3:0]  key_col,
    input  wire        sw_demo,
    input  wire        sw_mute,
    output wire        demo_en,
    output wire        mute_local,
    output reg  [2:0]  demo_scene,
    output reg  [15:0] demo_mag,
    output reg  [3:0]  walk_idx,
    output reg         self_tone_phase,
    output reg  [1:0]  self_tone,
    input  wire        self_done,
    output wire        uplink_txd
);

    //------------------------------------------------------------------
    // 列输入两级同步（异步引脚防亚稳态）
    //------------------------------------------------------------------
    reg [3:0] col_sync0, col_sync1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_sync0 <= 4'hF;
            col_sync1 <= 4'hF;
        end else begin
            col_sync0 <= key_col;
            col_sync1 <= col_sync0;
        end
    end

    //------------------------------------------------------------------
    // 行扫描：SAMPLE_DIV/行，相位末采样，四行采完 matrix_tick
    //------------------------------------------------------------------
    reg [1:0]  row_idx;
    reg [31:0] phase_cnt;
    reg [3:0]  rows [0:3];                // rows[r] = 该行 4 键当前原始采样
    reg        matrix_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_idx     <= 2'd0;
            phase_cnt   <= 32'd0;
            rows[0]     <= 4'h0;
            rows[1]     <= 4'h0;
            rows[2]     <= 4'h0;
            rows[3]     <= 4'h0;
            matrix_tick <= 1'b0;
        end else begin
            matrix_tick <= 1'b0;
            if (phase_cnt == SAMPLE_DIV - 1) begin
                phase_cnt <= 32'd0;
                rows[row_idx] <= ~col_sync1;  // 按下=列被拉低
                if (row_idx == 2'd3) begin
                    row_idx     <= 2'd0;
                    matrix_tick <= 1'b1;
                end else begin
                    row_idx <= row_idx + 2'd1;
                end
            end else begin
                phase_cnt <= phase_cnt + 32'd1;
            end
        end
    end

    // 行低有效 one-hot 驱动
    always @(*) begin
        key_row = ~(4'b0001 << row_idx);
    end

    // 位序：pos = row*4 + col，与丝印布局表的行主序一致
    wire [15:0] matrix_sample = {rows[3], rows[2], rows[1], rows[0]};

    //------------------------------------------------------------------
    // 16 键独立消抖（每键一个候选寄存器 + 计数器）
    //------------------------------------------------------------------
    reg [15:0] db_stable;
    reg [15:0] db_prev;

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : g_db
            reg       cand;
            reg [7:0] cnt;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cand <= 1'b0;
                    cnt  <= 8'd0;
                end else if (matrix_tick) begin
                    if (matrix_sample[g] != cand) begin
                        cand <= matrix_sample[g];
                        cnt  <= 8'd0;
                    end else begin
                        if (cnt < DEBOUNCE_CNT[7:0]) begin
                            cnt <= cnt + 8'd1;
                        end
                        // 第 DEBOUNCE_CNT 个连续一致采样确认（此刻采样仍等于候选）
                        if (cnt == DEBOUNCE_CNT - 1) begin
                            db_stable[g] <= cand;
                        end
                    end
                end
            end
        end
    endgenerate

    // 按下沿检测（低 -> 高）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) db_prev <= 16'h0;
        else        db_prev <= db_stable;
    end
    wire [15:0] press_edge = db_stable & ~db_prev;

    // 优先编码：多位同时按下取最低位（单拍脉冲）
    reg  [3:0] key_code_r;
    reg        key_evt_r;
    integer    k;
    always @(*) begin
        key_evt_r  = 1'b0;
        key_code_r = 4'd0;
        for (k = 15; k >= 0; k = k - 1) begin
            if (press_edge[k]) begin
                key_evt_r  = 1'b1;
                key_code_r = k[3:0];
            end
        end
    end

    reg [3:0] key_code;
    reg       key_evt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_evt  <= 1'b0;
            key_code <= 4'd0;
        end else begin
            key_evt  <= key_evt_r;
            key_code <= key_code_r;
        end
    end

    //------------------------------------------------------------------
    // 丝印映射（上板校准只改这里）
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // 键编号：板上丝印 KEY0~KEY15。
    //
    // 实测结论（按按下结果反推）：丝印编号 **等于** 矩阵扫描位置 pos
    // （行主序 pos = 行*4+列）。即扫描起点那一端实际是丝印 KEY0 那一角，
    // 因此编号直接用 pos，不做任何翻转。
    // 若日后丝印/接线再变，只改本函数；上行帧（0x81）与本地演示场景选择
    // 共用同一编号，改一处两侧同步生效。
    //------------------------------------------------------------------
    function [3:0] key_number;
        input [3:0] pos;
        begin
            key_number = pos;
        end
    endfunction

    wire [3:0] key_num = key_number(key_code);

    // 演示模式本地控制用的键位（全部按 KEY 编号）
    wire        k_low8    = (key_num <= 4'd7);    // KEY0..7  -> 场景 0..7
    wire        k_next    = (key_num == 4'd8);    // KEY8      -> 场景+1
    wire        k_dec     = (key_num == 4'd9);    // KEY9      -> 参数-
    wire        k_inc     = (key_num == 4'd10);   // KEY10     -> 参数+

    //------------------------------------------------------------------
    // 拨码消抖（与键盘同节奏采样）
    //------------------------------------------------------------------
    reg        demo_sync0, demo_sync1, mute_sync0, mute_sync1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            demo_sync0 <= 1'b0; demo_sync1 <= 1'b0;
            mute_sync0 <= 1'b0; mute_sync1 <= 1'b0;
        end else begin
            demo_sync0 <= sw_demo; demo_sync1 <= demo_sync0;
            mute_sync0 <= sw_mute; mute_sync1 <= mute_sync0;
        end
    end

    reg       demo_c, mute_c, demo_st, mute_st;
    reg [7:0] sw_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            demo_c <= 1'b0; mute_c <= 1'b0;
            demo_st <= 1'b0; mute_st <= 1'b0;
            sw_cnt <= 8'd0;
        end else if (matrix_tick) begin
            if (demo_sync1 != demo_c) begin
                demo_c <= demo_sync1; sw_cnt <= 8'd0;
            end else if (mute_sync1 != mute_c) begin
                mute_c <= mute_sync1; sw_cnt <= 8'd0;
            end else begin
                if (sw_cnt < DEBOUNCE_CNT[7:0]) begin
                    sw_cnt <= sw_cnt + 8'd1;
                end
                if (sw_cnt == DEBOUNCE_CNT - 1) begin
                    demo_st <= demo_c;
                    mute_st <= mute_c;
                end
            end
        end
    end

    assign demo_en    = demo_st;
    assign mute_local = mute_st;

    //------------------------------------------------------------------
    // 演示模式本地控制（KEY0~7 选场景 / KEY8 下一个 / KEY9·10 调参）
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            demo_scene      <= 3'd0;
            demo_mag        <= 16'd340;      // 默认 3.40g
            self_tone       <= 2'd0;
            self_tone_phase <= 1'b0;
        end else begin
            if (key_evt && demo_en) begin
                if (k_low8) begin
                    demo_scene      <= key_num[2:0];
                    self_tone_phase <= 1'b0;     // 重选场景回到段码相
                end else if (k_next) begin
                    demo_scene      <= (demo_scene == 3'd7) ? 3'd0 : demo_scene + 3'd1;
                    self_tone_phase <= 1'b0;
                end else if (k_dec) begin
                    if (demo_scene == 3'd6 && self_tone_phase) begin
                        // 音调试听相：KEY9 = 降档（循环）
                        self_tone <= (self_tone == 2'd0) ? 2'd2 : self_tone - 2'd1;
                    end else if (demo_mag >= 16'd10) begin
                        demo_mag <= demo_mag - 16'd10;
                    end else begin
                        demo_mag <= 16'd0;
                    end
                end else if (k_inc) begin
                    if (demo_scene == 3'd6 && self_tone_phase) begin
                        // 音调试听相：KEY10 = 升档（循环）
                        self_tone <= (self_tone == 2'd2) ? 2'd0 : self_tone + 2'd1;
                    end else if (demo_mag <= 16'd9989) begin
                        demo_mag <= demo_mag + 16'd10;
                    end else begin
                        demo_mag <= 16'd9999;
                    end
                end
            end
            // 段码巡检一轮完成 -> 进入音调相
            if (self_done) self_tone_phase <= 1'b1;
        end
    end

    //------------------------------------------------------------------
    // 场景 5 走灯序号（0..WALK_LEDS-1，SCENE_SLOT 步进）
    //------------------------------------------------------------------
    reg [31:0] slot_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            walk_idx <= 4'd0;
            slot_cnt <= 32'd0;
        end else if (demo_en && (demo_scene == 3'd5)) begin
            if (slot_cnt == SCENE_SLOT - 1) begin
                slot_cnt <= 32'd0;
                walk_idx <= (walk_idx == WALK_LEDS - 1) ? 4'd0 : walk_idx + 4'd1;
            end else begin
                slot_cnt <= slot_cnt + 32'd1;
            end
        end else begin
            walk_idx <= 4'd0;
            slot_cnt <= 32'd0;
        end
    end

    //------------------------------------------------------------------
    // 上行链路：键事件 FIFO(深度 8，容量 7) + 快照（变化/心跳）-> uart_tx
    //   单写者纪律：push 块只写 mem/wr_ptr；发送状态机只写 rd_ptr；
    //   快照去重/心跳计数也在发送状态机内完成，避免多驱动。
    //------------------------------------------------------------------
    reg [3:0] kf_mem [0:7];
    reg [2:0] kf_wr, kf_rd;

    wire kf_empty = (kf_wr == kf_rd);
    wire kf_full  = (((kf_wr + 3'd1) & 3'd7) == kf_rd);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kf_wr <= 3'd0;
        end else if (key_evt && !kf_full) begin
            kf_mem[kf_wr] <= key_num;    // 上行报告 KEY 编号（丝印 KEY0~15）
            kf_wr         <= kf_wr + 3'd1;
        end
        // 队满丢弃（人手按键间隔远大于帧时长，仅防御）
    end

    // 快照值：bit0=demo bit1=mute [4:2]=scene
    wire [15:0] snap_val = {11'b0, demo_scene, mute_st, demo_st};

    // 帧装配与字节流发送（唯一的状态机）
    reg [7:0] frm [0:7];
    reg [2:0] fidx;
    reg       load_req;
    reg       in_frame;
    reg [7:0] tx_byte;
    reg [15:0] snap_last;
    reg [31:0] hb_cnt;

    wire tx_busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fidx      <= 3'd0;
            load_req  <= 1'b0;
            in_frame  <= 1'b0;
            tx_byte   <= 8'h00;
            kf_rd     <= 3'd0;
            snap_last <= 16'hFFFF;      // 与复位后快照不同 -> 上电即发一次
            hb_cnt    <= 32'd0;
        end else begin
            load_req <= 1'b0;

            // 心跳计时（仅用于快照触发）
            if (hb_cnt >= SNAP_PERIOD - 1) hb_cnt <= 32'd0;
            else                           hb_cnt <= hb_cnt + 32'd1;

            if (!in_frame) begin
                if (!tx_busy && !kf_empty) begin
                    // 键事件帧：AA 55 81 01 00 <code> xor 0D
                    frm[0] <= 8'hAA;
                    frm[1] <= 8'h55;
                    frm[2] <= 8'h81;
                    frm[3] <= 8'h01;
                    frm[4] <= {4'd0, kf_mem[kf_rd]};
                    frm[5] <= 8'h00;
                    frm[6] <= 8'h81 ^ 8'h01 ^ {4'd0, kf_mem[kf_rd]};
                    frm[7] <= 8'h0D;
                    kf_rd  <= kf_rd + 3'd1;
                    fidx   <= 3'd0;
                    in_frame <= 1'b1;
                end else if (!tx_busy && ((snap_val != snap_last) ||
                                          (hb_cnt >= SNAP_PERIOD - 1))) begin
                    // 快照帧：AA 55 80 01 <p2lo> <p2hi> xor 0D
                    frm[0] <= 8'hAA;
                    frm[1] <= 8'h55;
                    frm[2] <= 8'h80;
                    frm[3] <= 8'h01;
                    frm[4] <= snap_val[7:0];
                    frm[5] <= snap_val[15:8];
                    frm[6] <= 8'h80 ^ 8'h01 ^ snap_val[7:0] ^ snap_val[15:8];
                    frm[7] <= 8'h0D;
                    snap_last <= snap_val;
                    hb_cnt    <= 32'd0;
                    fidx      <= 3'd0;
                    in_frame  <= 1'b1;
                end
            end else begin
                if (!tx_busy && !load_req) begin
                    tx_byte  <= frm[fidx];
                    load_req <= 1'b1;
                    if (fidx == 3'd7) begin
                        in_frame <= 1'b0;
                        fidx     <= 3'd0;
                    end else begin
                        fidx <= fidx + 3'd1;
                    end
                end
            end
        end
    end

    uart_tx #(.CLKS_PER_BIT(434)) u_uart_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (load_req),
        .byte_data (tx_byte),
        .busy      (tx_busy),
        .txd       (uplink_txd)
    );

endmodule
