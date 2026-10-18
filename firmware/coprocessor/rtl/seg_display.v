`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : seg_display
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 6 位七段数码管动态扫描显示。
//            布局 = 2 位状态助记符 + 4 位实时合加速度读数（单位 g，两位小数）。
//
// 接口说明 :
//   clk         50MHz 系统时钟
//   rst_n       低有效异步复位
//   state_code  S3 判定状态编号（来自 cmd_decoder）：
//               0 = IDLE 静止，1 = WALK 走动，2 = PENDING 待确认（冲击已发生），
//               3 = CONFIRMED 跌倒确认
//   alarm_on    告警标志（自检期间由 top 强制为 1）
//   mag_x100    合加速度幅值，单位 0.01g（来自 cmd_decoder 的 SET_STATE p2）
//   mag_valid   遥测是否有效；0 时读数位显示 "----"（p2 == 0xFFFF 哨兵）
//   mag_update  单周期脉冲：收到一帧有效遥测，复位老化计时器
//   seg[7:0]    段选输出。约定 seg[6:0] = {g,f,e,d,c,b,a}，seg[7] = dp。
//               高电平点亮（共阴极）。
//   an[5:0]     位选输出。本板为低电平选中（AN_ACTIVE_LOW=1）。
//
// 段码推导（自行推导，seg[6:0] = g f e d c b a，1=亮）：
//   0 = a,b,c,d,e,f        -> 0111111 = 0x3F
//   1 = b,c                -> 0000110 = 0x06
//   2 = a,b,d,e,g          -> 1011011 = 0x5B
//   3 = a,b,c,d,g          -> 1001111 = 0x4F
//   4 = b,c,f,g            -> 1100110 = 0x66
//   5 = a,c,d,f,g          -> 1101101 = 0x6D
//   6 = a,c,d,e,f,g        -> 1111101 = 0x7D
//   7 = a,b,c              -> 0000111 = 0x07
//   8 = a,b,c,d,e,f,g      -> 1111111 = 0x7F
//   9 = a,b,c,d,f,g        -> 1101111 = 0x6F
//   S = a,c,d,f,g          -> 1101101 = 0x6D（七段上 S 与 5 同形，靠位置区分）
//   t = d,e,f,g            -> 1111000 = 0x78
//   o = c,d,e,g            -> 1011100 = 0x5C
//   G = a,c,d,e,f          -> 0111101 = 0x3D（无 g 段，避免与 6 混淆）
//   P = a,b,e,f,g          -> 1110011 = 0x73
//   d = b,c,d,e,g          -> 1011110 = 0x5E
//   A = a,b,c,e,f,g        -> 1110111 = 0x77
//   L = d,e,f              -> 0111000 = 0x38
//   - = g                  -> 1000000 = 0x40
//   消隐                   -> 0000000 = 0x00
//
// 显示布局（自左向右 6 位，an[5] 为最左位）：
//   an[5] an[4] : 状态助记符 2 位
//                 state_code=0 -> "St"（静止）  1 -> "Go"（走动）
//                             2 -> "Pd"（待确认） 3 -> "AL"（告警）
//   an[3] an[2] an[1] an[0] : 合加速度读数 "XX.XX"（单位 g）
//                 an[2] 的小数点固定点亮作小数点；读数无效（未收到过遥测、
//                 收到 0xFFFF 哨兵、或超过 3s 未刷新）时四位全显 "-"
//
// 效果分档：
//   告警     : 整屏以 BLINK_HALF_DIV 决定的频率急闪（约 3.3Hz）
//   状态 2   : 助记符 2 位以 PEND_BLINK_HALF_DIV 决定的频率快闪（约 4Hz），
//              读数位保持稳定刷新 —— "冲击已发生、正在确认"
//   状态 0/1 : 稳定显示；读数本身随 S3 每秒刷新而变化，即"系统在跑"的存活指示
//
// 参数说明 :
//   SCAN_DIV            每位显示时长（clk 数）。默认 5000 -> 100us/位，
//                       6 位一轮 600us 约 1.67kHz 刷新，人眼无闪烁。
//   BLINK_HALF_DIV      告警急闪半周期（clk 数）。默认 7_500_000 -> 150ms
//                       半周期（约 3.3Hz 完整周期）。
//   PEND_BLINK_HALF_DIV 待确认快闪半周期。默认 6_250_000 -> 125ms（4Hz）。
//   TELEM_STALE_DIV     遥测老化阈值。默认 150_000_000 -> 3s（与 S3 侧
//                       DATA_TIMEOUT_MS 同量级）。超时后读数位显示 "----"。
//   AN_ACTIVE_LOW       位选极性。1=低电平选中（本板），0=高电平选中。
//   SEG_INVERT          段码极性。0=高电平点亮（本板共阴极），1=取反（共阳极）。
//   仿真时由 tb 用 defparam 覆盖分频参数以缩短仿真时间。
//////////////////////////////////////////////////////////////////////////////////

module seg_display #(
    parameter SCAN_DIV            = 5000,        // 100us / 位 @50MHz
    parameter BLINK_HALF_DIV      = 7500000,     // 150ms 半周期 -> 约 3.3Hz 急闪
    parameter PEND_BLINK_HALF_DIV = 6250000,     // 125ms 半周期 -> 4Hz 待确认快闪
    parameter TELEM_STALE_DIV     = 150000000,   // 3s 无有效遥测 -> 读数位显示 "----"
    parameter AN_ACTIVE_LOW       = 1,           // 位选低有效
    parameter SEG_INVERT          = 0            // 段码不取反（共阴极）
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  state_code,
    input  wire        alarm_on,
    input  wire [15:0] mag_x100,
    input  wire        mag_valid,
    input  wire        mag_update,
    output reg  [7:0]  seg,
    output reg  [5:0]  an
);

    //------------------------------------------------------------------
    // 段码表（seg[6:0]，dp 单独由 dp_raw 控制）
    //------------------------------------------------------------------
    localparam [6:0] SEG_0 = 7'h3F;
    localparam [6:0] SEG_1 = 7'h06;
    localparam [6:0] SEG_2 = 7'h5B;
    localparam [6:0] SEG_3 = 7'h4F;
    localparam [6:0] SEG_4 = 7'h66;
    localparam [6:0] SEG_5 = 7'h6D;
    localparam [6:0] SEG_6 = 7'h7D;
    localparam [6:0] SEG_7 = 7'h07;
    localparam [6:0] SEG_8 = 7'h7F;
    localparam [6:0] SEG_9 = 7'h6F;
    localparam [6:0] SEG_S = 7'h6D;
    localparam [6:0] SEG_t = 7'h78;
    localparam [6:0] SEG_o = 7'h5C;
    localparam [6:0] SEG_G = 7'h3D;
    localparam [6:0] SEG_P = 7'h73;
    localparam [6:0] SEG_d = 7'h5E;
    localparam [6:0] SEG_A = 7'h77;
    localparam [6:0] SEG_L = 7'h38;
    localparam [6:0] SEG_DASH  = 7'h40;
    localparam [6:0] SEG_BLANK = 7'h00;

    //------------------------------------------------------------------
    // 扫描计数器与位选索引
    //------------------------------------------------------------------
    reg [31:0] scan_cnt;
    reg [2:0]  digit_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt  <= 32'd0;
            digit_idx <= 3'd0;
        end else begin
            if (scan_cnt == SCAN_DIV - 1) begin
                scan_cnt  <= 32'd0;
                digit_idx <= (digit_idx == 3'd5) ? 3'd0 : digit_idx + 3'd1;
            end else begin
                scan_cnt <= scan_cnt + 32'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // 闪烁相位：告警整屏急闪；待确认助记符快闪
    //------------------------------------------------------------------
    reg [31:0] blink_cnt;
    reg        blink_on;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blink_cnt <= 32'd0;
            blink_on  <= 1'b1;
        end else begin
            if (blink_cnt == BLINK_HALF_DIV - 1) begin
                blink_cnt <= 32'd0;
                blink_on  <= ~blink_on;
            end else begin
                blink_cnt <= blink_cnt + 32'd1;
            end
        end
    end

    reg [31:0] pend_cnt;
    reg        pend_blink_on;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pend_cnt      <= 32'd0;
            pend_blink_on <= 1'b1;
        end else if (pend_cnt == PEND_BLINK_HALF_DIV - 1) begin
            pend_cnt      <= 32'd0;
            pend_blink_on <= ~pend_blink_on;
        end else begin
            pend_cnt <= pend_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // 遥测老化看门狗：最后一次"有效遥测"起算，超过 TELEM_STALE_DIV 未刷新
    // 就把读数位切成 "----"。
    //
    // 为什么放在 FPGA 侧：数据流停摆时 S3 的收包循环会阻塞在 recvfrom 上
    // （该超时在本 IDF 构建下实测不生效），靠 S3 主动发哨兵并不可靠；
    // FPGA 自己有 50MHz 时钟，老化判定永远是准的，且不依赖任何 S3 行为。
    // 正常情况下 S3 以 1Hz 刷新遥测，计时器每秒被清零，永不老化。
    //
    // 告警期间（alarm_on=1）**冻结计时器**：契约 §4.5 第 2 条要求确认跌倒后
    // 不再发 SET_STATE，若仍按 3s 老化，告警画面上的读数会在 3s 后变成
    // "----"——那不是我们想要的效果。告警时显示的应始终是"冲击那一刻的
    // 幅值"，因此这里让老化停摆，读数定格直到告警解除。
    //------------------------------------------------------------------
    reg [27:0] stale_cnt;
    reg        mag_fresh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stale_cnt <= 28'd0;
            mag_fresh <= 1'b0;
        end else if (mag_update) begin
            stale_cnt <= 28'd0;
            mag_fresh <= 1'b1;
        end else if (alarm_on) begin
            // 告警期间冻结：保持当前新鲜度，读数定格
            stale_cnt <= stale_cnt;
        end else if (stale_cnt < TELEM_STALE_DIV[27:0]) begin
            stale_cnt <= stale_cnt + 28'd1;
        end else begin
            mag_fresh <= 1'b0;
        end
    end

    // 真正驱动显示的"读数有效"：收到过有效遥测，且没超时
    wire rd_valid = mag_valid & mag_fresh;

    //------------------------------------------------------------------
    // 遥测读数拆分："XX.XX"（单位 g，mag_x100 单位 0.01g）
    //   mag_x100 已由发送方饱和到 9999，此处无需再处理溢出。
    //   常数除法会被综合为常数乘法，不产生除法器。
    //------------------------------------------------------------------
    wire [3:0] n_tens  = mag_x100 / 16'd1000;              // 十位（g）
    wire [3:0] n_ones  = (mag_x100 / 16'd100) % 16'd10;    // 个位（g）
    wire [3:0] n_ten   = (mag_x100 / 16'd10) % 16'd10;     // 十分位
    wire [3:0] n_hun   = mag_x100 % 16'd10;                // 百分位

    // 十进制数字 -> 段码
    function [6:0] dec_seg;
        input [3:0] d;
        begin
            case (d)
                4'd0: dec_seg = SEG_0;
                4'd1: dec_seg = SEG_1;
                4'd2: dec_seg = SEG_2;
                4'd3: dec_seg = SEG_3;
                4'd4: dec_seg = SEG_4;
                4'd5: dec_seg = SEG_5;
                4'd6: dec_seg = SEG_6;
                4'd7: dec_seg = SEG_7;
                4'd8: dec_seg = SEG_8;
                4'd9: dec_seg = SEG_9;
                default: dec_seg = SEG_BLANK;
            endcase
        end
    endfunction

    // 状态码 -> 助记符两字符
    reg [6:0] mn1;      // an[5]
    reg [6:0] mn2;      // an[4]

    always @(*) begin
        if (alarm_on) begin
            mn1 = SEG_A;                // "AL"
            mn2 = SEG_L;
        end else begin
            case (state_code)
                2'd0:    begin mn1 = SEG_S; mn2 = SEG_t; end   // "St" 静止
                2'd1:    begin mn1 = SEG_G; mn2 = SEG_o; end   // "Go" 走动
                2'd2:    begin mn1 = SEG_P; mn2 = SEG_d; end   // "Pd" 待确认
                2'd3:    begin mn1 = SEG_A; mn2 = SEG_L; end   // "AL" 告警
                default: begin mn1 = SEG_BLANK; mn2 = SEG_BLANK; end
            endcase
        end
    end

    //------------------------------------------------------------------
    // 每位要显示的段码与小数点（未取反）
    //------------------------------------------------------------------
    reg [6:0] seg_raw;
    reg       dp_raw;
    reg       dig_blank;    // 该位整体消隐（闪烁的灭相位）

    always @(*) begin
        seg_raw   = SEG_BLANK;
        dp_raw    = 1'b0;
        dig_blank = 1'b0;

        case (digit_idx)
            3'd5: seg_raw = mn1;
            3'd4: seg_raw = mn2;
            3'd3: seg_raw = rd_valid ? dec_seg(n_tens) : SEG_DASH;
            3'd2: begin
                seg_raw = rd_valid ? dec_seg(n_ones) : SEG_DASH;
                dp_raw  = rd_valid;                // 小数点固定在此位
            end
            3'd1: seg_raw = rd_valid ? dec_seg(n_ten) : SEG_DASH;
            3'd0: seg_raw = rd_valid ? dec_seg(n_hun) : SEG_DASH;
            default: seg_raw = SEG_BLANK;
        endcase

        if (alarm_on) begin
            // 告警：整屏急闪
            if (!blink_on) dig_blank = 1'b1;
        end else if (state_code == 2'd2) begin
            // 待确认：仅助记符 2 位快闪，读数保持稳定
            if (!pend_blink_on && (digit_idx == 3'd5 || digit_idx == 3'd4)) begin
                dig_blank = 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // 输出：段码 + 小数点（可选取反）、位选（可选极性）
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg <= 8'h00;
            an  <= (AN_ACTIVE_LOW == 1) ? 6'b111111 : 6'b000000;
        end else begin
            if (dig_blank) begin
                seg <= (SEG_INVERT == 1) ? 8'hFF : 8'h00;
            end else begin
                seg <= (SEG_INVERT == 1) ? {~dp_raw, ~seg_raw}
                                         : {dp_raw, seg_raw};
            end

            // 位选：一位选中，其余关闭
            case (digit_idx)
                3'd0: an <= (AN_ACTIVE_LOW == 1) ? 6'b111110 : 6'b000001;
                3'd1: an <= (AN_ACTIVE_LOW == 1) ? 6'b111101 : 6'b000010;
                3'd2: an <= (AN_ACTIVE_LOW == 1) ? 6'b111011 : 6'b000100;
                3'd3: an <= (AN_ACTIVE_LOW == 1) ? 6'b110111 : 6'b001000;
                3'd4: an <= (AN_ACTIVE_LOW == 1) ? 6'b101111 : 6'b010000;
                3'd5: an <= (AN_ACTIVE_LOW == 1) ? 6'b011111 : 6'b100000;
                default: an <= (AN_ACTIVE_LOW == 1) ? 6'b111111 : 6'b000000;
            endcase
        end
    end

endmodule
