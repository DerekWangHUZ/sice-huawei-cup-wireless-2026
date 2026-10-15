`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : seg_display
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 6 位七段数码管动态扫描显示。显示 S3 判定状态，跌倒告警时
//            闪烁显示 "FALL--"。
//
// 接口说明 :
//   clk         50MHz 系统时钟
//   rst_n       低有效异步复位
//   state_code  S3 判定状态编号 0..3（来自 cmd_decoder）
//   alarm_on    告警标志（来自 cmd_decoder）
//   seg[7:0]    段选输出。约定 seg[6:0] = {g,f,e,d,c,b,a}，seg[7] = dp（本
//               模块不使用 dp，恒 0）。高电平点亮。
//   an[5:0]     位选输出。本板为低电平选中（AN_ACTIVE_LOW=1）。
//
// 段码推导（自行推导，seg[6:0] = g f e d c b a，1=亮）：
//   0 = a,b,c,d,e,f        -> 0111111 = 0x3F
//   1 = b,c                -> 0000110 = 0x06
//   2 = a,b,d,e,g          -> 1011011 = 0x5B
//   3 = a,b,c,d,g          -> 1001111 = 0x4F
//   F = a,e,f,g            -> 1110001 = 0x71
//   A = a,b,c,e,f,g        -> 1110111 = 0x77
//   L = d,e,f              -> 0111000 = 0x38
//   - = g                  -> 1000000 = 0x40
//   消隐                   -> 0000000 = 0x00
//
// 显示布局（自左向右 6 位，an[5] 为最左位）：
//   告警     : F A L L - -   （以 BLINK_HALF_DIV 决定的频率整屏闪烁）
//   非告警   : 最右位(an[0])显示状态码数字 0..3，其余 5 位消隐
//              —— 采用"右对齐单数字"布局，简单可靠，一眼能区分状态
//
// 参数说明 :
//   SCAN_DIV       每位显示时长（clk 数）。默认 5000 → 100us/位，
//                  6 位一轮 600us ≈ 1.67kHz 刷新，人眼无闪烁。
//   BLINK_HALF_DIV 闪烁半周期（clk 数）。默认 25_000_000 → 500ms 半周期
//                  即 1Hz 亮灭（2Hz 完整周期）。仿真时由 tb 覆盖为小值。
//   AN_ACTIVE_LOW  位选极性。1=低电平选中（本板），0=高电平选中。
//   SEG_INVERT     段码极性。0=高电平点亮（本板共阴极），1=取反（共阳极）。
//////////////////////////////////////////////////////////////////////////////////

module seg_display #(
    parameter SCAN_DIV       = 5000,          // 100us / 位 @50MHz
    parameter BLINK_HALF_DIV = 25000000,      // 500ms 半周期 -> 1Hz 亮灭
    parameter AN_ACTIVE_LOW  = 1,             // 位选低有效
    parameter SEG_INVERT     = 0              // 段码不取反（共阴极）
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] state_code,
    input  wire       alarm_on,
    output reg  [7:0] seg,
    output reg  [5:0] an
);

    //------------------------------------------------------------------
    // 段码表（seg[6:0]，seg[7]=dp 恒 0）
    //------------------------------------------------------------------
    localparam [6:0] SEG_0 = 7'h3F;
    localparam [6:0] SEG_1 = 7'h06;
    localparam [6:0] SEG_2 = 7'h5B;
    localparam [6:0] SEG_3 = 7'h4F;
    localparam [6:0] SEG_F = 7'h71;
    localparam [6:0] SEG_A = 7'h77;
    localparam [6:0] SEG_L = 7'h38;
    localparam [6:0] SEG_DASH = 7'h40;
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
    // 闪烁控制：blink_on 为 1 时允许点亮
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

    //------------------------------------------------------------------
    // 每位要显示的段码（7 位，未取反）
    //   告警：an[5]='F' an[4]='A' an[3]='L' an[2]='L' an[1]='-' an[0]='-'
    //   正常：仅 an[0] 显示状态码数字，其余消隐
    //------------------------------------------------------------------
    reg [6:0] seg_raw;

    always @(*) begin
        if (alarm_on) begin
            case (digit_idx)
                3'd5: seg_raw = SEG_F;
                3'd4: seg_raw = SEG_A;
                3'd3: seg_raw = SEG_L;
                3'd2: seg_raw = SEG_L;
                3'd1: seg_raw = SEG_DASH;
                3'd0: seg_raw = SEG_DASH;
                default: seg_raw = SEG_BLANK;
            endcase
        end else begin
            case (digit_idx)
                3'd0: begin   // 最右位显示状态码
                    case (state_code)
                        2'd0: seg_raw = SEG_0;
                        2'd1: seg_raw = SEG_1;
                        2'd2: seg_raw = SEG_2;
                        2'd3: seg_raw = SEG_3;
                        default: seg_raw = SEG_BLANK;
                    endcase
                end
                default: seg_raw = SEG_BLANK;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 输出：段码（可选取反）+ 位选（可选极性）+ 告警闪烁消隐
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg <= 8'h00;
            an  <= (AN_ACTIVE_LOW == 1) ? 6'b111111 : 6'b000000;
        end else begin
            // 段码输出：告警时按 blink_on 整屏闪烁（灭时送全 0）
            if (alarm_on && !blink_on) begin
                seg <= (SEG_INVERT == 1) ? 8'hFF : 8'h00;
            end else begin
                seg <= (SEG_INVERT == 1) ? {1'b1, ~seg_raw} : {1'b0, seg_raw};
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
