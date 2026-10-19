`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : seg_display
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 6 位七段数码管动态扫描显示。
//            整屏显示"完整英文常用词 + 带单位数值"——观众无需任何图例即可读懂：
//
//   告警 / POST / 演示场景4 : "FALL  "  整屏 3.3Hz 急闪   = 跌倒告警
//   无链路（未收到过命令）   : "HELLO "  整屏 0.5Hz 慢闪   = 我开机了，在等基站
//   静止（状态0，已链路）    : "rEAdy "  稳定             = 链路正常，待机
//   走动（状态1）            : "run   "  稳定             = 检测到运动
//   待确认（状态2）          : "  3.40G" 4Hz 快闪         = 冲击 3.40g，确认中
//   待确认但无遥测/老化      : "------"  4Hz 快闪         = 无数据
//   演示场景5（走灯）        : "Ld 05 "  稳定             = 正在点亮第 5 颗
//   演示场景6a（段码自检）   : 全 6 位同显自检字符，dp 交替，逐个步进
//   演示场景6b（音调试听）   : "t1    "  稳定 + 蜂鸣器按档发声
//
//   数码管承担"远距离状态信号"角色；状态语义的中文说明由 S3 的 OLED
//   各页面承载，两者各司其职。
//
// 接口说明 :
//   clk/rst_n    50MHz / 异步低有效复位
//   linked       链路曾建立（cmd_decoder.ever_linked）；0 时整屏 HELLO 慢闪
//   state_code   最终显示状态码 0..3（top 仲裁：真实或演示场景映射）
//   alarm_on     最终告警标志（UART 告警 | POST | 演示场景 4）
//   mag_x100     合加速度幅值 0.01g（真实遥测或演示合成值）
//   mag_valid    遥测有效（0xFFFF 哨兵帧清除）
//   mag_update   单拍：收到有效遥测（复位老化计时器）
//   stale_en     老化看门狗使能（演示合成值时置 0，永不老化）
//   special      2'b00 正常词形显示；01 走灯序号；10 段码自检；11 音调显示
//   walk_idx     走灯序号 0..11（input_ctrl 生成）
//   self_tone    音调档 0..2（显示 t1/t2/t3）
//   self_done    输出单拍：段码自检一轮（GLYPH_N 个字符）完成
//   seg[7:0]     段选，seg[6:0]={g,f,e,d,c,b,a}，seg[7]=dp，高电平点亮
//   an[5:0]      位选，低电平选中（AN_ACTIVE_LOW=1）
//
// 参数说明 :
//   SCAN_DIV              每位显示时长 5000 -> 100us/位（约 1.67kHz 刷新）
//   BLINK_HALF_DIV        告警急闪半周期 7_500_000 -> 150ms（3.3Hz）
//   PEND_BLINK_HALF_DIV   待确认快闪半周期 6_250_000 -> 125ms（4Hz）
//   LINK_BLINK_HALF_DIV   无链路慢闪半周期 50_000_000 -> 1s（0.5Hz）
//   TELEM_STALE_DIV       遥测老化阈值 150_000_000 -> 3s，超时待确认显示 "------"
//   SELF_SLOT_DIV         段码自检字符步进 25_000_000 -> 0.5s
//   GLYPH_N               段码自检字符数（一轮 = GLYPH_N x 0.5s）
//   AN_ACTIVE_LOW / SEG_INVERT  极性参数（本板低选中共阴）
//   仿真时由 tb 用 defparam 覆盖为小值以缩短仿真时间。
////////////////////////////////////////////////////////////////////////////////

module seg_display #(
    parameter SCAN_DIV            = 5000,
    parameter BLINK_HALF_DIV      = 7500000,
    parameter PEND_BLINK_HALF_DIV = 6250000,
    parameter LINK_BLINK_HALF_DIV = 50_000_000,
    parameter TELEM_STALE_DIV     = 150000000,
    parameter SELF_SLOT_DIV       = 25_000_000,
    parameter GLYPH_N             = 23,
    parameter AN_ACTIVE_LOW       = 1,
    parameter SEG_INVERT          = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        linked,
    input  wire [1:0]  state_code,
    input  wire        alarm_on,
    input  wire [15:0] mag_x100,
    input  wire        mag_valid,
    input  wire        mag_update,
    input  wire        stale_en,
    input  wire [1:0]  special,
    input  wire [3:0]  walk_idx,
    input  wire [1:0]  self_tone,
    output reg         self_done,
    output reg  [7:0]  seg,
    output reg  [5:0]  an
);

    //------------------------------------------------------------------
    // 段码表（seg[6:0]，dp 单独控制；自行推导，1=亮）
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
    localparam [6:0] SEG_A = 7'h77;
    localparam [6:0] SEG_L = 7'h38;
    localparam [6:0] SEG_d = 7'h5E;
    localparam [6:0] SEG_o = 7'h5C;
    localparam [6:0] SEG_t = 7'h78;
    localparam [6:0] SEG_S = 7'h6D;
    localparam [6:0] SEG_G = 7'h3D;      // 单位 G（无 g 段，避免与 6 混淆）
    localparam [6:0] SEG_H = 7'h76;      // HELLO
    localparam [6:0] SEG_E = 7'h79;      // HELLO / rEAdy
    localparam [6:0] SEG_F = 7'h71;      // FALL
    localparam [6:0] SEG_r = 7'h50;      // rEAdy / run
    localparam [6:0] SEG_u = 7'h1C;      // run
    localparam [6:0] SEG_n = 7'h54;      // run
    localparam [6:0] SEG_y = 7'h6E;      // rEAdy
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
    // 闪烁相位：告警 3.3Hz / 待确认 4Hz / 无链路 0.5Hz
    //------------------------------------------------------------------
    reg [31:0] blink_cnt;
    reg        blink_on;                  // 告警急闪相位
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blink_cnt <= 32'd0;
            blink_on  <= 1'b1;
        end else if (blink_cnt == BLINK_HALF_DIV - 1) begin
            blink_cnt <= 32'd0;
            blink_on  <= ~blink_on;
        end else begin
            blink_cnt <= blink_cnt + 32'd1;
        end
    end

    reg [31:0] pend_cnt;
    reg        pend_blink_on;             // 待确认快闪相位
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

    reg [31:0] lblink_cnt;
    reg        link_blink_on;             // 无链路慢闪相位
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lblink_cnt     <= 32'd0;
            link_blink_on  <= 1'b1;
        end else if (lblink_cnt == LINK_BLINK_HALF_DIV - 1) begin
            lblink_cnt     <= 32'd0;
            link_blink_on  <= ~link_blink_on;
        end else begin
            lblink_cnt <= lblink_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // 遥测老化看门狗（只作用于待确认数值显示）：
    //   3s 未收到有效遥测 -> 待确认画面显示 "------"。
    //   老化判定完全在 FPGA 侧，不依赖 S3 行为；告警画面不再显示数值
    //   （显示 FALL），因此无需"告警冻结"逻辑。
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
        end else if (stale_en) begin
            if (stale_cnt < TELEM_STALE_DIV[27:0]) begin
                stale_cnt <= stale_cnt + 28'd1;
            end else begin
                mag_fresh <= 1'b0;
            end
        end
        // stale_en=0（演示合成值）：计时器与新鲜度均保持
    end

    // 真正驱动显示的"读数有效"：
    //   正常模式 = 收到过有效遥测且未超时；
    //   演示模式（stale_en=0，合成值）= 永远有效（合成值不存在老化）。
    wire rd_valid = mag_valid & (mag_fresh | ~stale_en);

    //------------------------------------------------------------------
    // 遥测读数拆分：右对齐 "N.NNG"（N.NN = XX.XX g）
    //   整数部分 1~2 位，小数固定 2 位，末位单位 G。
    //------------------------------------------------------------------
    wire [7:0] int_part  = mag_x100 / 16'd100;           // 0..99
    wire [6:0] frac_part = mag_x100 % 16'd100;           // 0..99
    wire [3:0] f_hi      = frac_part / 7'd10;            // 十分位
    wire [3:0] f_lo      = frac_part % 7'd10;            // 百分位
    wire        two_int  = (int_part >= 8'd10);
    wire [3:0]  i_tens   = int_part / 8'd10;
    wire [3:0]  i_ones   = int_part % 8'd10;

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

    // 走灯序号十进制拆分（0..11）
    wire [3:0] w_tens = walk_idx / 4'd10;
    wire [3:0] w_ones = walk_idx % 4'd10;

    //------------------------------------------------------------------
    // 段码自检：special==2 时内部步进自检字符表，一轮完成发 self_done
    //------------------------------------------------------------------
    reg [31:0] self_cnt;
    reg [4:0]  self_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            self_cnt  <= 32'd0;
            self_idx  <= 5'd0;
            self_done <= 1'b0;
        end else begin
            self_done <= 1'b0;
            if (special == 2'b10) begin
                if (self_cnt == SELF_SLOT_DIV - 1) begin
                    self_cnt <= 32'd0;
                    if (self_idx == GLYPH_N - 1) begin
                        self_idx  <= 5'd0;
                        self_done <= 1'b1;       // 一轮完成
                    end else begin
                        self_idx <= self_idx + 5'd1;
                    end
                end else begin
                    self_cnt <= self_cnt + 32'd1;
                end
            end else begin
                self_cnt <= 32'd0;
                self_idx <= 5'd0;
            end
        end
    end

    // 自检字符表（覆盖数字与全部新增词形字形 + 单位 G）
    function [6:0] self_glyph;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  self_glyph = SEG_0;
                5'd1:  self_glyph = SEG_1;
                5'd2:  self_glyph = SEG_2;
                5'd3:  self_glyph = SEG_3;
                5'd4:  self_glyph = SEG_4;
                5'd5:  self_glyph = SEG_5;
                5'd6:  self_glyph = SEG_6;
                5'd7:  self_glyph = SEG_7;
                5'd8:  self_glyph = SEG_8;
                5'd9:  self_glyph = SEG_9;
                5'd10: self_glyph = SEG_H;
                5'd11: self_glyph = SEG_E;
                5'd12: self_glyph = SEG_F;
                5'd13: self_glyph = SEG_A;
                5'd14: self_glyph = SEG_L;
                5'd15: self_glyph = SEG_d;
                5'd16: self_glyph = SEG_o;
                5'd17: self_glyph = SEG_r;
                5'd18: self_glyph = SEG_u;
                5'd19: self_glyph = SEG_n;
                5'd20: self_glyph = SEG_t;
                5'd21: self_glyph = SEG_y;
                5'd22: self_glyph = SEG_G;
                default: self_glyph = SEG_BLANK;
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // 每 位内容（未取反段码 + dp + 整位消隐）
    //------------------------------------------------------------------
    reg [6:0] seg_raw;
    reg       dp_raw;
    reg       dig_blank;

    always @(*) begin
        seg_raw   = SEG_BLANK;
        dp_raw    = 1'b0;
        dig_blank = 1'b0;

        if (alarm_on) begin
            // 告警 / POST：FALL 整屏急闪
            case (digit_idx)
                3'd5: seg_raw = SEG_F;
                3'd4: seg_raw = SEG_A;
                3'd3: seg_raw = SEG_L;
                3'd2: seg_raw = SEG_L;
                default: seg_raw = SEG_BLANK;
            endcase
            if (!blink_on) dig_blank = 1'b1;
        end else if (special == 2'b01) begin
            // 走灯诊断：Ld + 两位序号
            case (digit_idx)
                3'd5: seg_raw = SEG_L;
                3'd4: seg_raw = SEG_d;
                3'd2: seg_raw = dec_seg(w_tens);
                3'd1: seg_raw = dec_seg(w_ones);
                default: seg_raw = SEG_BLANK;
            endcase
        end else if (special == 2'b10) begin
            // 段码自检：全 6 位同显一个字符，dp 交替检查
            seg_raw = self_glyph(self_idx);
            dp_raw  = self_idx[0];
        end else if (special == 2'b11) begin
            // 音调试听：t + 档位号（1..3）
            case (digit_idx)
                3'd5: seg_raw = SEG_t;
                3'd4: seg_raw = dec_seg({2'd0, self_tone} + 4'd1);
                default: seg_raw = SEG_BLANK;
            endcase
        end else if (!linked) begin
            // 无链路：HELLO 整屏慢闪
            case (digit_idx)
                3'd5: seg_raw = SEG_H;
                3'd4: seg_raw = SEG_E;
                3'd3: seg_raw = SEG_L;
                3'd2: seg_raw = SEG_L;
                3'd1: seg_raw = SEG_o;
                default: seg_raw = SEG_BLANK;
            endcase
            if (!link_blink_on) dig_blank = 1'b1;
        end else begin
            case (state_code)
                2'd0: begin
                    // 静止待机：rEAdy
                    case (digit_idx)
                        3'd5: seg_raw = SEG_r;
                        3'd4: seg_raw = SEG_E;
                        3'd3: seg_raw = SEG_A;
                        3'd2: seg_raw = SEG_d;
                        3'd1: seg_raw = SEG_y;
                        default: seg_raw = SEG_BLANK;
                    endcase
                end
                2'd1: begin
                    // 走动：run
                    case (digit_idx)
                        3'd5: seg_raw = SEG_r;
                        3'd4: seg_raw = SEG_u;
                        3'd3: seg_raw = SEG_n;
                        default: seg_raw = SEG_BLANK;
                    endcase
                end
                2'd2: begin
                    // 待确认：右对齐 "N.NNG"，无遥测显示 "------"；整屏 4Hz 快闪
                    if (rd_valid) begin
                        case (digit_idx)
                            3'd5: seg_raw = two_int ? dec_seg(i_tens) : SEG_BLANK;
                            3'd4: begin
                                seg_raw = two_int ? dec_seg(i_ones) : SEG_BLANK;
                                dp_raw  = two_int;
                            end
                            3'd3: begin
                                seg_raw = dec_seg(i_ones);
                                dp_raw  = ~two_int;
                            end
                            3'd2: seg_raw = dec_seg(f_hi);
                            3'd1: seg_raw = dec_seg(f_lo);
                            3'd0: seg_raw = SEG_G;
                            default: seg_raw = SEG_BLANK;
                        endcase
                    end else begin
                        seg_raw = SEG_DASH;
                    end
                    if (!pend_blink_on) dig_blank = 1'b1;
                end
                default: begin
                    // 防御：状态 3 但未告警（正常应由 disp_alarm 覆盖）
                    case (digit_idx)
                        3'd5: seg_raw = SEG_F;
                        3'd4: seg_raw = SEG_A;
                        3'd3: seg_raw = SEG_L;
                        3'd2: seg_raw = SEG_L;
                        default: seg_raw = SEG_BLANK;
                    endcase
                    if (!blink_on) dig_blank = 1'b1;
                end
            endcase
        end
    end

    //------------------------------------------------------------------
    // 输出：段码 + 小数点（可选取反）、位选（低电平选中）
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
