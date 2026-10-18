`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : buzzer_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 跌倒告警蜂鸣器驱动。告警时输出「音频载波 x 报警节拍」两层调制信号，
//            未告警时输出恒 0（静默）。
//
// 接口说明 :
//   clk        50MHz 系统时钟
//   rst_n      低有效异步复位
//   alarm_on   告警标志（来自 cmd_decoder，自检期间由 top 强制为 1）
//   buzzer     蜂鸣器驱动输出（引脚 L2）
//   beat       节拍包络（1 = 本段应发声）。供 led_ctrl 做"声光同步爆闪"，
//              不引出到引脚。
//
// 输出波形说明 :
//   第 1 层：音频载波（TONE_ENABLE=1 时）
//     buzzer 输出 TONE_HALF_DIV 决定的方波，默认 9_260 clk 半周期
//     -> 50MHz / (2 x 9260) = 2.70kHz，小无源蜂鸣器的典型共鸣点。
//     【音量关键】无源蜂鸣器在共鸣频率附近的声压远高于低频通断：
//     原实现只做 2Hz 通断，对无源蜂鸣器仅是"咔哒"声；叠加载波后
//     才是真正的响亮蜂鸣，这是本次"增大音量"的主要手段。
//   第 2 层：报警节拍（三短一停，急促感）
//     响 150ms -> 停 100ms -> 响 150ms -> 停 100ms -> 响 150ms -> 静 600ms
//     1.25s 一个循环。buzzer = 载波 AND 节拍。
//
//   alarm_on = 0 : buzzer = 0，所有计数器清零，保证解除告警后立刻静音，
//                  下次告警从节拍第 1 段重新开始。
//
// 参数说明 :
//   TONE_ENABLE    1 = 叠加音频载波（默认，适配无源蜂鸣器）；
//                  0 = 纯节拍通断（若实板确认为有源蜂鸣器且载波叠加后
//                      音色变差，改 0 即回到干净的通断驱动，一处回退）。
//   TONE_HALF_DIV  载波半周期（clk 数）。默认 9260 -> 2.70kHz。
//                  若实板蜂鸣器共鸣点偏高（4kHz 一档），可改 6250 试。
//   BEAT_ON_DIV    节拍"响"段时长（clk 数），默认 7_500_000 -> 150ms。
//   BEAT_GAP_DIV   节拍"停"段时长（clk 数），默认 5_000_000 -> 100ms。
//   BEAT_REST_DIV  一轮结束后的静默段（clk 数），默认 30_000_000 -> 600ms。
//                  仿真时由 tb 用 defparam 覆盖为小值以缩短仿真时间。
//
// 节拍段序（seg_idx 0..5）:
//   0:响 1:停 2:响 3:停 4:响 5:静 -> 回到 0
//////////////////////////////////////////////////////////////////////////////////

module buzzer_ctrl #(
    parameter TONE_ENABLE   = 1,
    parameter TONE_HALF_DIV = 9260,           // 2.70kHz 载波半周期 @50MHz
    parameter BEAT_ON_DIV   = 7500000,        // 150ms
    parameter BEAT_GAP_DIV  = 5000000,        // 100ms
    parameter BEAT_REST_DIV = 30000000        // 600ms
)(
    input  wire clk,
    input  wire rst_n,
    input  wire alarm_on,
    output wire buzzer,
    output wire beat
);

    //------------------------------------------------------------------
    // 节拍包络：seg_idx 为当前段号，偶数段（0/2/4）为"响"
    //------------------------------------------------------------------
    reg [2:0]  seg_idx;      // 0..5
    reg [31:0] seg_cnt;

    wire [31:0] seg_len = (seg_idx == 3'd5) ? BEAT_REST_DIV :
                          (seg_idx[0] == 1'b1) ? BEAT_GAP_DIV :
                                                 BEAT_ON_DIV;

    assign beat = (seg_idx == 3'd0) ||
                  (seg_idx == 3'd2) ||
                  (seg_idx == 3'd4);

    //------------------------------------------------------------------
    // 音频载波：TONE_HALF_DIV 决定的方波
    //------------------------------------------------------------------
    reg [31:0] tone_cnt;
    reg        tone_sq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_idx  <= 3'd0;
            seg_cnt  <= 32'd0;
            tone_cnt <= 32'd0;
            tone_sq  <= 1'b0;
        end else if (!alarm_on) begin
            // 告警解除：立即静音、全部计数清零，下次告警从头开始
            seg_idx  <= 3'd0;
            seg_cnt  <= 32'd0;
            tone_cnt <= 32'd0;
            tone_sq  <= 1'b0;
        end else begin
            // --- 载波翻转 ---
            if (tone_cnt == TONE_HALF_DIV - 1) begin
                tone_cnt <= 32'd0;
                tone_sq  <= ~tone_sq;
            end else begin
                tone_cnt <= tone_cnt + 32'd1;
            end

            // --- 节拍推进 ---
            if (seg_cnt == seg_len - 1) begin
                seg_cnt <= 32'd0;
                seg_idx <= (seg_idx == 3'd5) ? 3'd0 : seg_idx + 3'd1;
            end else begin
                seg_cnt <= seg_cnt + 32'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // 输出：载波与节拍相乘（TONE_ENABLE=0 时不叠加载波）
    //------------------------------------------------------------------
    assign buzzer = (TONE_ENABLE == 1) ? (beat & tone_sq) : beat;

endmodule
