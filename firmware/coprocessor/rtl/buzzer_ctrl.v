`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : buzzer_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 蜂鸣器驱动。告警时输出「音频载波 x 报警节拍」两层调制信号；
//            待确认（状态 2）可选 1Hz 短提示音；其余情况静默。
//
// 接口说明 :
//   clk/rst_n    50MHz / 异步低有效复位
//   alarm_on     告警标志（top 仲裁后的最终值：UART 告警 | POST | 演示场景 4）
//   state_code   最终显示状态码（用于待确认提示音判定）
//   audio_mute   静音门（拨码本地静音 | 0x03 命令静音），**优先级高于一切发声**
//   pend_beep_en 待确认提示音使能（0x03 命令 p1 bit1）
//   tone_div_in  运行期载波半周期（0 = 使用 DEF_TONE_HALF）——0x03 命令 p2，
//                使音调可运行期调整，不再需要改 RTL 重综合
//   buzzer       蜂鸣器驱动输出（引脚 L2）
//   beat         节拍包络（1 = 告警本段应发声）。供 led_ctrl 做"声光同步爆闪"
//
// 输出波形说明 :
//   第 1 层：音频载波
//     buzzer 输出 tone_half_eff 决定的方波。默认 DEF_TONE_HALF = 9260
//     -> 50MHz / (2 x 9260) = 2.70kHz，小无源蜂鸣器的典型共鸣点。
//     【音量关键】无源蜂鸣器在共鸣频率附近的声压远高于低频通断。
//   第 2 层：报警节拍（三短一停，急促感）
//     响 150ms -> 停 100ms -> 响 150ms -> 停 100ms -> 响 150ms -> 静 600ms
//     1.25s 一个循环。告警输出 = 载波 AND 节拍。
//   待确认提示音（可选）：1Hz 周期、每周期响 PEND_ON_DIV（80ms）的短音，
//     同样叠加载波。提示音不改变 LED 的 KITT 扫描效果。
//
//   audio_mute = 1：输出恒 0（含 POST / 告警 / 提示音），告警节拍计数
//   保持复位态，解除静音后从节拍第 1 段重新开始。
//
//   演示场景 6 音调试听（tone_aud_en=1）：按 tone_sel_aud 档位
//   （AUD_TONE0/1/2 三档载波）以「响 AUD_ON / 停 AUD_OFF」节奏发声，
//   供现场选定共鸣点后经设置页把对应频率固化（0x03 命令 p2）。
//
// 参数说明 :
//   DEF_TONE_HALF 默认载波半周期（clk 数），9260 -> 2.70kHz @50MHz
//   BEAT_ON/GAP/REST_DIV 节拍时长（三短一停）
//   PEND_ON_DIV / PEND_PERIOD 提示音响时长 / 周期
//   AUD_ON/OFF_DIV 音调试听响/停时长；AUD_TONE0/1/2 三档半周期
//   仿真时由 tb 用 defparam 覆盖为小值以缩短仿真时间。
////////////////////////////////////////////////////////////////////////////////

module buzzer_ctrl #(
    parameter DEF_TONE_HALF = 9260,           // 2.70kHz 载波半周期 @50MHz
    parameter BEAT_ON_DIV   = 7500000,        // 150ms
    parameter BEAT_GAP_DIV  = 5000000,        // 100ms
    parameter BEAT_REST_DIV = 30000000,       // 600ms
    parameter PEND_ON_DIV   = 4_000_000,      // 提示音响 80ms
    parameter PEND_PERIOD   = 50_000_000,     // 提示音周期 1s
    parameter AUD_ON_DIV    = 10_000_000,     // 试听响 200ms
    parameter AUD_OFF_DIV   = 20_000_000,     // 试听停 400ms
    parameter AUD_TONE0     = 9260,           // 低档 2.7kHz
    parameter AUD_TONE1     = 8000,           // 中档 ~3.1kHz
    parameter AUD_TONE2     = 6250            // 高档 4.0kHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        alarm_on,
    input  wire [1:0]  state_code,
    input  wire        audio_mute,
    input  wire        pend_beep_en,
    input  wire [15:0] tone_div_in,
    input  wire        tone_aud_en,
    input  wire [1:0]  tone_sel_aud,
    output wire        buzzer,
    output wire        beat
);

    //------------------------------------------------------------------
    // 有效载波半周期：
    //   音调试听相（tone_aud_en=1）按档位表取值；否则用 0x03 命令值
    //   （0 = 默认 DEF_TONE_HALF）
    //------------------------------------------------------------------
    reg [15:0] aud_half;
    always @(*) begin
        case (tone_sel_aud)
            2'd0:    aud_half = AUD_TONE0[15:0];
            2'd1:    aud_half = AUD_TONE1[15:0];
            default: aud_half = AUD_TONE2[15:0];
        endcase
    end

    wire [15:0] cmd_half     = (tone_div_in == 16'd0) ? DEF_TONE_HALF[15:0]
                                                      : tone_div_in;
    wire [15:0] tone_half_eff = tone_aud_en ? aud_half : cmd_half;

    //------------------------------------------------------------------
    // 音频载波：自由运行（告警与提示音共用同一载波）
    //------------------------------------------------------------------
    reg [31:0] tone_cnt;
    reg        tone_sq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tone_cnt <= 32'd0;
            tone_sq  <= 1'b0;
        end else begin
            if (tone_cnt >= {16'd0, tone_half_eff} - 32'd1) begin
                tone_cnt <= 32'd0;
                tone_sq  <= ~tone_sq;
            end else begin
                tone_cnt <= tone_cnt + 32'd1;
            end
        end
    end

    //------------------------------------------------------------------
    // 告警节拍包络：seg_idx 为当前段号，偶数段（0/2/4）为"响"
    //   静音期间保持复位态（解除静音后从第 1 段重新开始）
    //------------------------------------------------------------------
    reg [2:0]  seg_idx;      // 0..5
    reg [31:0] seg_cnt;

    wire [31:0] seg_len = (seg_idx == 3'd5) ? BEAT_REST_DIV :
                          (seg_idx[0] == 1'b1) ? BEAT_GAP_DIV :
                                                 BEAT_ON_DIV;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_idx <= 3'd0;
            seg_cnt <= 32'd0;
        end else if (!alarm_on || audio_mute) begin
            // 告警解除 / 静音：节拍全部清零，恢复后从头开始
            seg_idx <= 3'd0;
            seg_cnt <= 32'd0;
        end else begin
            if (seg_cnt == seg_len - 1) begin
                seg_cnt <= 32'd0;
                seg_idx <= (seg_idx == 3'd5) ? 3'd0 : seg_idx + 3'd1;
            end else begin
                seg_cnt <= seg_cnt + 32'd1;
            end
        end
    end

    assign beat = (seg_idx == 3'd0) ||
                  (seg_idx == 3'd2) ||
                  (seg_idx == 3'd4);

    //------------------------------------------------------------------
    // 待确认提示音：1s 周期自由计数，前 PEND_ON_DIV 为响段
    //------------------------------------------------------------------
    reg [31:0] pend_cnt;
    wire       pend_win    = (pend_cnt < PEND_ON_DIV);
    wire       pend_active = pend_beep_en && (state_code == 2'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                           pend_cnt <= 32'd0;
        else if (pend_cnt >= PEND_PERIOD - 1) pend_cnt <= 32'd0;
        else                                  pend_cnt <= pend_cnt + 32'd1;
    end

    //------------------------------------------------------------------
    // 音调试听窗口：响 AUD_ON / 停 AUD_OFF 自由循环
    //------------------------------------------------------------------
    reg [31:0] aud_cnt;
    wire       aud_win = (aud_cnt < AUD_ON_DIV);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) aud_cnt <= 32'd0;
        else if (aud_cnt >= (AUD_ON_DIV + AUD_OFF_DIV) - 1) aud_cnt <= 32'd0;
        else                                                aud_cnt <= aud_cnt + 32'd1;
    end

    //------------------------------------------------------------------
    // 输出仲裁：静音 > 告警（载波 x 节拍） > 待确认提示音 > 音调试听 > 静默
    //------------------------------------------------------------------
    assign buzzer = audio_mute  ? 1'b0 :
                    alarm_on    ? (beat & tone_sq) :
                    pend_active ? (pend_win & tone_sq) :
                    tone_aud_en ? (aud_win & tone_sq) : 1'b0;

endmodule
