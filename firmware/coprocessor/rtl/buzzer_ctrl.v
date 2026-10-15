`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : buzzer_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 跌倒告警蜂鸣器驱动。告警时输出断续方波（通断交替），
//            未告警时输出恒 0（静默）。
//
// 接口说明 :
//   clk        50MHz 系统时钟
//   rst_n      低有效异步复位
//   alarm_on   告警标志（来自 cmd_decoder）
//   buzzer     蜂鸣器驱动输出（引脚 L2）
//
// 输出波形说明 :
//   alarm_on = 1 : buzzer 以 BEEP_HALF_DIV 决定的半周期方波断续鸣响
//                  默认 12_500_000 clk = 250ms，即响 250ms / 停 250ms
//                  （2Hz 完整周期）。板载为有源蜂鸣器时，此方波即为
//                  "嘀-嘀-嘀"断续声；若为无源蜂鸣器，需在此模块内再叠
//                  加音频载波（本期板载为有源，故直接用通断调制）。
//   alarm_on = 0 : buzzer = 0，且计数器清零，保证解除告警后立刻静音，
//                  下次告警从头开始一个完整周期。
//
// 参数说明 :
//   BEEP_HALF_DIV 半周期时钟数，默认 12_500_000 → 250ms。
//                 仿真时由 tb 覆盖为小值以缩短仿真时间。
//////////////////////////////////////////////////////////////////////////////////

module buzzer_ctrl #(
    parameter BEEP_HALF_DIV = 12500000        // 250ms 半周期 @50MHz
)(
    input  wire clk,
    input  wire rst_n,
    input  wire alarm_on,
    output reg  buzzer
);

    reg [31:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt    <= 32'd0;
            buzzer <= 1'b0;
        end else if (!alarm_on) begin
            // 告警解除：立即静音并清零计数
            cnt    <= 32'd0;
            buzzer <= 1'b0;
        end else begin
            if (cnt == BEEP_HALF_DIV - 1) begin
                cnt    <= 32'd0;
                buzzer <= ~buzzer;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end

endmodule
