`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : top
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 跌倒检测基站的 FPGA 协处理器顶层。
//            接收 ESP32-S3 经 UART 下发的 8 字节控制帧（协议见
//            docs/团队协作规范与接口契约.md §4），解码后驱动声光告警：
//              - 6 位数码管显示状态 / 告警时闪烁 "FALL--"
//              - 蜂鸣器告警时断续鸣响
//              - 16 颗 LED 指示状态，告警时全亮
//
// 系统数据流 :
//   rxd(F13) -> uart_rx -> cmd_decoder -> +--> seg_display -> seg/an -> 数码管
//                                         +--> buzzer_ctrl  -> buzzer -> 蜂鸣器
//                                         +--> LED 状态映射 -> led
//
// 上电自检（POST）:
//   上电后前 POST_CYCLES 个时钟周期（默认 2s）强制走告警显示路径：
//   数码管 FALL-- 闪烁、蜂鸣器断续鸣响、16 颗 LED 全亮；随后自动恢复
//   正常待机。用途：不接 S3、单板上电即可验证数码管/蜂鸣器/LED 全链路
//   （板间独立验证）。自检期间 UART 命令照常解码锁存，仅显示/蜂鸣输出
//   被自检覆盖，自检结束后生效。
//
// 接口说明 :
//   clk        50MHz 主时钟（D4）
//   rst_n      低有效复位（T9，XDC 中开内部 PULLUP）
//   rxd        UART 接收（F13），来自 S3 的 TX
//   txd        UART 发送（F12）。本期 S3->FPGA 单向，故恒驱动为 1（空闲态）
//   led[15:0]  16 颗 LED，高电平点亮
//   seg[7:0]   数码管段选（seg[6:0]=gfedcba，seg[7]=dp）
//   an[5:0]    数码管位选，低电平选中
//   buzzer     蜂鸣器驱动（L2）
//
// 端口名与 pins.xdc 严格一一对应，改名必须同步改 XDC。
//
// 已知板卡个体问题：led[15]（引脚 M1）在本板物理不亮，非工程问题，验收忽略。
//////////////////////////////////////////////////////////////////////////////////

module top #(
    parameter POST_CYCLES = 100000000   // 上电自检时长：2s @50MHz（tb 覆盖为小值）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rxd,
    output wire        txd,
    output wire [15:0] led,
    output wire [7:0]  seg,
    output wire [5:0]  an,
    output wire        buzzer
);

    //------------------------------------------------------------------
    // 上电自检计数器：计满 POST_CYCLES 后停住（饱和，防回绕）
    //------------------------------------------------------------------
    reg [26:0] post_cnt;                       // 2^27 = 134M > 100M，够用

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            post_cnt <= 27'd0;
        else if (post_cnt < POST_CYCLES)
            post_cnt <= post_cnt + 27'd1;
    end

    wire post_active = (post_cnt < POST_CYCLES);   // 自检进行中

    //------------------------------------------------------------------
    // UART 接收
    //------------------------------------------------------------------
    wire       byte_valid;
    wire [7:0] byte_data;

    uart_rx u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rxd        (rxd),
        .byte_data  (byte_data),
        .byte_valid (byte_valid)
    );

    //------------------------------------------------------------------
    // 命令解码
    //------------------------------------------------------------------
    wire       alarm_on;
    wire [1:0] state_code;
    wire       cmd_valid;
    wire       cmd_error;

    cmd_decoder u_cmd_decoder (
        .clk        (clk),
        .rst_n      (rst_n),
        .byte_valid (byte_valid),
        .byte_data  (byte_data),
        .alarm_on   (alarm_on),
        .state_code (state_code),
        .cmd_valid  (cmd_valid),
        .cmd_error  (cmd_error)
    );

    //------------------------------------------------------------------
    // 告警通路仲裁：自检期间强制走告警显示，其余以命令解码结果为准
    //------------------------------------------------------------------
    wire disp_alarm = alarm_on | post_active;

    //------------------------------------------------------------------
    // 数码管显示（参数默认值用于综合；tb 用 defparam 覆盖为小值便于仿真）
    //------------------------------------------------------------------
    seg_display u_seg_display (
        .clk        (clk),
        .rst_n      (rst_n),
        .state_code (state_code),
        .alarm_on   (disp_alarm),
        .seg        (seg),
        .an         (an)
    );

    //------------------------------------------------------------------
    // 蜂鸣器告警（同上，tb 用 defparam 覆盖 BEEP_HALF_DIV）
    //------------------------------------------------------------------
    buzzer_ctrl u_buzzer_ctrl (
        .clk      (clk),
        .rst_n    (rst_n),
        .alarm_on (disp_alarm),
        .buzzer   (buzzer)
    );

    //------------------------------------------------------------------
    // LED 状态映射
    //   告警（或自检）：16 颗全亮（最醒目的"光"告警）
    //   正常：led[0]=state_code[0]，led[1]=state_code[1]，其余灭
    //------------------------------------------------------------------
    assign led = disp_alarm ? 16'hFFFF : {14'h0000, state_code[1], state_code[0]};

    //------------------------------------------------------------------
    // UART 发送：本期不用，恒驱动空闲高电平
    //------------------------------------------------------------------
    assign txd = 1'b1;

endmodule
