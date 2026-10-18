`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : top
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 跌倒检测基站的 FPGA 协处理器顶层。
//            接收 ESP32-S3 经 UART 下发的 8 字节控制帧（协议见
//            docs/团队协作规范与接口契约.md §4），解码后驱动声光告警：
//              - 6 位数码管：状态显示 "S+数字"（小数点心跳）/ 待确认数字
//                快闪 / 告警时 "FALL--" 急闪 + 小数点流水
//              - 蜂鸣器：音频载波 x 三短一停节拍（详见 buzzer_ctrl.v）
//              - 16 颗 LED：呼吸灯 / 环形流水 / KITT 扫描 / 告警时与蜂鸣
//                节拍同步的声光爆闪（详见 led_ctrl.v）
//
// 系统数据流 :
//   rxd -> uart_rx -> cmd_decoder -> +--> seg_display -> seg/an -> 数码管
//                                   +--> buzzer_ctrl  -> buzzer -> 蜂鸣器
//                                   |                \-> beat
//                                   +--> led_ctrl <----/     -> led
//
// 上电自检（POST）:
//   上电后前 POST_CYCLES 个时钟周期（默认 2s）强制走告警显示路径：
//   数码管 FALL-- 急闪、蜂鸣器三短一停鸣响、16 颗 LED 随节拍爆闪；随后
//   自动恢复正常待机。用途：不接 S3、单板上电即可验证数码管/蜂鸣器/LED
//   全链路（板间独立验证）。自检期间 UART 命令照常解码锁存，仅显示/蜂鸣
//   输出被自检覆盖，自检结束后生效。
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
    wire [15:0] mag_x100;
    wire       mag_valid;
    wire       mag_update;
    wire       cmd_valid;
    wire       cmd_error;

    cmd_decoder u_cmd_decoder (
        .clk        (clk),
        .rst_n      (rst_n),
        .byte_valid (byte_valid),
        .byte_data  (byte_data),
        .alarm_on   (alarm_on),
        .state_code (state_code),
        .mag_x100   (mag_x100),
        .mag_valid  (mag_valid),
        .mag_update (mag_update),
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
        .mag_x100   (mag_x100),
        .mag_valid  (mag_valid),
        .mag_update (mag_update),
        .seg        (seg),
        .an         (an)
    );

    //------------------------------------------------------------------
    // 蜂鸣器告警（同上，tb 用 defparam 覆盖节拍/载波分频）
    //   beat 为节拍包络，同时送给 led_ctrl 做声光同步爆闪
    //------------------------------------------------------------------
    wire beep_beat;

    buzzer_ctrl u_buzzer_ctrl (
        .clk      (clk),
        .rst_n    (rst_n),
        .alarm_on (disp_alarm),
        .buzzer   (buzzer),
        .beat     (beep_beat)
    );

    //------------------------------------------------------------------
    // LED 效果引擎
    //   告警（或自检）：16 颗随蜂鸣节拍同步爆闪（最醒目的声光告警）
    //   状态 0/1/2   ：呼吸灯 / 环形流水 / KITT 扫描（详见 led_ctrl.v）
    //------------------------------------------------------------------
    led_ctrl u_led_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .state_code (state_code),
        .alarm_on   (disp_alarm),
        .beat       (beep_beat),
        .led        (led)
    );

    //------------------------------------------------------------------
    // UART 发送：本期不用，恒驱动空闲高电平
    //------------------------------------------------------------------
    assign txd = 1'b1;

endmodule
