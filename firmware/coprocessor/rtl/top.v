`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : top
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 跌倒检测基站的 FPGA 协处理器顶层。
//            接收 ESP32-S3 经 UART 下发的 8 字节控制帧（协议见
//            docs/团队协作规范与接口契约.md §4），解码后驱动声光告警；
//            同时读取板上人机输入（4x4 矩阵键盘 + 拨码），经上行 UART
//            报告给 S3 驱动 OLED 多页面 UI，并在演示模式下本地接管声光。
//
// 系统数据流 :
//   rxd -> uart_rx -> cmd_decoder -+--> seg_display -> seg/an -> 数码管
//                                  +--> buzzer_ctrl -> buzzer -> 蜂鸣器
//                                  |               \-> beat
//                                  +--> led_ctrl <----/      -> led
//   key_row/col + sw -> input_ctrl -+--> demo_scene/demo_mag（演示本地接管）
//                                  +--> uart_tx -> txd（上行：快照/键事件）
//
// 显示仲裁（优先级从高到低）：
//   1) 告警 = UART ALARM | 上电自检(POST) | 演示场景4 —— 声光全开
//   2) 演示模式（sw_demo=1 且场景!=0）本地接管显示内容；
//      场景 0 = S3 全链路剧本，FPGA 透传 UART 命令（等同正常运行）
//   3) 正常运行：UART 命令内容
//   静音 = 拨码 sw_mute | 0x03 命令静音位，优先级高于一切发声（含 POST）。
//
// 数码管显示整屏"完整英文词 + 带单位数值"（HELLO/rEAdy/run/FALL/N.NNG），
// 可用 LED 数由 led_ctrl.LED_COUNT 决定（默认 12，LED12~16 禁用恒灭）。
//
// 上电自检（POST）:
//   上电后前 POST_CYCLES 个时钟周期（默认 2s）强制走告警显示路径：
//   数码管 FALL 急闪、蜂鸣器三短一停鸣响（静音开关可完全静音）、
//   可用 LED 随节拍爆闪；随后自动进入正常待机（无链路时 HELLO 慢闪）。
//   用途：不接 S3、单板上电即可验证数码管/蜂鸣器/LED 全链路。
//   自检期间 UART 命令照常解码锁存，仅显示/蜂鸣输出被自检覆盖。
//
// 接口说明 :
//   clk        50MHz 主时钟（D4）
//   rst_n      低有效复位（T9，XDC 中开内部 PULLUP）
//   rxd        UART 接收（B15），来自 S3 的 TX
//   txd        UART 上行发送（现阶段约束 F12 板上 USB-UART 桥，电脑串口
//              可直接观测上行帧；接 S3 时改约束到扩展排母空闲脚）
//   sw_demo    演示模式总开关（T8，拨码 sw[1]）
//   sw_mute    本地硬静音（N4，拨码 sw[9]）
//   key_row    矩阵键盘行扫描输出（K3/M6/P10/R10）
//   key_col    矩阵键盘列输入（R12/T12/R11/T10，板上拉）
//   led[15:0]  LED，高电平点亮；led[11:0] 有效，led[15:12] 恒灭（禁用）
//   seg[7:0]   数码管段选（seg[6:0]=gfedcba，seg[7]=dp）
//   an[5:0]    数码管位选，低电平选中
//   buzzer     蜂鸣器驱动（L2）
//
// 端口名与 pins.xdc 严格一一对应，改名必须同步改 XDC。
////////////////////////////////////////////////////////////////////////////////

module top #(
    parameter POST_CYCLES = 100000000   // 上电自检时长：2s @50MHz（tb 覆盖为小值）
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rxd,
    output wire        txd,
    input  wire        sw_demo,
    input  wire        sw_mute,
    output wire [3:0]  key_row,
    input  wire [3:0]  key_col,
    output wire [15:0] led,
    output wire [7:0]  seg,
    output wire [5:0]  an,
    output wire        buzzer
);

    //------------------------------------------------------------------
    // 上电自检计数器：计满 POST_CYCLES 后停住（饱和，防回绕）
    //------------------------------------------------------------------
    reg [26:0] post_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            post_cnt <= 27'd0;
        else if (post_cnt < POST_CYCLES)
            post_cnt <= post_cnt + 27'd1;
    end

    wire post_active = (post_cnt < POST_CYCLES);   // 自检进行中

    //------------------------------------------------------------------
    // UART 接收 + 命令解码
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

    wire        alarm_on;
    wire [1:0]  state_code;
    wire [15:0] mag_x100;
    wire        mag_valid;
    wire        mag_update;
    wire        audio_mute_cmd;
    wire        pend_beep_en;
    wire [15:0] tone_div;
    wire        ever_linked;
    wire        cmd_valid;
    wire        cmd_error;

    cmd_decoder u_cmd_decoder (
        .clk          (clk),
        .rst_n        (rst_n),
        .byte_valid   (byte_valid),
        .byte_data    (byte_data),
        .alarm_on     (alarm_on),
        .state_code   (state_code),
        .mag_x100     (mag_x100),
        .mag_valid    (mag_valid),
        .mag_update   (mag_update),
        .audio_mute   (audio_mute_cmd),
        .pend_beep_en (pend_beep_en),
        .tone_div     (tone_div),
        .ever_linked  (ever_linked),
        .cmd_valid    (cmd_valid),
        .cmd_error    (cmd_error)
    );

    //------------------------------------------------------------------
    // 人机输入子系统（键盘/拨码 + 演示场景 + 上行链路）
    //------------------------------------------------------------------
    wire        demo_en;
    wire        mute_local;
    wire [2:0]  demo_scene;
    wire [15:0] demo_mag;
    wire [3:0]  walk_idx;
    wire        self_tone_phase;
    wire [1:0]  self_tone;
    wire        self_done;

    input_ctrl u_input_ctrl (
        .clk             (clk),
        .rst_n           (rst_n),
        .key_row         (key_row),
        .key_col         (key_col),
        .sw_demo         (sw_demo),
        .sw_mute         (sw_mute),
        .demo_en         (demo_en),
        .mute_local      (mute_local),
        .demo_scene      (demo_scene),
        .demo_mag        (demo_mag),
        .walk_idx        (walk_idx),
        .self_tone_phase (self_tone_phase),
        .self_tone       (self_tone),
        .self_done       (self_done),
        .uplink_txd      (txd)
    );

    //------------------------------------------------------------------
    // 显示/发声仲裁
    //------------------------------------------------------------------
    // 演示模式本地接管（场景 0 = S3 剧本 = 透传，不算接管）
    wire demo_owned = demo_en && !alarm_on && !post_active && (demo_scene != 3'd0);
    // 演示场景 4 = 告警演示（真实告警优先级不变，此处为并集）
    wire demo_alarm = demo_en && (demo_scene == 3'd4);
    // 最终告警：真实 UART 告警 | 上电自检 | 演示告警场景
    wire disp_alarm = alarm_on | post_active | demo_alarm;

    // 演示场景 -> 状态码映射（场景 1/2/3 对应状态 0/1/2；4 走告警路径）
    reg [1:0] scene_state;
    always @(*) begin
        case (demo_scene)
            3'd1:    scene_state = 2'd0;
            3'd2:    scene_state = 2'd1;
            3'd3:    scene_state = 2'd2;
            default: scene_state = 2'd0;
        endcase
    end

    wire [1:0]  disp_state     = demo_owned ? scene_state : state_code;
    wire [15:0] disp_mag       = demo_owned ? demo_mag    : mag_x100;
    wire        disp_mag_valid = demo_owned ? 1'b1        : mag_valid;

    // 数码管特殊画面（演示场景 5/6）
    reg [1:0] seg_special;
    always @(*) begin
        if (demo_owned && (demo_scene == 3'd5))
            seg_special = 2'b01;                                   // 走灯
        else if (demo_owned && (demo_scene == 3'd6))
            seg_special = self_tone_phase ? 2'b11 : 2'b10;         // 音调/段码
        else
            seg_special = 2'b00;
    end

    //------------------------------------------------------------------
    // 数码管
    //------------------------------------------------------------------
    seg_display u_seg_display (
        .clk        (clk),
        .rst_n      (rst_n),
        .linked     (ever_linked),
        .state_code (disp_state),
        .alarm_on   (disp_alarm),
        .mag_x100   (disp_mag),
        .mag_valid  (disp_mag_valid),
        .mag_update (mag_update),
        .stale_en   (!demo_owned),        // 演示合成值永不老化
        .special    (seg_special),
        .walk_idx   (walk_idx),
        .self_tone  (self_tone),
        .self_done  (self_done),
        .seg        (seg),
        .an         (an)
    );

    //------------------------------------------------------------------
    // 蜂鸣器（静音优先级最高；beat 同时供 LED 声光同步）
    //------------------------------------------------------------------
    wire beep_beat;

    buzzer_ctrl u_buzzer_ctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .alarm_on     (disp_alarm),
        .state_code   (disp_state),
        .audio_mute   (mute_local | audio_mute_cmd),
        .pend_beep_en (pend_beep_en),
        .tone_div_in  (tone_div),
        .tone_aud_en  (demo_owned && (demo_scene == 3'd6) && self_tone_phase),
        .tone_sel_aud (self_tone),
        .buzzer       (buzzer),
        .beat         (beep_beat)
    );

    //------------------------------------------------------------------
    // LED 效果引擎（LED12~16 禁用，led[15:12] 恒灭）
    //------------------------------------------------------------------
    led_ctrl u_led_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .state_code (disp_state),
        .alarm_on   (disp_alarm),
        .beat       (beep_beat),
        .walk_mode  (demo_owned && (demo_scene == 3'd5)),
        .walk_pos   (walk_idx),
        .link_ok    (ever_linked),
        .link_act   (cmd_valid),
        .led        (led)
    );

endmodule
