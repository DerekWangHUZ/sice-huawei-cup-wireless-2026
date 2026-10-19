`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 文件名称 : tb_top.v
// 功能描述 : coprocessor 顶层 testbench。无头仿真（xvlog/xelab/xsim）用，
//            不进 Vivado 工程源文件集，由 sim/run_sim.ps1 调用。
//
// 测试场景 :
//   a0) POST(静音)   : 上电自检期 LED 爆闪、数码管 FALL 急闪；拨码静音下
//                      蜂鸣器恒 0（静音优先级高于 POST）
//   a)  无链路       : POST 后显示 "HELLO " 慢闪（双相位），led[11] 慢闪，
//                      ever_linked=0，蜂鸣器静默，静止动效只用 led[0]/led[11]
//   a1) 首帧同步     : SET_STATE(0,0xFFFF) -> ever_linked=1 -> "rEAdy "
//   a2) 待确认读数   : SET_STATE(2,102) -> "  1.02G" 4Hz 快闪（单位 G）
//   a3) 走动         : SET_STATE(1,0) -> "run" + 环形流水（12 颗）推进
//   a4) 待确认读数2  : SET_STATE(2,340) -> "  3.40G"
//   a5) 遥测老化     : 超时无有效遥测 -> 待确认显示 "------"
//   b)  ALARM        : FALL 整屏急闪（不显示读数）+ 声光同步爆闪 + 载波
//   c)  CLEAR        : 蜂鸣器静默，回状态动效（KITT 扫描 0..11，显示 "------"）
//   d)  负测试-坏校验 : cmd_error 脉冲，状态不变
//   e)  负测试-未知命令: 无 cmd_valid，状态不变
//   f)  重同步       : 垃圾字节后紧跟合法 ALARM 帧仍解出
//   g)  SET_AUDIO    : 命令静音（告警无声但 LED 照常）/ 恢复 /
//                      运行期改音调（载波边沿计数骤降）/ 待确认 1Hz 提示音
//   h)  演示模式     : sw_demo=1 -> 上行快照 bit0=1；键盘 '4'(pos4) 选场景 4
//                      -> FALL + 爆闪；'C'(pos11) 调值 330；'3'(pos2) 显示
//                      合成值 "  3.30G"；'D'(pos15) 回 340；'5'(pos5) 走灯
//                      （Ld+序号、单灯推进）；'6'(pos6) 段码自检（全六位
//                      同字形+dp 交替）-> 音调试听（t1/t2 换档）；
//                      真实 ALARM 突破演示场景 1；sw_demo=0 退出（快照回 0）
//   i)  心跳快照     : SNAP_PERIOD 周期自发上行快照
//   不变式           : led[15:12] 全程恒灭（LED12~16 禁用）
//
// 键盘模型 : 行由 DUT 逐行驱动低，按下的键把行/列短接（列被拉低）。
//            键编号 = 板上丝印 KEY0~KEY15（矩阵位置 pos = 行*4+列，行主序），
//            与 rtl/input_ctrl.v 的 key_number() 一致。
//
// 参数覆盖说明 :
//   仿真用 defparam 把各模块分频参数改小。LED 为软 PWM，瞬时值在脉宽内
//   抖动，LED 断言一律用"观察窗口内是否出现过某形态"的累积标志。
//   上/下行 uart 位时间均为 8681ns（一帧 8 字节约 694us），涉及上行帧
//   完整接收的检查点均预留 >1 帧时长的等待。
////////////////////////////////////////////////////////////////////////////////

module tb_top;

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    reg rxd   = 1'b1;          // UART 空闲高电平

    always #10 clk = ~clk;     // 50MHz -> 20ns 周期

    //------------------------------------------------------------------
    // 输入激励寄存器
    //------------------------------------------------------------------
    reg        sw_demo = 1'b0;
    reg        sw_mute = 1'b0;
    reg [15:0] pressed = 16'h0;      // 键盘按下模型（pos 位图）

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    wire        txd;
    wire [3:0]  key_row;
    wire [3:0]  key_col;
    wire [15:0] led;
    wire [7:0]  seg;
    wire [5:0]  an;
    wire        buzzer;

    top dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .rxd      (rxd),
        .txd      (txd),
        .sw_demo  (sw_demo),
        .sw_mute  (sw_mute),
        .key_row  (key_row),
        .key_col  (key_col),
        .led      (led),
        .seg      (seg),
        .an       (an),
        .buzzer   (buzzer)
    );

    //------------------------------------------------------------------
    // 参数覆盖：缩短仿真时间
    //------------------------------------------------------------------
    defparam dut.POST_CYCLES = 5000;                       // 自检 100us

    defparam dut.u_seg_display.SCAN_DIV            = 50;     // 1us/位
    defparam dut.u_seg_display.BLINK_HALF_DIV      = 2000;   // 40us
    defparam dut.u_seg_display.PEND_BLINK_HALF_DIV = 1000;   // 20us
    defparam dut.u_seg_display.LINK_BLINK_HALF_DIV = 2000;   // 40us
    defparam dut.u_seg_display.TELEM_STALE_DIV     = 100000; // 2ms 老化
    defparam dut.u_seg_display.SELF_SLOT_DIV       = 2000;   // 40us/字符
    defparam dut.u_seg_display.GLYPH_N             = 23;

    defparam dut.u_buzzer_ctrl.DEF_TONE_HALF = 20;           // 载波周期 800ns
    defparam dut.u_buzzer_ctrl.BEAT_ON_DIV   = 1500;         // 30us
    defparam dut.u_buzzer_ctrl.BEAT_GAP_DIV  = 1000;         // 20us
    defparam dut.u_buzzer_ctrl.BEAT_REST_DIV = 4000;         // 80us
    defparam dut.u_buzzer_ctrl.PEND_ON_DIV   = 2000;         // 40us
    defparam dut.u_buzzer_ctrl.PEND_PERIOD   = 10000;        // 200us
    defparam dut.u_buzzer_ctrl.AUD_ON_DIV    = 2000;         // 40us
    defparam dut.u_buzzer_ctrl.AUD_OFF_DIV   = 4000;         // 80us
    defparam dut.u_buzzer_ctrl.AUD_TONE0     = 20;
    defparam dut.u_buzzer_ctrl.AUD_TONE1     = 40;
    defparam dut.u_buzzer_ctrl.AUD_TONE2     = 80;

    defparam dut.u_led_ctrl.BREATHE_DIV    = 200;            // 4us/级
    defparam dut.u_led_ctrl.CHASE_STEP_DIV = 200;            // 4us/步
    defparam dut.u_led_ctrl.SCAN_STEP_DIV  = 100;            // 2us/步
    defparam dut.u_led_ctrl.HB_PERIOD_DIV  = 2000;           // 40us 周期
    defparam dut.u_led_ctrl.ACT_MS_DIV     = 300;            // 6us 闪亮

    defparam dut.u_input_ctrl.SAMPLE_DIV   = 100;            // 2us/行，8us 全矩阵
    defparam dut.u_input_ctrl.DEBOUNCE_CNT = 5;              // 5 x 8us = 40us
    defparam dut.u_input_ctrl.SNAP_PERIOD  = 100_000;        // 2ms 心跳
    defparam dut.u_input_ctrl.SCENE_SLOT   = 2000;           // 40us 步进
    defparam dut.u_input_ctrl.WALK_LEDS    = 12;
    defparam dut.u_input_ctrl.TONE_ON_DIV  = 2000;
    defparam dut.u_input_ctrl.TONE_OFF_DIV = 4000;

    //------------------------------------------------------------------
    // 层次引用：观测 DUT 内部状态
    //------------------------------------------------------------------
    wire        alarm_on    = dut.alarm_on;
    wire [1:0]  state_code  = dut.state_code;
    wire        cmd_valid   = dut.cmd_valid;
    wire        cmd_error   = dut.cmd_error;
    wire        disp_alarm  = dut.disp_alarm;
    wire        beat_w      = dut.beep_beat;
    wire        ever_linked = dut.ever_linked;
    wire        audio_mute  = dut.audio_mute_cmd;
    wire [3:0]  chase_pos   = dut.u_led_ctrl.chase_pos;
    wire [3:0]  walk_idx_w  = dut.u_input_ctrl.walk_idx;

    //------------------------------------------------------------------
    // 键盘电气模型：行由 DUT 驱动低；按下 = 行列短接（列拉低）
    //------------------------------------------------------------------
    wire [1:0] cur_row = dut.u_input_ctrl.row_idx;
    reg [3:0]  col_low;
    integer    kk;
    always @(*) begin
        col_low = 4'b0000;
        for (kk = 0; kk < 16; kk = kk + 1) begin
            if (pressed[kk] && ((kk >> 2) == cur_row)) begin
                col_low[kk % 4] = 1'b1;
            end
        end
    end
    assign key_col = ~col_low;    // 未按下 = 上拉高

    // 按键任务：按住 200us（远大于 40us 消抖）后释放
    task press_key;
        input [3:0] pos;
        begin
            pressed[pos] = 1'b1;
            #200000;
            pressed[pos] = 1'b0;
            #200000;
        end
    endtask

    //------------------------------------------------------------------
    // 上行帧捕获：用 uart_rx 收 txd，按 8 字节滑动窗口解帧
    //------------------------------------------------------------------
    wire       up_byte_valid;
    wire [7:0] up_byte_data;

    uart_rx u_up_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rxd        (txd),
        .byte_data  (up_byte_data),
        .byte_valid (up_byte_valid)
    );

    reg [63:0]  up_shift;
    wire [63:0] uw = {up_shift[55:0], up_byte_data};
    wire up_frame = (uw[63:56] == 8'hAA) && (uw[55:48] == 8'h55) &&
                    (uw[15:8]  == (uw[47:40] ^ uw[39:32] ^ uw[31:24] ^ uw[23:16])) &&
                    (uw[7:0]   == 8'h0D);

    reg       saw_snap;
    reg       snap_demo, snap_mute;
    reg [2:0] snap_scene;
    reg       saw_key;
    reg [3:0] key_captured;

    //------------------------------------------------------------------
    // 脉冲/形态捕捉标志
    //------------------------------------------------------------------
    reg saw_valid, saw_error;
    reg saw_buzzer_hi, saw_buzzer_lo;
    reg buzzer_d;
    integer buzzer_edges = 0;
    integer fast_cnt, slow_cnt, aud0_cnt, aud1_cnt;
    reg post_mute_beep;
    reg disp_alarm_d1, disp_alarm_d2;

    reg [6:0] snap_seg [0:5];
    reg       snap_dp  [0:5];
    integer   si;

    // 词形字形常量（seg[6:0] = gfedcba）
    localparam [6:0] G_0=7'h3F, G_1=7'h06, G_2=7'h5B, G_3=7'h4F, G_4=7'h66,
                     G_5=7'h6D, G_6=7'h7D, G_7=7'h07, G_8=7'h7F, G_9=7'h6F;
    localparam [6:0] G_H=7'h76, G_E=7'h79, G_F=7'h71, G_L=7'h38, G_o=7'h5C,
                     G_r=7'h50, G_u=7'h1C, G_n=7'h54, G_A=7'h77, G_d=7'h5E,
                     G_y=7'h6E, G_G=7'h3D, G_t=7'h78, G_DASH=7'h40;

    reg saw_hello, saw_ready, saw_run, saw_fall;
    reg saw_val_102, saw_val_340, saw_val_330;
    reg saw_dash6, saw_Ld;
    reg saw_glyph8, saw_glyph9;
    reg saw_tone1, saw_tone2;
    reg hello_blink_lo, hello_blink_hi;
    reg pend_blink_lo,  pend_blink_hi;
    reg fall_blink_lo,  fall_blink_hi;

    reg saw_led_allon_alarm, saw_led_beat_sync, saw_led_off_alarm;
    reg saw_idle_bad, saw_idle_any, saw_link_blink;
    reg saw_walk_any, saw_walk_allon;
    reg saw_pend_any, saw_pend_allon;
    reg saw_kitt_min, saw_kitt_max;
    reg saw_walkscene_any, saw_walkscene_move;
    reg [3:0] last_walk_idx;
    reg bad_led_hi;

    //------------------------------------------------------------------
    // 捕捉逻辑
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_valid <= 1'b0;
            saw_error <= 1'b0;
        end else begin
            if (cmd_valid) saw_valid <= 1'b1;
            if (cmd_error) saw_error <= 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_buzzer_hi <= 1'b0;
            saw_buzzer_lo <= 1'b1;
            buzzer_d      <= 1'b0;
        end else begin
            if (buzzer) saw_buzzer_hi <= 1'b1;
            else        saw_buzzer_lo <= 1'b1;
            buzzer_d <= buzzer;
            if (buzzer && !buzzer_d) buzzer_edges = buzzer_edges + 1;
        end
    end

    // POST 期间（消抖完成后）静音必须完全无声
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                     post_mute_beep <= 1'b0;
        else if (dut.post_active && buzzer && dut.u_input_ctrl.mute_local)
                                        post_mute_beep <= 1'b1;
    end

    // disp_alarm 两拍延迟：LED 寄存器滞后于仲裁组合逻辑，避开撤销沿误采样
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_alarm_d1 <= 1'b0;
            disp_alarm_d2 <= 1'b0;
        end else begin
            disp_alarm_d1 <= disp_alarm;
            disp_alarm_d2 <= disp_alarm_d1;
        end
    end

    // 调试：键事件与场景/合成值变化
    always @(posedge clk) begin
        if (dut.u_input_ctrl.key_evt)
            $display("DBG %0t KEY code=%0d en=%b scene=%0d mag=%0d",
                     $time, dut.u_input_ctrl.key_code,
                     dut.demo_en, dut.demo_scene, dut.demo_mag);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (si = 0; si < 6; si = si + 1) begin
                snap_seg[si] <= 7'h00;
                snap_dp[si]  <= 1'b0;
            end
        end else if (seg[6:0] != 7'h00) begin
            case (an)
                6'b111110: begin snap_seg[0] <= seg[6:0]; snap_dp[0] <= seg[7]; end
                6'b111101: begin snap_seg[1] <= seg[6:0]; snap_dp[1] <= seg[7]; end
                6'b111011: begin snap_seg[2] <= seg[6:0]; snap_dp[2] <= seg[7]; end
                6'b110111: begin snap_seg[3] <= seg[6:0]; snap_dp[3] <= seg[7]; end
                6'b101111: begin snap_seg[4] <= seg[6:0]; snap_dp[4] <= seg[7]; end
                6'b011111: begin snap_seg[5] <= seg[6:0]; snap_dp[5] <= seg[7]; end
                default: ;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_hello <= 0; saw_ready <= 0; saw_run <= 0; saw_fall <= 0;
            saw_val_102 <= 0; saw_val_340 <= 0; saw_val_330 <= 0;
            saw_dash6 <= 0; saw_Ld <= 0;
            saw_glyph8 <= 0; saw_glyph9 <= 0;
            saw_tone1 <= 0; saw_tone2 <= 0;
        end else begin
            if (snap_seg[5]==G_H && snap_seg[4]==G_E && snap_seg[3]==G_L &&
                snap_seg[2]==G_L && snap_seg[1]==G_o) saw_hello <= 1;
            if (snap_seg[5]==G_r && snap_seg[4]==G_E && snap_seg[3]==G_A &&
                snap_seg[2]==G_d && snap_seg[1]==G_y) saw_ready <= 1;
            if (snap_seg[5]==G_r && snap_seg[4]==G_u && snap_seg[3]==G_n)
                saw_run <= 1;
            if (snap_seg[5]==G_F && snap_seg[4]==G_A && snap_seg[3]==G_L &&
                snap_seg[2]==G_L) saw_fall <= 1;
            // "  1.02G"：an3='1'+dp, an2='0', an1='2', an0=G
            if (snap_seg[3]==G_1 && snap_dp[3] && snap_seg[2]==G_0 &&
                snap_seg[1]==G_2 && snap_seg[0]==G_G) saw_val_102 <= 1;
            // "  3.40G"
            if (snap_seg[3]==G_3 && snap_dp[3] && snap_seg[2]==G_4 &&
                snap_seg[1]==G_0 && snap_seg[0]==G_G) saw_val_340 <= 1;
            // "  3.30G"
            if (snap_seg[3]==G_3 && snap_dp[3] && snap_seg[2]==G_3 &&
                snap_seg[1]==G_0 && snap_seg[0]==G_G) saw_val_330 <= 1;
            // 六位横杠
            if (snap_seg[5]==G_DASH && snap_seg[4]==G_DASH && snap_seg[3]==G_DASH &&
                snap_seg[2]==G_DASH && snap_seg[1]==G_DASH && snap_seg[0]==G_DASH)
                saw_dash6 <= 1;
            // 走灯 Ld
            if (snap_seg[5]==G_L && snap_seg[4]==G_d) saw_Ld <= 1;
            // 段码自检：全六位同字形（idx8 dp=0 / idx9 dp=1）
            if (snap_seg[5]==G_8 && snap_seg[4]==G_8 && snap_seg[3]==G_8 &&
                snap_seg[2]==G_8 && snap_seg[1]==G_8 && snap_seg[0]==G_8 &&
                !snap_dp[5] && !snap_dp[4] && !snap_dp[3] &&
                !snap_dp[2] && !snap_dp[1] && !snap_dp[0]) saw_glyph8 <= 1;
            if (snap_seg[5]==G_9 && snap_seg[4]==G_9 && snap_seg[3]==G_9 &&
                snap_seg[2]==G_9 && snap_seg[1]==G_9 && snap_seg[0]==G_9 &&
                snap_dp[5] && snap_dp[4] && snap_dp[3] &&
                snap_dp[2] && snap_dp[1] && snap_dp[0]) saw_glyph9 <= 1;
            // 音调显示 t1 / t2
            if (snap_seg[5]==G_t && snap_seg[4]==G_1) saw_tone1 <= 1;
            if (snap_seg[5]==G_t && snap_seg[4]==G_2) saw_tone2 <= 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hello_blink_lo <= 0; hello_blink_hi <= 0;
            pend_blink_lo  <= 0; pend_blink_hi  <= 0;
            fall_blink_lo  <= 0; fall_blink_hi  <= 0;
        end else begin
            if (!dut.u_seg_display.link_blink_on) hello_blink_lo <= 1;
            else                                   hello_blink_hi <= 1;
            if (!dut.u_seg_display.pend_blink_on) pend_blink_lo <= 1;
            else                                   pend_blink_hi <= 1;
            if (!dut.u_seg_display.blink_on) fall_blink_lo <= 1;
            else                              fall_blink_hi <= 1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_led_allon_alarm <= 0; saw_led_beat_sync <= 0; saw_led_off_alarm <= 0;
            saw_idle_bad <= 0; saw_idle_any <= 0; saw_link_blink <= 0;
            saw_walk_any <= 0; saw_walk_allon <= 0;
            saw_pend_any <= 0; saw_pend_allon <= 0;
            saw_kitt_min <= 0; saw_kitt_max <= 0;
            saw_walkscene_any <= 0; saw_walkscene_move <= 0;
            last_walk_idx <= 4'd0;
            bad_led_hi <= 0;
        end else begin
            // 禁用组恒灭不变式（全程）
            if (led[15:12] != 4'h0) bad_led_hi <= 1;

            // 告警：12 颗随节拍爆闪（用延迟两拍的 disp_alarm 避开撤销沿）
            if (disp_alarm_d2) begin
                if (led[11:0] == 12'hFFF)           saw_led_allon_alarm <= 1;
                if (beat_w && led[11:0]==12'hFFF)   saw_led_beat_sync  <= 1;
                if (!beat_w && led[11:0]==12'h000)  saw_led_off_alarm  <= 1;
            end

            // 静止：呼吸灯（led[0]）+ 链路灯（led[11]），其余不得亮
            if (!disp_alarm_d2 && (state_code == 2'd0)) begin
                if ((led & 16'hF7FE) != 16'h0000) saw_idle_bad <= 1;
                if (led[0])                       saw_idle_any   <= 1;
                if (led[11])                      saw_link_blink <= 1;
            end

            // 走动：流水（12 颗，不得全亮）
            if (!disp_alarm_d2 && (state_code == 2'd1)) begin
                if (led[11:0] != 12'h000) saw_walk_any   <= 1;
                if (led[11:0] == 12'hFFF) saw_walk_allon <= 1;
            end

            // 待确认：KITT 扫描 0..11
            if (!disp_alarm_d2 && (state_code == 2'd2)) begin
                if (led[11:0] != 12'h000) saw_pend_any   <= 1;
                if (led[11:0] == 12'hFFF) saw_pend_allon <= 1;
                if (dut.u_led_ctrl.kitt_pos <= 4'd1)  saw_kitt_min <= 1;
                if (dut.u_led_ctrl.kitt_pos >= 4'd11) saw_kitt_max <= 1;
            end

            // 演示走灯：单灯推进
            if (dut.demo_owned && (dut.demo_scene == 3'd5)) begin
                if (led[11:0] != 12'h000) saw_walkscene_any <= 1;
                if (walk_idx_w != last_walk_idx) begin
                    saw_walkscene_move <= 1;
                    last_walk_idx      <= walk_idx_w;
                end
            end
        end
    end

    // 上行帧解析
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            up_shift     <= 64'h0;
            saw_snap     <= 1'b0;
            snap_demo    <= 1'b0;
            snap_mute    <= 1'b0;
            snap_scene   <= 3'd0;
            saw_key      <= 1'b0;
            key_captured <= 4'd0;
        end else if (up_byte_valid) begin
            up_shift <= {up_shift[55:0], up_byte_data};
            if (up_frame) begin
                if (uw[47:40] == 8'h80) begin
                    saw_snap   <= 1'b1;
                    snap_demo  <= uw[24];        // p2[0]
                    snap_mute  <= uw[25];        // p2[1]
                    snap_scene <= uw[27:25];     // p2[4:2]
                end else if (uw[47:40] == 8'h81) begin
                    saw_key      <= 1'b1;
                    key_captured <= uw[27:24];   // p2[3:0] = byte4 低半字节
                end
            end
        end
    end

    //------------------------------------------------------------------
    // UART 发送任务（下行）
    //------------------------------------------------------------------
    localparam BIT_PERIOD = 8681;      // ns，115200bps

    task send_uart_byte;
        input [7:0] data;
        integer i;
        begin
            rxd = 1'b0;
            #(BIT_PERIOD);
            for (i = 0; i < 8; i = i + 1) begin
                rxd = data[i];
                #(BIT_PERIOD);
            end
            rxd = 1'b1;
            #(BIT_PERIOD);
        end
    endtask

    task send_cmd_frame;
        input [7:0] cmd;
        input [7:0] p1;
        input [7:0] p2lo;
        input [7:0] p2hi;
        input [7:0] xor_override;
        reg [7:0] x;
        begin
            x = (xor_override == 8'h00) ? (cmd ^ p1 ^ p2lo ^ p2hi) : xor_override;
            send_uart_byte(8'hAA);
            send_uart_byte(8'h55);
            send_uart_byte(cmd);
            send_uart_byte(p1);
            send_uart_byte(p2lo);
            send_uart_byte(p2hi);
            send_uart_byte(x);
            send_uart_byte(8'h0D);
            #50000;
        end
    endtask

    //------------------------------------------------------------------
    // 断言辅助
    //------------------------------------------------------------------
    integer errors;

    task check;
        input [255:0] msg;
        input         cond;
        begin
            if (cond) $display("TB:   [PASS] %0s", msg);
            else begin
                $display("TB:   [FAIL] %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    task clear_flags;
        begin
            saw_valid = 0; saw_error = 0;
            saw_buzzer_hi = 0; saw_buzzer_lo = 1;
            saw_snap = 0; snap_demo = 0; snap_mute = 0; snap_scene = 0;
            saw_key = 0; key_captured = 0;
            saw_hello = 0; saw_ready = 0; saw_run = 0; saw_fall = 0;
            saw_val_102 = 0; saw_val_340 = 0; saw_val_330 = 0;
            saw_dash6 = 0; saw_Ld = 0;
            saw_glyph8 = 0; saw_glyph9 = 0;
            saw_tone1 = 0; saw_tone2 = 0;
            saw_led_allon_alarm = 0; saw_led_beat_sync = 0; saw_led_off_alarm = 0;
            saw_idle_bad = 0; saw_idle_any = 0; saw_link_blink = 0;
            saw_walk_any = 0; saw_walk_allon = 0;
            saw_pend_any = 0; saw_pend_allon = 0;
            saw_kitt_min = 0; saw_kitt_max = 0;
            saw_walkscene_any = 0; saw_walkscene_move = 0;
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
    reg [3:0] pos1, pos2;

    initial begin
        errors = 0;
        $display("TB: ============================================");
        $display("TB: coprocessor top-level simulation start");
        $display("TB: ============================================");

        // 复位前：静音拨码 ON（测 POST 静音）
        sw_mute = 1'b1;
        sw_demo = 1'b0;
        pressed = 16'h0;
        rst_n = 1'b0;
        #100;
        rst_n = 1'b1;

        //--------------------------------------------------------------
        // a0) POST（静音下）
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a0 POST with mute on");
        #20000;
        check("a0: LED strobes all-on during POST", saw_led_allon_alarm == 1);
        check("a0: LED all-on synced to beep beat", saw_led_beat_sync == 1);
        check("a0: internal alarm_on still 0",      alarm_on == 1'b0);
        check("a0: FALL shown during POST",         saw_fall == 1'b1);
        // 40us 后静音消抖完成，POST 余下时间内必须完全无声
        #40000;
        check("a0: buzzer silent while muted (post-debounce)", buzzer == 1'b0);
        #60000;
        check("a0: no beep after mute settled in POST", post_mute_beep == 1'b0);

        // 等 POST 结束并解除静音
        #250000;
        sw_mute = 1'b0;

        //--------------------------------------------------------------
        // a) 无链路：HELLO 慢闪
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a no-link standby");
        clear_flags;
        #300000;
        check("a: HELLO word shown",            saw_hello == 1);
        check("a: slow blink both phases",      hello_blink_lo && hello_blink_hi);
        check("a: link LED slow blink seen",    saw_link_blink == 1);
        check("a: ever_linked == 0",            ever_linked == 1'b0);
        check("a: buzzer quiet",                buzzer == 1'b0);
        check("a: idle effect only led0/led11", saw_idle_bad == 0);
        check("a: breathing visible (led0)",    saw_idle_any == 1);

        //--------------------------------------------------------------
        // a1) 首帧同步 -> rEAdy
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a1 first frame sync");
        clear_flags;
        send_cmd_frame(8'h10, 8'h00, 8'hFF, 8'hFF, 8'h00);
        #300000;
        check("a1: cmd_valid seen",   saw_valid == 1);
        check("a1: ever_linked == 1", ever_linked == 1'b1);
        check("a1: rEAdy shown",      saw_ready == 1);
        check("a1: no cmd_error",     saw_error == 1'b0);

        //--------------------------------------------------------------
        // a2) 待确认读数 "  1.02G"
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a2 pending value 1.02G");
        clear_flags;
        send_cmd_frame(8'h10, 8'h02, 8'h66, 8'h00, 8'h00);
        #300000;
        check("a2: state_code == 2",    state_code == 2'd2);
        check("a2: value 1.02G shown",  saw_val_102 == 1);
        check("a2: pending blink both", pend_blink_lo && pend_blink_hi);
        check("a2: alarm_on == 0",      alarm_on == 1'b0);

        //--------------------------------------------------------------
        // a3) 走动 run + 流水
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a3 walk run + chase");
        clear_flags;
        send_cmd_frame(8'h10, 8'h01, 8'h00, 8'h00, 8'h00);
        #300000;
        check("a3: run shown",          saw_run == 1);
        check("a3: chase visible",      saw_walk_any == 1);
        check("a3: chase never all-on", saw_walk_allon == 0);
        pos1 = chase_pos;
        #100000;
        pos2 = chase_pos;
        check("a3: chase position advances", pos1 != pos2);

        //--------------------------------------------------------------
        // a4) 待确认读数 "  3.40G"
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a4 pending value 3.40G");
        clear_flags;
        send_cmd_frame(8'h10, 8'h02, 8'h54, 8'h01, 8'h00);
        #300000;
        check("a4: value 3.40G shown", saw_val_340 == 1);

        //--------------------------------------------------------------
        // a5) 遥测老化 -> "------"
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a5 telemetry aging");
        clear_flags;
        #3_200_000;
        check("a5: readout aged to ------", saw_dash6 == 1);

        //--------------------------------------------------------------
        // b) ALARM
        //--------------------------------------------------------------
        $display("TB: SCENARIO_b ALARM");
        clear_flags;
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        #1_000_000;
        check("b: cmd_valid seen",          saw_valid == 1);
        check("b: alarm_on == 1",           alarm_on == 1'b1);
        check("b: FALL shown",              saw_fall == 1);
        check("b: fall blink both phases",  fall_blink_lo && fall_blink_hi);
        check("b: value NOT shown in alarm",
              (saw_val_102 || saw_val_340 || saw_val_330) == 1'b0);
        check("b: LED strobes all-on",      saw_led_allon_alarm == 1);
        check("b: LED off during gap",      saw_led_off_alarm == 1);
        check("b: LED synced to beat",      saw_led_beat_sync == 1);
        check("b: buzzer carrier toggling", buzzer_edges > 50);

        //--------------------------------------------------------------
        // c) CLEAR
        //--------------------------------------------------------------
        $display("TB: SCENARIO_c CLEAR");
        clear_flags;
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);
        #300000;
        check("c: alarm_on == 0",     alarm_on == 1'b0);
        check("c: buzzer silent",     buzzer == 1'b0);
        check("c: pending dashes",    saw_dash6 == 1);
        check("c: LED back to KITT",  saw_pend_any == 1);
        check("c: KITT not all-on",   saw_pend_allon == 0);
        check("c: KITT reaches low",  saw_kitt_min == 1);
        check("c: KITT reaches high", saw_kitt_max == 1);

        //--------------------------------------------------------------
        // d) 负测试：坏校验
        //--------------------------------------------------------------
        $display("TB: SCENARIO_d bad checksum");
        clear_flags;
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h5A);
        #300000;
        check("d: cmd_error seen",     saw_error == 1);
        check("d: cmd_valid NOT seen", saw_valid == 0);
        check("d: alarm unchanged",    alarm_on == 1'b0);
        check("d: state unchanged",    state_code == 2'd2);

        //--------------------------------------------------------------
        // e) 负测试：未知命令
        //--------------------------------------------------------------
        $display("TB: SCENARIO_e unknown cmd");
        clear_flags;
        send_cmd_frame(8'h77, 8'h00, 8'h00, 8'h00, 8'h00);
        #300000;
        check("e: cmd_valid NOT seen", saw_valid == 0);
        check("e: cmd_error NOT seen", saw_error == 0);
        check("e: alarm unchanged",    alarm_on == 1'b0);

        //--------------------------------------------------------------
        // f) 重同步
        //--------------------------------------------------------------
        $display("TB: SCENARIO_f resync after garbage");
        clear_flags;
        send_uart_byte(8'h12);
        send_uart_byte(8'hAA);
        send_uart_byte(8'h99);
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        check("f: cmd_valid seen", saw_valid == 1);
        check("f: alarm_on == 1",  alarm_on == 1'b1);
        check("f: no cmd_error",   saw_error == 0);
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);   // 清告警
        #200000;

        //--------------------------------------------------------------
        // g) SET_AUDIO
        //--------------------------------------------------------------
        $display("TB: SCENARIO_g SET_AUDIO");
        clear_flags;
        send_cmd_frame(8'h03, 8'h01, 8'h00, 8'h00, 8'h00);   // mute=1
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);   // ALARM
        #500000;
        check("g: audio_mute latched",            audio_mute == 1'b1);
        check("g: buzzer silent in muted alarm",  buzzer == 1'b0);
        check("g: LED still strobes",             saw_led_allon_alarm == 1);
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);   // CLEAR

        clear_flags;
        send_cmd_frame(8'h03, 8'h00, 8'h00, 8'h00, 8'h00);   // unmute
        #200000;
        check("g: unmuted", audio_mute == 1'b0);

        // 运行期改音调：ALARM 下载波 6250，边沿计数应骤降
        clear_flags;
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);   // ALARM（默认载波）
        #200000;
        buzzer_edges = 0;
        #200000;
        fast_cnt = buzzer_edges;
        send_cmd_frame(8'h03, 8'h00, 8'h62, 8'h18, 8'h00);   // p2=0x1862=6250
        buzzer_edges = 0;
        #200000;
        slow_cnt = buzzer_edges;
        check("g: default carrier fast",      fast_cnt > 100);
        check("g: tone 6250 much slower",     (slow_cnt * 10) < fast_cnt);
        send_cmd_frame(8'h03, 8'h00, 8'h00, 8'h00, 8'h00);   // 恢复默认
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);   // CLEAR
        #200000;

        // 待确认提示音
        clear_flags;
        send_cmd_frame(8'h03, 8'h02, 8'h00, 8'h00, 8'h00);   // pend_beep=1
        send_cmd_frame(8'h10, 8'h02, 8'h54, 8'h01, 8'h00);   // state 2
        #700000;
        check("g: pend beep beeps",    saw_buzzer_hi == 1);
        check("g: pend beep has gaps", saw_buzzer_lo == 1);
        check("g: alarm still off",    alarm_on == 1'b0);
        send_cmd_frame(8'h03, 8'h00, 8'h00, 8'h00, 8'h00);   // 全关
        #200000;

        //--------------------------------------------------------------
        // h) 演示模式
        //--------------------------------------------------------------
        $display("TB: SCENARIO_h demo mode");
        clear_flags;
        sw_demo = 1'b1;
        #1_200_000;
        check("h: snapshot frame with demo=1", saw_snap == 1 && snap_demo == 1);
        check("h: no key frame yet",           saw_key == 0);

        // KEY4 -> 场景 4 告警演示（键编号 = 丝印 KEY0~15）
        clear_flags;
        press_key(4);
        #900000;
        check("h: key frame KEY4 uplinked", saw_key == 1 && key_captured == 4);
        check("h: demo alarm latched",      disp_alarm == 1'b1);
        check("h: FALL shown in demo",      saw_fall == 1);
        check("h: demo strobes LED",        saw_led_allon_alarm == 1);

        // KEY9 调值 340->330；KEY3 场景 3 显示合成值
        clear_flags;
        press_key(9);
        #900000;
        check("h: key KEY9 uplinked", saw_key == 1 && key_captured == 9);
        press_key(3);
        #900000;
        check("h: scene3 demo value 3.30G", saw_val_330 == 1);
        // KEY10 调值 330->340（pos = 15-10 = 5）
        press_key(10);
        #900000;
        check("h: scene3 demo value 3.40G", saw_val_340 == 1);

        // KEY5 走灯（pos = 15-5 = 10）
        clear_flags;
        press_key(5);
        #900000;
        check("h: walk scene Ld shown",  saw_Ld == 1);
        check("h: walk single LED seen", saw_walkscene_any == 1);
        check("h: walk position moves",  saw_walkscene_move == 1);

        // KEY6 段码自检 -> 音调试听
        clear_flags;
        press_key(6);
        #2_500_000;
        check("h: glyph all-six 8 seen",  saw_glyph8 == 1);
        check("h: glyph all-six 9 seen",  saw_glyph9 == 1);
        check("h: tone phase t1 shown",   saw_tone1 == 1);
        check("h: tone audition beeping", saw_buzzer_hi == 1);

        // KEY10 换音调档 -> t2（pos = 5）
        clear_flags;
        buzzer_edges = 0;
        #480000;
        aud0_cnt = buzzer_edges;
        press_key(10);
        #900000;
        check("h: tone t2 shown", saw_tone2 == 1);
        buzzer_edges = 0;
        #480000;
        aud1_cnt = buzzer_edges;
        check("h: audition t0 faster than t1",
              (aud0_cnt > 100) && (aud0_cnt > aud1_cnt + 50));

        // 真实 ALARM 突破演示场景 1
        clear_flags;
        press_key(1);             // KEY1 -> 场景 1 静止（pos=14）
        #900000;
        check("h: scene1 rEAdy shown", saw_ready == 1);
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        #400000;
        check("h: real ALARM breaks demo", disp_alarm == 1'b1 && saw_fall == 1);
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);
        #400000;
        check("h: back to scene1 rEAdy", saw_ready == 1 && disp_alarm == 1'b0);

        // 退出演示
        clear_flags;
        sw_demo = 1'b0;
        #1_200_000;
        check("h: exit snapshot demo=0", saw_snap == 1 && snap_demo == 0);

        //--------------------------------------------------------------
        // i) 心跳快照
        //--------------------------------------------------------------
        $display("TB: SCENARIO_i heartbeat snapshot");
        clear_flags;
        #3_000_000;
        check("i: heartbeat snapshot seen", saw_snap == 1);

        //--------------------------------------------------------------
        // 收尾
        //--------------------------------------------------------------
        #200000;
        $display("TB: ============================================");
        check("INV: led[15:12] always dark (LED12-16 banned)", bad_led_hi == 0);
        if (errors == 0) begin
            $display("TB: ALL CHECKS PASSED");
        end else begin
            $display("TB: %0d CHECK(S) FAILED", errors);
            $fatal(1, "TB: simulation failed");
        end
        $display("TB: ============================================");
        $finish;
    end

endmodule
