`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 文件名称 : tb_top.v
// 功能描述 : coprocessor 顶层 testbench。无头仿真（xvlog/xelab/xsim）用，
//            不进 Vivado 工程源文件集，由 sim/run_sim.ps1 调用。
//
// 测试场景 :
//   a) 上电默认      : alarm_on=0, state_code=0；LED 呼吸灯（仅 led[0] 可亮）
//                     数码管 "St" + 读数 "----"（尚无遥测）
//   a0) POST         : 上电自检期内强制告警效果，数码管 "AL" + "----"
//   a1) 遥测读数      : SET_STATE 的 p2 携带合加速度（0.01g）-> 显示 "01.02"/"03.40"
//   b) SET_STATE(1)  : state_code 变 1，alarm_on 仍 0；数码管 "Go"，LED 流水
//   c) SET_STATE(2)  : state_code 变 2；数码管 "Pd" 助记符快闪，LED KITT 扫描
//   d) ALARM         : alarm_on 拉高，数码管 "AL" 整屏急闪 + 读数保持，蜂鸣器载波+节拍，
//                      LED 与节拍同步爆闪
//   e) CLEAR         : alarm_on 拉低，蜂鸣器静默，LED 回到状态效果
//   f) 负测试-坏校验 : cmd_error 脉冲，alarm_on / state_code 不变
//   g) 负测试-未知命令: 校验正确但 cmd 未知 -> 无 cmd_valid，状态不变
//   h) 重同步测试    : 3 个垃圾字节后紧跟合法 ALARM 帧 -> 仍能正确解码
//
// 参数覆盖说明 :
//   为缩短仿真时间，用 defparam 把各模块的分频参数改小。综合时使用模块
//   默认值（见各模块 parameter）。LED 采用软 PWM，瞬时值在脉宽内抖动，
//   因此 LED 断言一律用"观察窗口内是否出现过某形态"的累积标志，
//   不对某一时刻的引脚值做精确断言（避免 1/256 概率的假失败）。
//
// UART 位时间 :
//   1/115200 = 8680.56 ns，取 8681 ns（误差 0.005%，十位累计 0.05 位，安全）
//////////////////////////////////////////////////////////////////////////////////

module tb_top;

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    reg rxd   = 1'b1;          // UART 空闲高电平

    always #10 clk = ~clk;     // 50MHz -> 20ns 周期

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    wire        txd;
    wire [15:0] led;
    wire [7:0]  seg;
    wire [5:0]  an;
    wire        buzzer;

    top dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .rxd    (rxd),
        .txd    (txd),
        .led    (led),
        .seg    (seg),
        .an     (an),
        .buzzer (buzzer)
    );

    //------------------------------------------------------------------
    // 参数覆盖：缩短仿真时间
    //------------------------------------------------------------------
    defparam dut.POST_CYCLES                  = 5000;    // 上电自检 100us

    defparam dut.u_seg_display.SCAN_DIV       = 50;      // 1us / 位
    defparam dut.u_seg_display.BLINK_HALF_DIV = 2000;    // 40us 半周期
    defparam dut.u_seg_display.PEND_BLINK_HALF_DIV = 1000;  // 20us -> 快闪
    // 遥测老化阈值：必须显著大于"一帧 UART 传输时间"（8 字节 @115200 约 700us），
    // 否则连续两帧之间就会老化，测出来的不是设计行为。取 2ms，实机默认 3s。
    defparam dut.u_seg_display.TELEM_STALE_DIV     = 100000; // 2ms 遥测老化

    defparam dut.u_buzzer_ctrl.TONE_HALF_DIV  = 20;      // 载波周期 800ns
    defparam dut.u_buzzer_ctrl.BEAT_ON_DIV    = 1500;    // 30us
    defparam dut.u_buzzer_ctrl.BEAT_GAP_DIV   = 1000;    // 20us
    defparam dut.u_buzzer_ctrl.BEAT_REST_DIV  = 4000;    // 80us

    defparam dut.u_led_ctrl.BREATHE_DIV       = 200;     // 4us / 级
    defparam dut.u_led_ctrl.CHASE_STEP_DIV    = 200;     // 4us / 步
    defparam dut.u_led_ctrl.SCAN_STEP_DIV     = 100;     // 2us / 步

    //------------------------------------------------------------------
    // 层次引用：观测 DUT 内部状态
    //------------------------------------------------------------------
    wire       alarm_on   = dut.alarm_on;
    wire [1:0] state_code = dut.state_code;
    wire       cmd_valid  = dut.cmd_valid;
    wire       cmd_error  = dut.cmd_error;
    wire       disp_alarm = dut.disp_alarm;      // 含自检覆盖
    wire       beat_w     = dut.beep_beat;       // 蜂鸣节拍包络
    wire [3:0] kitt_pos   = dut.u_led_ctrl.kitt_pos;

    //------------------------------------------------------------------
    // 脉冲捕捉器：只要出现过就置位，避免脆弱的边沿等待
    //------------------------------------------------------------------
    reg saw_valid;
    reg saw_error;
    reg saw_buzzer_hi;
    reg saw_buzzer_lo;
    // 数码管字形观察标志（新布局：2 位助记符 + 4 位读数 "XX.XX"）
    reg saw_mn_St, saw_mn_Go, saw_mn_Pd, saw_mn_AL;
    reg saw_read_0102;      // 遥测 1.02g -> "01.02"
    reg saw_read_340;       // 遥测 3.40g -> "03.40"
    reg saw_read_dash;      // mag_valid=0 -> "----"
    reg saw_read_stale;     // 曾有有效读数，随后因老化变回 "----"

    // LED 效果形态标志（累积，见文件头说明）
    reg saw_led_allon_alarm;    // 告警中全亮
    reg saw_led_beat_sync;      // 告警中"节拍响=全亮"同时出现
    reg saw_led_off_alarm;      // 告警中"节拍停=全灭"同时出现（证明是爆闪）
    reg saw_idle_hi;            // 静止态出现了 led[0] 以外的灯（应为 0）
    reg saw_idle_any;           // 静止态 led[0] 亮过（呼吸灯可见）
    reg saw_walk_any;           // 走动态 LED 亮过
    reg saw_walk_allon;         // 走动态出现全亮（应为 0）
    reg saw_pend_any;           // 待确认态 LED 亮过
    reg saw_pend_allon;         // 待确认态出现全亮（应为 0）
    reg saw_kitt_min;           // KITT 扫到低位端
    reg saw_kitt_max;           // KITT 扫到高位端
    reg saw_pend_digit_on;      // 待确认助记符亮相位
    reg saw_pend_digit_off;     // 待确认助记符灭相位

    // 蜂鸣器上升沿计数（证明载波在翻转，而非 2Hz 慢通断）
    reg     buzzer_d;
    integer buzzer_edges;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_valid <= 1'b0;
        end else if (cmd_valid) begin
            saw_valid <= 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_error <= 1'b0;
        end else if (cmd_error) begin
            saw_error <= 1'b1;
        end
    end

    // 蜂鸣器：告警期间应出现高低交替
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_buzzer_hi <= 1'b0;
            saw_buzzer_lo <= 1'b1;   // 初始静默为低，记一次
        end else begin
            if (buzzer) saw_buzzer_hi <= 1'b1;
            else        saw_buzzer_lo <= 1'b1;
        end
    end

    // 蜂鸣器上升沿计数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buzzer_d     <= 1'b0;
            buzzer_edges <= 0;
        end else begin
            buzzer_d <= buzzer;
            if (buzzer && !buzzer_d) buzzer_edges <= buzzer_edges + 1;
        end
    end

    // 数码管字形快照：按位选把"最近一次非消隐字形"记入 snap_seg/snap_dp，
    // 这样断言可以直接比对 6 位组合，而不用赌某一时刻的扫描相位。
    //   位选（低有效）：an[5]=6'b011111 ... an[0]=6'b111110
    //   新布局：an[5]an[4] = 状态助记符，an[3..0] = 读数 "XX.XX"（an[2] 带小数点）
    reg [6:0]  snap_seg [0:5];
    reg        snap_dp  [0:5];
    integer    si;

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

    // 助记符 / 读数组合判定（snap 已是"最近一次点亮"的组合）
    localparam [6:0] G_S = 7'h6D, G_t = 7'h78, G_G = 7'h3D, G_o = 7'h5C;
    localparam [6:0] G_P = 7'h73, G_d = 7'h5E, G_A = 7'h77, G_L = 7'h38;
    localparam [6:0] G_0 = 7'h3F, G_1 = 7'h06, G_2 = 7'h5B, G_3 = 7'h4F;
    localparam [6:0] G_4 = 7'h66, G_DASH = 7'h40;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_mn_St    <= 1'b0;
            saw_mn_Go    <= 1'b0;
            saw_mn_Pd    <= 1'b0;
            saw_mn_AL    <= 1'b0;
            saw_read_0102 <= 1'b0;
            saw_read_340  <= 1'b0;
            saw_read_dash <= 1'b0;
            saw_read_stale <= 1'b0;
        end else begin
            if (snap_seg[5] == G_S && snap_seg[4] == G_t) saw_mn_St <= 1'b1;
            if (snap_seg[5] == G_G && snap_seg[4] == G_o) saw_mn_Go <= 1'b1;
            if (snap_seg[5] == G_P && snap_seg[4] == G_d) saw_mn_Pd <= 1'b1;
            if (snap_seg[5] == G_A && snap_seg[4] == G_L) saw_mn_AL <= 1'b1;

            // 1.02 g -> "01.02"（an[2] 的 dp 必须点亮）
            if (snap_seg[3] == G_0 && snap_seg[2] == G_1 && snap_dp[2] &&
                snap_seg[1] == G_0 && snap_seg[0] == G_2) saw_read_0102 <= 1'b1;

            // 3.40 g -> "03.40"
            if (snap_seg[3] == G_0 && snap_seg[2] == G_3 && snap_dp[2] &&
                snap_seg[1] == G_4 && snap_seg[0] == G_0) saw_read_340 <= 1'b1;

            // 无遥测 -> "----"
            if (snap_seg[3] == G_DASH && snap_seg[2] == G_DASH &&
                snap_seg[1] == G_DASH && snap_seg[0] == G_DASH) saw_read_dash <= 1'b1;

            // 老化：先出现过有效读数，之后又变回 "----"（说明是超时而非从未有数据）
            if ((saw_read_0102 || saw_read_340) &&
                snap_seg[3] == G_DASH && snap_seg[2] == G_DASH &&
                snap_seg[1] == G_DASH && snap_seg[0] == G_DASH) saw_read_stale <= 1'b1;
        end
    end

    // LED 形态标志与待确认快闪相位捕捉
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_led_allon_alarm <= 1'b0;
            saw_led_beat_sync   <= 1'b0;
            saw_led_off_alarm   <= 1'b0;
            saw_idle_hi         <= 1'b0;
            saw_idle_any        <= 1'b0;
            saw_walk_any        <= 1'b0;
            saw_walk_allon      <= 1'b0;
            saw_pend_any        <= 1'b0;
            saw_pend_allon      <= 1'b0;
            saw_kitt_min        <= 1'b0;
            saw_kitt_max        <= 1'b0;
            saw_pend_digit_on   <= 1'b0;
            saw_pend_digit_off  <= 1'b0;
        end else begin
            // --- 告警态：声光同步爆闪 ---
            if (disp_alarm) begin
                if (led == 16'hFFFF)                    saw_led_allon_alarm <= 1'b1;
                if (beat_w && (led == 16'hFFFF))        saw_led_beat_sync   <= 1'b1;
                if (!beat_w && (led == 16'h0000))       saw_led_off_alarm   <= 1'b1;
            end

            // --- 静止态：呼吸灯（只允许 led[0]） ---
            if (!disp_alarm && (state_code == 2'd0)) begin
                if ((led & 16'hFFFE) != 16'h0000) saw_idle_hi  <= 1'b1;
                if (led[0])                       saw_idle_any <= 1'b1;
            end

            // --- 走动态：环形流水（不该全亮） ---
            if (!disp_alarm && (state_code == 2'd1)) begin
                if (led != 16'h0000) saw_walk_any   <= 1'b1;
                if (led == 16'hFFFF) saw_walk_allon <= 1'b1;
            end

            // --- 待确认态：KITT 来回扫描（不该全亮） ---
            if (!disp_alarm && (state_code == 2'd2)) begin
                if (led != 16'h0000)   saw_pend_any   <= 1'b1;
                if (led == 16'hFFFF)   saw_pend_allon <= 1'b1;
                if (kitt_pos <= 4'd1)  saw_kitt_min   <= 1'b1;
                if (kitt_pos >= 4'd14) saw_kitt_max   <= 1'b1;
            end

            // --- 待确认助记符快闪：亮/灭两个相位都要抓到（an[5] = "P" 位） ---
            if (!disp_alarm && (state_code == 2'd2) && (an == 6'b011111)) begin
                if (seg[6:0] == 7'h73) saw_pend_digit_on  <= 1'b1;
                if (seg[6:0] == 7'h00) saw_pend_digit_off <= 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // UART 发送任务
    //------------------------------------------------------------------
    localparam BIT_PERIOD = 8681;      // ns，115200bps

    task send_uart_byte;
        input [7:0] data;
        integer i;
        begin
            rxd = 1'b0;                        // 起始位
            #(BIT_PERIOD);
            for (i = 0; i < 8; i = i + 1) begin
                rxd = data[i];                 // LSB 先发
                #(BIT_PERIOD);
            end
            rxd = 1'b1;                        // 停止位
            #(BIT_PERIOD);
        end
    endtask

    // 组帧并发送：AA 55 cmd p1 p2lo p2hi xor 0D
    task send_cmd_frame;
        input [7:0] cmd;
        input [7:0] p1;
        input [7:0] p2lo;
        input [7:0] p2hi;
        input [7:0] xor_override;   // 正常传 0 表示自动算；非 0 用于坏校验测试
        reg   [7:0] x;
        begin
            x = (xor_override == 8'h00) ? (cmd ^ p1 ^ p2lo ^ p2hi)
                                        : xor_override;
            send_uart_byte(8'hAA);
            send_uart_byte(8'h55);
            send_uart_byte(cmd);
            send_uart_byte(p1);
            send_uart_byte(p2lo);
            send_uart_byte(p2hi);
            send_uart_byte(x);
            send_uart_byte(8'h0D);
            #50000;                            // 帧间 50us，保证帧边界清晰
        end
    endtask

    //------------------------------------------------------------------
    // 断言辅助
    //------------------------------------------------------------------
    integer errors;
    integer i;

    task check;
        input [255:0] msg;
        input         cond;
        begin
            if (cond) begin
                $display("TB:   [PASS] %0s", msg);
            end else begin
                $display("TB:   [FAIL] %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    // 清除捕捉标志
    task clear_flags;
        begin
            saw_valid           = 1'b0;
            saw_error           = 1'b0;
            saw_buzzer_hi       = 1'b0;
            saw_buzzer_lo       = 1'b0;
            saw_mn_St           = 1'b0;
            saw_mn_Go           = 1'b0;
            saw_mn_Pd           = 1'b0;
            saw_mn_AL           = 1'b0;
            saw_read_0102       = 1'b0;
            saw_read_340        = 1'b0;
            saw_read_dash       = 1'b0;
            saw_read_stale      = 1'b0;
            saw_led_allon_alarm = 1'b0;
            saw_led_beat_sync   = 1'b0;
            saw_led_off_alarm   = 1'b0;
            saw_idle_hi         = 1'b0;
            saw_idle_any        = 1'b0;
            saw_walk_any        = 1'b0;
            saw_walk_allon      = 1'b0;
            saw_pend_any        = 1'b0;
            saw_pend_allon      = 1'b0;
            saw_kitt_min        = 1'b0;
            saw_kitt_max        = 1'b0;
            saw_pend_digit_on   = 1'b0;
            saw_pend_digit_off  = 1'b0;
            buzzer_edges        = 0;
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
    reg [3:0] walk_pos1, walk_pos2;

    initial begin
        errors = 0;
        $display("TB: ============================================");
        $display("TB: coprocessor top-level simulation start");
        $display("TB: ============================================");

        // 复位 100ns
        rst_n = 1'b0;
        #100;
        rst_n = 1'b1;

        //--------------------------------------------------------------
        // 场景 a0) 上电自检（POST）：自检期内强制告警效果
        //   POST_CYCLES=5000 -> 自检持续 100us
        //   此时处于节拍第 1 段（响），LED 应为全亮爆闪
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a0 power-on-self-test");
        #20000;   // 上电 20us，处于自检期（0~100us）
        check("a0: LED strobes all-on during POST", saw_led_allon_alarm == 1'b1);
        check("a0: LED all-on synced to beep beat", saw_led_beat_sync   == 1'b1);
        check("a0: buzzer active during POST",      saw_buzzer_hi       == 1'b1);
        check("a0: internal alarm_on still 0",      alarm_on            == 1'b0);

        // 等自检结束（复位后 100us，再留 150us 裕量到 250us）
        #250000;
        $display("TB: SCENARIO_a default-after-POST");
        check("a: alarm_on == 0",    alarm_on   == 1'b0);
        check("a: state_code == 0",  state_code == 2'd0);
        check("a: buzzer quiet",     buzzer     == 1'b0);

        // 观察 200us 的静止态呼吸灯：只允许 led[0] 出现，且确实亮过
        clear_flags;
        #200000;
        check("a: breathing uses only led[0]", saw_idle_hi  == 1'b0);
        check("a: breathing visible (led[0] lit)", saw_idle_any == 1'b1);
        // 尚无遥测帧 -> 助记符 "St" + 读数 "----"
        check("a: seg mnemonic St",        saw_mn_St    == 1'b1);
        check("a: readout dashes when no telemetry", saw_read_dash == 1'b1);

        //--------------------------------------------------------------
        // 场景 a1) 遥测读数：SET_STATE 携带 p2 = 合加速度幅值（0.01g）
        //   完整帧 = AA 55 10 00 66 00 76 0D   (xor = 10^00^66^00 = 76)
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a1 telemetry readout");
        clear_flags;
        send_cmd_frame(8'h10, 8'h00, 8'h66, 8'h00, 8'h00);   // 102 -> 1.02 g
        #200000;
        check("a1: state_code == 0", state_code == 2'd0);
        check("a1: seg mnemonic St", saw_mn_St == 1'b1);
        check("a1: readout shows 01.02",  saw_read_0102 == 1'b1);
        check("a1: no cmd_error",    saw_error == 1'b0);

        clear_flags;
        send_cmd_frame(8'h10, 8'h00, 8'h54, 8'h01, 8'h00);   // 340 -> 3.40 g
        #200000;
        check("a1: readout shows 03.40", saw_read_340 == 1'b1);

        //--------------------------------------------------------------
        // 场景 a2) 遥测老化看门狗
        //   TELEM_STALE_DIV=100000 -> 2ms（须显著大于一帧 UART 的 ~700us）。
        //   数据流停摆（S3 收包任务阻塞在 recvfrom 上、发不出哨兵）时，
        //   FPGA 必须自己把读数切成 "----"，不能一直显示过期值。
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a2 telemetry staleness watchdog");
        // 先等 a1 的最后一帧彻底老化掉（2ms 阈值 + 裕量），再清标志，
        // 避免残留新鲜值干扰
        #3000000;
        clear_flags;
        check("a2: readout aged out before new frame",
              dut.u_seg_display.rd_valid == 1'b0);

        send_cmd_frame(8'h10, 8'h00, 8'h66, 8'h00, 8'h00);   // 102 -> 1.02 g
        #200000;                                             // 帧后 200us：仍新鲜
        check("a2: readout fresh right after telemetry", saw_read_0102 == 1'b1);
        check("a2: not stale yet",                      saw_read_stale == 1'b0);

        #3000000;                                            // 再 3ms -> 远超 2ms
        check("a2: readout ages to ---- when telemetry stops",
              saw_read_stale == 1'b1);

        // 恢复刷新 -> 读数回来
        clear_flags;
        send_cmd_frame(8'h10, 8'h00, 8'h66, 8'h00, 8'h00);
        #200000;
        check("a2: readout recovers after a fresh frame", saw_read_0102 == 1'b1);

        //--------------------------------------------------------------
        // 场景 b) SET_STATE(1) -> WALK：数码管 "Go"，LED 环形流水
        //--------------------------------------------------------------
        $display("TB: SCENARIO_b SET_STATE(1)");
        clear_flags;
        send_cmd_frame(8'h10, 8'h01, 8'h00, 8'h00, 8'h00);
        check("b: cmd_valid seen",   saw_valid   == 1'b1);
        check("b: state_code == 1",  state_code  == 2'd1);
        check("b: alarm_on still 0", alarm_on    == 1'b0);
        check("b: no cmd_error",     saw_error   == 1'b0);

        #200000;   // 观察流水灯
        check("b: seg mnemonic Go",        saw_mn_Go     == 1'b1);
        check("b: LED chase visible",      saw_walk_any  == 1'b1);
        check("b: LED chase never all-on", saw_walk_allon == 1'b0);

        // 流水位置必须在走：4us/步，隔 40us（=10 步）采样必不同
        #100000;
        walk_pos1 = dut.u_led_ctrl.chase_pos;
        #40000;
        walk_pos2 = dut.u_led_ctrl.chase_pos;
        check("b: LED chase position advances", walk_pos1 != walk_pos2);

        //--------------------------------------------------------------
        // 场景 c) SET_STATE(2) -> PENDING：助记符快闪 + LED KITT 扫描
        //--------------------------------------------------------------
        $display("TB: SCENARIO_c SET_STATE(2)");
        clear_flags;
        send_cmd_frame(8'h10, 8'h02, 8'h00, 8'h00, 8'h00);
        check("c: cmd_valid seen",  saw_valid  == 1'b1);
        check("c: state_code == 2", state_code == 2'd2);

        #200000;   // 观察快闪与扫描
        check("c: seg mnemonic Pd",        saw_mn_Pd          == 1'b1);
        check("c: mnemonic lit phase",     saw_pend_digit_on  == 1'b1);
        check("c: mnemonic blink phase",   saw_pend_digit_off == 1'b1);
        check("c: LED scan visible",       saw_pend_any       == 1'b1);
        check("c: LED scan never all-on",  saw_pend_allon     == 1'b0);
        check("c: KITT reaches low end",   saw_kitt_min       == 1'b1);
        check("c: KITT reaches high end",  saw_kitt_max       == 1'b1);

        //--------------------------------------------------------------
        // 场景 d) ALARM：AL 整屏急闪 + 载波节拍蜂鸣 + 声光同步爆闪
        //--------------------------------------------------------------
        $display("TB: SCENARIO_d ALARM");
        clear_flags;
        // 先把遥测刷新到 3.40g（模拟 S3 报警前的最后一帧周期刷新），再拉告警：
        // 告警只改 alarm_on 与助记符，读数应保持不变
        send_cmd_frame(8'h10, 8'h02, 8'h54, 8'h01, 8'h00);   // 340 -> 3.40 g
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        check("d: cmd_valid seen",  saw_valid == 1'b1);
        check("d: alarm_on == 1",   alarm_on  == 1'b1);

        // 观察 3ms（远超 2ms 遥测老化阈值），覆盖数码管多轮扫描 + 蜂鸣器多个节拍
        #3000000;
        check("d: LED strobes all-on in alarm",  saw_led_allon_alarm == 1'b1);
        check("d: LED off during beep gap",      saw_led_off_alarm   == 1'b1);
        check("d: LED synced to beep beat",      saw_led_beat_sync   == 1'b1);
        check("d: buzzer carrier toggling",      buzzer_edges > 50);
        check("d: buzzer toggles",               saw_buzzer_hi == 1'b1);
        check("d: seg mnemonic AL",              saw_mn_AL == 1'b1);
        check("d: telemetry readout retained",   saw_read_340 == 1'b1);
        // 告警期间老化必须冻结：读数定格为冲击幅值，不能 3s 后变 "----"。
        // 这里观察 3ms（远超 2ms 老化阈值）后 mag_fresh 仍应为 1。
        check("d: readout does not age while alarm latched",
              dut.u_seg_display.mag_fresh == 1'b1);

        //--------------------------------------------------------------
        // 场景 e) CLEAR
        //--------------------------------------------------------------
        $display("TB: SCENARIO_e CLEAR");
        clear_flags;
        send_cmd_frame(8'h02, 8'h00, 8'h00, 8'h00, 8'h00);
        check("e: cmd_valid seen",    saw_valid == 1'b1);
        check("e: alarm_on == 0",     alarm_on  == 1'b0);
        #200000;
        check("e: buzzer silent",     buzzer == 1'b0);
        // state_code 仍为 2 -> LED 应回到 KITT 扫描，而不是全亮
        check("e: LED back to state effect", saw_pend_any   == 1'b1);
        check("e: LED not all-on",           saw_pend_allon == 1'b0);

        //--------------------------------------------------------------
        // 场景 f) 负测试：坏校验和
        //--------------------------------------------------------------
        $display("TB: SCENARIO_f bad-checksum");
        clear_flags;
        // 正确 xor = 01^00^00^00 = 01；这里强制写 0x5A 使其失配
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h5A);
        #200000;
        check("f: cmd_error seen",    saw_error  == 1'b1);
        check("f: cmd_valid NOT seen", saw_valid == 1'b0);
        check("f: alarm_on unchanged", alarm_on  == 1'b0);
        check("f: state_code unchanged", state_code == 2'd2);

        //--------------------------------------------------------------
        // 场景 g) 负测试：未知命令码（校验正确）
        //--------------------------------------------------------------
        $display("TB: SCENARIO_g unknown-cmd");
        clear_flags;
        send_cmd_frame(8'h77, 8'h00, 8'h00, 8'h00, 8'h00);
        #200000;
        check("g: cmd_valid NOT seen",  saw_valid   == 1'b0);
        check("g: cmd_error NOT seen",  saw_error   == 1'b0);
        check("g: alarm_on unchanged",  alarm_on    == 1'b0);
        check("g: state_code unchanged", state_code == 2'd2);

        //--------------------------------------------------------------
        // 场景 h) 重同步：垃圾字节 + 合法 ALARM 帧
        //--------------------------------------------------------------
        $display("TB: SCENARIO_h resync-after-garbage");
        clear_flags;
        send_uart_byte(8'h12);
        send_uart_byte(8'hAA);
        send_uart_byte(8'h99);
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        check("h: cmd_valid seen",   saw_valid == 1'b1);
        check("h: alarm_on == 1",    alarm_on  == 1'b1);
        check("h: no cmd_error",     saw_error == 1'b0);

        //--------------------------------------------------------------
        // 收尾
        //--------------------------------------------------------------
        #100000;
        $display("TB: ============================================");
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
