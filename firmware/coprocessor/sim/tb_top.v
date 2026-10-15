`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 文件名称 : tb_top.v
// 功能描述 : coprocessor 顶层 testbench。无头仿真（xvlog/xelab/xsim）用，
//            不进 Vivado 工程源文件集，由 sim/run_sim.ps1 调用。
//
// 测试场景 :
//   a) 上电默认      : alarm_on=0, state_code=0
//   b) SET_STATE(1)  : state_code 变 1，alarm_on 仍 0
//   c) SET_STATE(2)  : state_code 变 2
//   d) ALARM         : alarm_on 拉高，数码管出现 FALL-- 字形，蜂鸣器断续翻转
//   e) CLEAR         : alarm_on 拉低，蜂鸣器静默
//   f) 负测试-坏校验 : cmd_error 脉冲，alarm_on / state_code 不变
//   g) 负测试-未知命令: 校验正确但 cmd 未知 -> 无 cmd_valid，状态不变
//   h) 重同步测试    : 3 个垃圾字节后紧跟合法 ALARM 帧 -> 仍能正确解码
//
// 参数覆盖说明 :
//   为缩短仿真时间，用 defparam 把 seg_display 的扫描/闪烁分频与
//   buzzer_ctrl 的蜂鸣半周期改小。综合时使用模块默认值（见各模块 parameter）。
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
    defparam dut.u_buzzer_ctrl.BEEP_HALF_DIV  = 500;     // 10us 半周期

    //------------------------------------------------------------------
    // 层次引用：观测 DUT 内部状态
    //------------------------------------------------------------------
    wire       alarm_on   = dut.alarm_on;
    wire [1:0] state_code = dut.state_code;
    wire       cmd_valid  = dut.cmd_valid;
    wire       cmd_error  = dut.cmd_error;

    //------------------------------------------------------------------
    // 脉冲捕捉器：只要出现过就置位，避免脆弱的边沿等待
    //------------------------------------------------------------------
    reg saw_valid;
    reg saw_error;
    reg saw_buzzer_hi;
    reg saw_buzzer_lo;
    reg saw_seg_F, saw_seg_A, saw_seg_L;

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

    // 数码管字形捕捉：an 低有效，digit_idx=5 -> an[5]=0 -> 6'b011111
    //   F: seg[6:0]=7'h71  出现在 an=6'b011111 (最左位)
    //   A: seg[6:0]=7'h77  出现在 an=6'b101111
    //   L: seg[6:0]=7'h38  出现在 an=6'b110111 或 6'b111011
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_seg_F <= 1'b0;
            saw_seg_A <= 1'b0;
            saw_seg_L <= 1'b0;
        end else begin
            if ((an == 6'b011111) && (seg[6:0] == 7'h71)) saw_seg_F <= 1'b1;
            if ((an == 6'b101111) && (seg[6:0] == 7'h77)) saw_seg_A <= 1'b1;
            if ((an == 6'b110111 || an == 6'b111011) &&
                (seg[6:0] == 7'h38))                      saw_seg_L <= 1'b1;
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
            saw_valid     = 1'b0;
            saw_error     = 1'b0;
            saw_buzzer_hi = 1'b0;
            saw_buzzer_lo = 1'b0;
            saw_seg_F     = 1'b0;
            saw_seg_A     = 1'b0;
            saw_seg_L     = 1'b0;
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
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
        // 场景 a0) 上电自检（POST）：自检期内强制告警显示
        //   POST_CYCLES=5000 -> 自检持续 100us
        //--------------------------------------------------------------
        $display("TB: SCENARIO_a0 power-on-self-test");
        #20000;   // 上电 20us，处于自检期（0~100us）
        check("a0: LED all on during POST", led == 16'hFFFF);
        check("a0: buzzer active during POST", saw_buzzer_hi == 1'b1);
        check("a0: internal alarm_on still 0", alarm_on == 1'b0);

        // 等自检结束（复位后 100us，再留 150us 裕量到 250us）
        #250000;
        $display("TB: SCENARIO_a default-after-POST");
        check("a: LED back to state 0",  led        == {14'h0000, 2'b00});
        check("a: buzzer quiet",         buzzer     == 1'b0);
        check("a: alarm_on == 0",        alarm_on   == 1'b0);
        check("a: state_code == 0",      state_code == 2'd0);
        clear_flags;   // 清掉自检期抓到的蜂鸣器标志，后续场景从零计

        //--------------------------------------------------------------
        // 场景 b) SET_STATE(1)
        //--------------------------------------------------------------
        $display("TB: SCENARIO_b SET_STATE(1)");
        clear_flags;
        send_cmd_frame(8'h10, 8'h01, 8'h00, 8'h00, 8'h00);
        check("b: cmd_valid seen",   saw_valid   == 1'b1);
        check("b: state_code == 1",  state_code  == 2'd1);
        check("b: alarm_on still 0", alarm_on    == 1'b0);
        check("b: no cmd_error",     saw_error   == 1'b0);

        //--------------------------------------------------------------
        // 场景 c) SET_STATE(2)
        //--------------------------------------------------------------
        $display("TB: SCENARIO_c SET_STATE(2)");
        clear_flags;
        send_cmd_frame(8'h10, 8'h02, 8'h00, 8'h00, 8'h00);
        check("c: cmd_valid seen",  saw_valid  == 1'b1);
        check("c: state_code == 2", state_code == 2'd2);

        //--------------------------------------------------------------
        // 场景 d) ALARM
        //--------------------------------------------------------------
        $display("TB: SCENARIO_d ALARM");
        clear_flags;
        send_cmd_frame(8'h01, 8'h00, 8'h00, 8'h00, 8'h00);
        check("d: cmd_valid seen",  saw_valid == 1'b1);
        check("d: alarm_on == 1",   alarm_on  == 1'b1);

        // 观察 400us，覆盖数码管多轮扫描 + 蜂鸣器多次翻转 + 闪烁亮相位
        #400000;
        check("d: LED all on",        led     == 16'hFFFF);
        check("d: buzzer toggles",    saw_buzzer_hi == 1'b1);
        check("d: seg shows F",       saw_seg_F == 1'b1);
        check("d: seg shows A",       saw_seg_A == 1'b1);
        check("d: seg shows L",       saw_seg_L == 1'b1);

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
        check("e: LED back to state", led   == {14'h0000, state_code[1], state_code[0]});

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
