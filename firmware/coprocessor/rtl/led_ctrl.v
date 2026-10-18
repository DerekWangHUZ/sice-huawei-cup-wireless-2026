`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : led_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : 16 颗 LED 的状态效果引擎。用 8 位软 PWM 做亮度控制，按
//            state_code / alarm_on 呈现四种效果：
//
//   alarm（或上电自检）: 全 16 颗跟随蜂鸣器节拍"声光同步爆闪"——
//                        beat=1 全亮、beat=0 全灭。声音与光同步是演示
//                        现场最直观的告警表达。
//   state 0（IDLE 静止）: led[0] 三角波呼吸灯（亮 -> 暗 -> 亮约 2.6s），
//                        "系统在跑"的柔和存活指示。
//   state 1（WALK 走动）: 单灯带 2 级拖尾的环形流水灯（80ms/步），
//                        把人"正在走动"这件事显示成运动感。
//   state 2（PENDING 待确认): KITT 式来回扫描（50ms/步），速度介于流水与
//                        告警之间 —— 冲击已发生、正在确认的中间态。
//
// 接口说明 :
//   clk         50MHz 系统时钟
//   rst_n       低有效异步复位
//   state_code  S3 判定状态编号 0..3（来自 cmd_decoder）
//   alarm_on    告警标志（自检期间由 top 强制为 1）
//   beat        蜂鸣器节拍包络（来自 buzzer_ctrl），告警时用于声光同步
//   led[15:0]   高电平点亮。led[15]（M1）在本板物理不亮，属板卡个体问题，
//               验收忽略（本模块照常输出，不做特殊处理）。
//
// 参数说明 :
//   PWMW            软 PWM 位宽，默认 8 -> 帧率 50MHz/256 约 195kHz，无频闪。
//   BREATHE_DIV     呼吸步进间隔（clk 数）。256 级 x 250_000 clk = 1.28s 单向，
//                   往返约 2.6s。
//   CHASE_STEP_DIV  流水灯步进间隔。默认 4_000_000 -> 80ms/步。
//   SCAN_STEP_DIV   KITT 扫描步进间隔。默认 2_500_000 -> 50ms/步。
//   仿真时由 tb 用 defparam 覆盖为小值以缩短仿真时间。
//////////////////////////////////////////////////////////////////////////////////

module led_ctrl #(
    parameter PWMW           = 8,
    parameter BREATHE_DIV    = 250000,
    parameter CHASE_STEP_DIV = 4000000,
    parameter SCAN_STEP_DIV  = 2500000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  state_code,
    input  wire        alarm_on,
    input  wire        beat,
    output reg  [15:0] led
);

    //------------------------------------------------------------------
    // 软 PWM 计数器：0..(2^PWMW - 1) 自由计数
    //------------------------------------------------------------------
    reg [PWMW-1:0] pwm_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pwm_cnt <= {PWMW{1'b0}};
        else        pwm_cnt <= pwm_cnt + 1'b1;
    end

    //------------------------------------------------------------------
    // 呼吸灯：三角波亮度，0 -> 255 -> 0 往复
    //------------------------------------------------------------------
    reg [7:0]  breathe_duty;
    reg        breathe_dir;        // 0=上行 1=下行
    reg [31:0] breathe_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            breathe_duty <= 8'd0;
            breathe_dir  <= 1'b0;
            breathe_cnt  <= 32'd0;
        end else if (breathe_cnt == BREATHE_DIV - 1) begin
            breathe_cnt <= 32'd0;
            if (!breathe_dir) begin
                if (breathe_duty == 8'd255) begin
                    breathe_dir  <= 1'b1;
                    breathe_duty <= 8'd254;
                end else begin
                    breathe_duty <= breathe_duty + 8'd1;
                end
            end else begin
                if (breathe_duty == 8'd0) begin
                    breathe_dir  <= 1'b0;
                    breathe_duty <= 8'd1;
                end else begin
                    breathe_duty <= breathe_duty - 8'd1;
                end
            end
        end else begin
            breathe_cnt <= breathe_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // 流水灯位置：0..15 循环；拖尾位置用 4 位回绕减法自然得到
    //------------------------------------------------------------------
    reg [3:0]  chase_pos;
    reg [31:0] chase_cnt;

    wire [3:0] chase_prev1 = chase_pos - 4'd1;
    wire [3:0] chase_prev2 = chase_pos - 4'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chase_pos <= 4'd0;
            chase_cnt <= 32'd0;
        end else if (chase_cnt == CHASE_STEP_DIV - 1) begin
            chase_cnt <= 32'd0;
            chase_pos <= chase_pos + 4'd1;
        end else begin
            chase_cnt <= chase_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // KITT 扫描位置：0 -> 15 -> 0 来回
    //------------------------------------------------------------------
    reg [3:0]  kitt_pos;
    reg        kitt_dir;           // 0=向高位 1=向低位
    reg [31:0] kitt_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kitt_pos <= 4'd0;
            kitt_dir <= 1'b0;
            kitt_cnt <= 32'd0;
        end else if (kitt_cnt == SCAN_STEP_DIV - 1) begin
            kitt_cnt <= 32'd0;
            if (!kitt_dir) begin
                if (kitt_pos == 4'd15) begin
                    kitt_dir <= 1'b1;
                    kitt_pos <= 4'd14;
                end else begin
                    kitt_pos <= kitt_pos + 4'd1;
                end
            end else begin
                if (kitt_pos == 4'd0) begin
                    kitt_dir <= 1'b0;
                    kitt_pos <= 4'd1;
                end else begin
                    kitt_pos <= kitt_pos - 4'd1;
                end
            end
        end else begin
            kitt_cnt <= kitt_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // 每颗灯的占空比（组合译码）
    //------------------------------------------------------------------
    reg [7:0] duty [0:15];
    integer   i;

    always @(*) begin
        for (i = 0; i < 16; i = i + 1) begin
            if (alarm_on) begin
                duty[i] = beat ? 8'd255 : 8'd0;              // 声光同步爆闪
            end else if (state_code == 2'd0) begin
                duty[i] = (i == 0) ? breathe_duty : 8'd0;    // 呼吸灯
            end else if (state_code == 2'd1) begin
                if (i == chase_pos)        duty[i] = 8'd255; // 流水：头 + 2 级拖尾
                else if (i == chase_prev1) duty[i] = 8'd96;
                else if (i == chase_prev2) duty[i] = 8'd40;
                else                       duty[i] = 8'd0;
            end else begin
                duty[i] = (i == kitt_pos) ? 8'd255 : 8'd0;   // KITT 扫描
            end
        end
    end

    //------------------------------------------------------------------
    // 输出：占空比比较。duty=0 恒灭，duty=255 仅 pwm_cnt=255 那一拍灭
    //------------------------------------------------------------------
    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led <= 16'h0000;
        end else begin
            for (j = 0; j < 16; j = j + 1) begin
                led[j] <= (pwm_cnt < duty[j]);
            end
        end
    end

endmodule
