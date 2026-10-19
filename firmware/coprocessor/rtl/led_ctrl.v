`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : led_ctrl
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : LED 状态效果引擎。用 8 位软 PWM 做亮度控制，按
//            state_code / alarm_on / 演示场景呈现以下效果：
//
//   alarm（或上电自检 / 演示场景 4）: 全部可用 LED 跟随蜂鸣器节拍
//                                     "声光同步爆闪"——beat=1 全亮、beat=0 全灭。
//   演示场景 5（走灯诊断）          : 单灯逐颗步进（SCENE_SLOT 步进），
//                                     兼作丝印↔引脚映射核对工具。
//   state 0（IDLE 静止）            : led[0] 三角波呼吸灯（亮 -> 暗 -> 亮约 2.6s），
//                                     "系统在跑"的柔和存活指示。
//   state 1（WALK 走动）            : 单灯带 2 级拖尾的环形流水（80ms/步）。
//   state 2（PENDING 待确认）       : KITT 式来回扫描（50ms/步）。
//   链路灯（led[LED_COUNT-1]）      : 非告警/非走灯状态下叠加：
//                                     无链路 = 0.5Hz 慢闪（满亮）；
//                                     已链路且静止 = 低亮度柔和脉动（不抢呼吸灯视觉）；
//                                     已链路且运动 = 每帧满亮闪 60ms（帧活动心跳）。
//
//   可用 LED 数由 LED_COUNT 决定（默认 12，对应板上主排 LED0~LED11）。
//   LED12~16 按项目要求禁用：led[LED_COUNT:15] 恒输出 0。
//   led[15]（M1）本就物理不亮，禁用后不再受其影响。
//
// 接口说明 :
//   clk/rst_n    50MHz / 异步低有效复位
//   state_code   最终显示状态码 0..3（top 仲裁后）
//   alarm_on     最终告警标志（UART 告警 | POST | 演示场景 4）
//   beat         蜂鸣器节拍包络（告警声光同步）
//   walk_mode    演示场景 5 走灯模式（alarm_on 优先于本模式）
//   walk_pos     走灯当前位置 0..LED_COUNT-1
//   link_ok      链路曾建立（cmd_decoder.ever_linked）
//   link_act     单拍脉冲：收到一帧合法命令（帧活动）
//   led[15:0]    高电平点亮；LED_COUNT 位及以上恒灭
//
// 参数说明 :
//   LED_COUNT       可用 LED 数（0..LED_COUNT-1 有效）
//   PWMW            软 PWM 位宽 8 -> 帧率 50MHz/256 约 195kHz
//   BREATHE_DIV     呼吸步进间隔：256 级 x 250_000 clk = 1.28s 单向
//   CHASE_STEP_DIV  流水步进间隔 4_000_000 -> 80ms/步
//   SCAN_STEP_DIV   KITT 扫描步进间隔 2_500_000 -> 50ms/步
//   HB_PERIOD_DIV   无链路慢闪整周期 50_000_000 -> 1s（0.5Hz）
//   ACT_MS_DIV      帧活动闪亮时长 3_000_000 -> 60ms
//   仿真时由 tb 用 defparam 覆盖为小值以缩短仿真时间。
////////////////////////////////////////////////////////////////////////////////

module led_ctrl #(
    parameter LED_COUNT      = 12,
    parameter PWMW           = 8,
    parameter BREATHE_DIV    = 250000,
    parameter CHASE_STEP_DIV = 4000000,
    parameter SCAN_STEP_DIV  = 2500000,
    parameter HB_PERIOD_DIV  = 50_000_000,
    parameter ACT_MS_DIV     = 3_000_000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  state_code,
    input  wire        alarm_on,
    input  wire        beat,
    input  wire        walk_mode,
    input  wire [3:0]  walk_pos,
    input  wire        link_ok,
    input  wire        link_act,
    output reg  [15:0] led
);

    localparam LINK_IDX  = LED_COUNT - 1;        // 链路灯位（最高可用位）
    localparam [3:0] LAST_IDX = LED_COUNT - 1;   // 供 4 位位置运算使用

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
    reg        breathe_dir;
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
    // 流水灯位置：0..LED_COUNT-1 循环；拖尾用回绕减法
    //------------------------------------------------------------------
    reg [3:0]  chase_pos;
    reg [31:0] chase_cnt;

    wire [3:0] chase_prev1 = (chase_pos == 4'd0) ? LAST_IDX : chase_pos - 4'd1;
    wire [3:0] chase_prev2 = (chase_prev1 == 4'd0) ? LAST_IDX : chase_prev1 - 4'd1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            chase_pos <= 4'd0;
            chase_cnt <= 32'd0;
        end else if (chase_cnt == CHASE_STEP_DIV - 1) begin
            chase_cnt <= 32'd0;
            chase_pos <= (chase_pos == LAST_IDX) ? 4'd0 : chase_pos + 4'd1;
        end else begin
            chase_cnt <= chase_cnt + 32'd1;
        end
    end

    //------------------------------------------------------------------
    // KITT 扫描位置：0 -> LED_COUNT-1 -> 0 来回
    //------------------------------------------------------------------
    reg [3:0]  kitt_pos;
    reg        kitt_dir;
    reg [31:0] kitt_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kitt_pos <= 4'd0;
            kitt_dir <= 1'b0;
            kitt_cnt <= 32'd0;
        end else if (kitt_cnt == SCAN_STEP_DIV - 1) begin
            kitt_cnt <= 32'd0;
            if (!kitt_dir) begin
                if (kitt_pos == LAST_IDX) begin
                    kitt_dir <= 1'b1;
                    kitt_pos <= LAST_IDX - 4'd1;
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
    // 链路灯：无链路 0.5Hz 慢闪；已链路帧活动闪 60ms
    //------------------------------------------------------------------
    reg [31:0] hb_cnt;
    wire       hb_sq = (hb_cnt < HB_PERIOD_DIV / 2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                          hb_cnt <= 32'd0;
        else if (hb_cnt >= HB_PERIOD_DIV - 1) hb_cnt <= 32'd0;
        else                                  hb_cnt <= hb_cnt + 32'd1;
    end

    reg [31:0] act_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              act_cnt <= 32'd0;
        else if (link_act)       act_cnt <= ACT_MS_DIV;
        else if (act_cnt != 0)   act_cnt <= act_cnt - 32'd1;
    end

    /* 链路灯（led[LED_COUNT-1]）的显示策略：
     *   未建立链路     -> 0.5Hz 慢闪（满亮）   "我在等基站"
     *   已链路 + 静止  -> **不点亮**          呼吸灯已足够表达"系统在跑"；
     *                                          静止画面里叠加心跳闪会形成孤立的
     *                                          亮点，观感像故障
     *   已链路 + 走动/待确认 -> 满亮 60ms 帧活动闪（此时画面本来在动） */
    reg  [7:0] act_duty;
    always @(*) begin
        if (state_code == 2'd0)      act_duty = 8'd0;      /* 静止：不叠加 */
        else                          act_duty = 8'd255;
    end
    wire [7:0] hb_duty = link_ok ? ((act_cnt != 0) ? act_duty : 8'd0)
                                 : (hb_sq ? 8'd255 : 8'd0);

    //------------------------------------------------------------------
    // 每颗 LED 的占空比（组合译码）
    //------------------------------------------------------------------
    reg [7:0] duty [0:15];
    integer   i;

    always @(*) begin
        for (i = 0; i < 16; i = i + 1) begin
            if (i >= LED_COUNT) begin
                duty[i] = 8'd0;                              // 禁用组恒灭
            end else if (alarm_on) begin
                duty[i] = beat ? 8'd255 : 8'd0;              // 声光同步爆闪
            end else if (walk_mode) begin
                duty[i] = ((walk_pos <= LAST_IDX) && (i == walk_pos))
                          ? 8'd255 : 8'd0;                   // 走灯诊断
            end else begin
                case (state_code)
                    2'd0: duty[i] = (i == 0) ? breathe_duty : 8'd0;
                    2'd1: begin
                        if (i == chase_pos)        duty[i] = 8'd255;
                        else if (i == chase_prev1) duty[i] = 8'd96;
                        else if (i == chase_prev2) duty[i] = 8'd40;
                        else                       duty[i] = 8'd0;
                    end
                    default: duty[i] = (i == kitt_pos) ? 8'd255 : 8'd0;
                endcase
                // 链路灯叠加（仅状态动效区，告警/走灯不叠加）
                if (i == LINK_IDX) begin
                    if (hb_duty > duty[i]) duty[i] = hb_duty;
                end
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
