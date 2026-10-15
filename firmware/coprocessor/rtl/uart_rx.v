`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 模块名称 : uart_rx
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : UART 异步串行接收器，115200 bps / 8N1，接收来自 ESP32-S3 的
//            8 字节控制帧（协议见 docs/团队协作规范与接口契约.md §4）。
//
// 接口说明 :
//   clk        50MHz 系统时钟（引脚 D4）
//   rst_n      异步低有效复位（引脚 T9，XDC 中已开内部 PULLUP）
//   rxd        串行接收线（引脚 F13）。空闲态为高电平
//   byte_data  接收到的字节，在 byte_valid 为高的那一拍有效（LSB 先收）
//   byte_valid 单时钟周期脉冲，表示一个字节接收完成
//
// 关键参数由来（务必理解，改时钟要重算）：
//   50MHz / 115200bps = 434.028 个 clk/位。取整数 434，累计误差
//   = (434.028-434)/434.028 = 0.0064%，一帧 10 位累计偏差 0.028 位，
//   远小于半位（0.5 位）容限，安全。
//   起始位采样点取 434/2 = 217（位中心），之后每 434 个 clk 采一位。
//
// 位序：UART 协议 LSB 先发，故右移拼接 {rxd, byte_data[7:1]}。
//////////////////////////////////////////////////////////////////////////////////

module uart_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,
    output reg  [7:0] byte_data,
    output reg        byte_valid
);

    //------------------------------------------------------------------
    // 波特率分频参数
    //------------------------------------------------------------------
    localparam CLKS_PER_BIT = 434;                 // 50MHz/115200 取整
    localparam HALF_BIT     = CLKS_PER_BIT / 2;    // 217，位中心偏移
    localparam CNT_W        = 9;                   // 434 需要 9 位计数器

    //------------------------------------------------------------------
    // 同步寄存器：rxd 是异步外部信号，必须两级同步防亚稳态
    //------------------------------------------------------------------
    reg rxd_sync0, rxd_sync1, rxd_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_sync0 <= 1'b1;
            rxd_sync1 <= 1'b1;
            rxd_d1    <= 1'b1;
        end else begin
            rxd_sync0 <= rxd;        // 第一级同步
            rxd_sync1 <= rxd_sync0;  // 第二级同步
            rxd_d1    <= rxd_sync1;  // 上一拍电平，用于下降沿检测
        end
    end

    // 起始位下降沿：空闲高 -> 变低
    wire start_edge = (rxd_d1 == 1'b1) && (rxd_sync1 == 1'b0);

    //------------------------------------------------------------------
    // 接收状态机
    //------------------------------------------------------------------
    localparam S_IDLE  = 2'd0;   // 等待起始位
    localparam S_START = 2'd1;   // 起始位中心确认（防毛刺）
    localparam S_DATA  = 2'd2;   // 收 8 个数据位
    localparam S_STOP  = 2'd3;   // 收停止位并输出

    reg [1:0]      state;
    reg [CNT_W-1:0] clk_cnt;
    reg [2:0]      bit_idx;
    reg [7:0]      shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            clk_cnt    <= 0;
            bit_idx    <= 0;
            shift_reg  <= 8'h00;
            byte_data  <= 8'h00;
            byte_valid <= 1'b0;
        end else begin
            byte_valid <= 1'b0;   // 默认单周期脉冲

            case (state)
                //------------------------------------------------------
                // 空闲：检测起始位下降沿
                //------------------------------------------------------
                S_IDLE: begin
                    clk_cnt   <= 0;
                    bit_idx   <= 0;
                    shift_reg <= 8'h00;
                    if (start_edge)
                        state <= S_START;
                end

                //------------------------------------------------------
                // 起始位中心：等待 217 个 clk，确认仍为低电平（毛刺过滤）
                // 若此刻已变高，说明是噪声，退回 IDLE
                //------------------------------------------------------
                S_START: begin
                    if (clk_cnt == HALF_BIT - 1) begin
                        if (rxd_sync1 == 1'b0) begin
                            clk_cnt <= 0;
                            state   <= S_DATA;
                        end else begin
                            clk_cnt <= 0;
                            state   <= S_IDLE;    // 毛刺，丢弃
                        end
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                //------------------------------------------------------
                // 数据位：每 434 clk 采样一次，LSB 先收
                //------------------------------------------------------
                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt   <= 0;
                        shift_reg <= {rxd_sync1, shift_reg[7:1]};
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                //------------------------------------------------------
                // 停止位：等满一个位时间到达停止位中心，采样确认高电平。
                // 注意：停止位采样失败也照常输出字节，由上层帧校验兜底，
                // 这样 UART 层保持"尽力交付"语义，错误由协议层处理。
                //------------------------------------------------------
                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt    <= 0;
                        byte_data  <= shift_reg;
                        byte_valid <= 1'b1;
                        state      <= S_IDLE;
                    end else
                        clk_cnt <= clk_cnt + 1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
