`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// 模块名称 : uart_tx
// 所属工程 : coprocessor (Artix-7 xc7a35tftg256-1)
// 功能描述 : UART 异步串行发送器，115200 bps / 8N1。
//            用于 FPGA -> ESP32-S3 的上行 UI 链路（拨码快照 / 键盘事件帧），
//            与 uart_rx 完全镜像（同 434 clk/位参数，50MHz）。
//            现阶段物理引脚为板上 USB-UART 桥（F12）：不接 S3 时用电脑串口
//            即可直接观测上行帧字节，作为无板/上板前验证手段。
//
// 接口说明 :
//   clk        50MHz 系统时钟
//   rst_n      异步低有效复位
//   start      单时钟周期脉冲：请求发送 byte_data（busy=0 时有效）
//   byte_data  待发送字节（start 当拍必须有效，模块内部锁存）
//   busy       1 = 正在发送（start 必须等待）
//   txd        串行输出，空闲高电平
//
// 位时序与 uart_rx 相同：50MHz/115200 = 434.028 clk/位，取 434，
// 累计误差 0.0064%/位，一帧 10 位偏差 0.064 位，远小于半位容限。
////////////////////////////////////////////////////////////////////////////////

module uart_tx #(
    parameter CLKS_PER_BIT = 434          // 50MHz / 115200 取整
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] byte_data,
    output wire       busy,
    output reg        txd
);

    //------------------------------------------------------------------
    // 发送状态机：空闲 -> 起始位 -> 8 数据位 -> 停止位
    //------------------------------------------------------------------
    localparam S_IDLE  = 2'd0;
    localparam S_START = 2'd1;
    localparam S_DATA  = 2'd2;
    localparam S_STOP  = 2'd3;

    reg [1:0]        state;
    reg [8:0]        clk_cnt;             // 434 需要 9 位
    reg [2:0]        bit_idx;
    reg [7:0]        shift;

    assign busy = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            clk_cnt  <= 0;
            bit_idx  <= 0;
            shift    <= 8'h00;
            txd      <= 1'b1;              // 空闲高电平
        end else begin
            case (state)
                S_IDLE: begin
                    txd <= 1'b1;
                    if (start) begin
                        shift   <= byte_data;
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_START;
                    end
                end

                S_START: begin
                    txd <= 1'b0;           // 起始位
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    txd <= shift[0];       // LSB 先发
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        shift   <= {1'b0, shift[7:1]};
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    txd <= 1'b1;           // 停止位
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        state   <= S_IDLE;
                        clk_cnt <= 0;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
