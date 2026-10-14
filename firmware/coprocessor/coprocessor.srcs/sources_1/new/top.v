`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/10/14 23:06:40
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top (
    input  wire        clk,
    output reg [15:0]  led
);
    reg [24:0] cnt;
    always @(posedge clk) begin
        if (cnt == 25'd24_999_999) begin
            cnt <= 25'd0;
            led <= ~led;          // 16 颗一起 1Hz 同步闪
        end else
            cnt <= cnt + 25'd1;
    end
endmodule