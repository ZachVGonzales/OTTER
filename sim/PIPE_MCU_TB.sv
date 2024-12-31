`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gonzales
// 
// Create Date: 02/05/2024 12:03:49 PM
// Design Name: 
// Module Name: PIPE_MCU_TB
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


module PIPE_MCU_TB();
    logic RST;                // reset (button to resart program)
    logic INTR;               // not used currently
    logic [31:0] IOBUS_IN;    // input data
    logic CLK;                // clock
    logic IOBUS_WR;          // signal telling the wrapper to store the output
    logic [31:0] IOBUS_OUT;  // output data
    logic [31:0] IOBUS_ADDR;  // address signal (input or output)
    
PIPE_OTTERV2 UUT (.RST(RST), .INTR(INTR), .IOBUS_IN(IOBUS_IN), .CLK(CLK), .IOBUS_WR(IOBUS_WR), .IOBUS_OUT(IOBUS_OUT), .IOBUS_ADDR(IOBUS_ADDR));

always begin
#5 CLK = 0;
#5 CLK = 1;
end

always begin
#10 RST = 1;
#10 RST = 0;
#1000000 RST = 0;
end
endmodule
