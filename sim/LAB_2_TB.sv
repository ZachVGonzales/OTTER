`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gonzales
// 
// Create Date: 11/13/2023 02:08:39 PM
// Design Name: 
// Module Name: EXP_7_TB
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


module LAB_2_TB();

logic        CLK_TB;
logic [4:0]  BUTTONS_TB;
logic [15:0] SWITCHES_TB,
             LEDS_TB;
logic [7:0]  SEGS_TB;
logic [3:0]  AN_TB;
      
OTTER_Wrapper UUT (
      .clk      (CLK_TB),
      .buttons  (BUTTONS_TB),
      .switches (SWITCHES_TB),
      .leds     (LEDS_TB),
      .segs     (SEGS_TB),
      .an       (AN_TB));

    always begin
        #5
        CLK_TB = 0;
        #5
        CLK_TB = 1;
    end
    
    always begin
        BUTTONS_TB = 5'b01000;
        #240
        
        BUTTONS_TB = 5'b00000;
        #100000000
        
        BUTTONS_TB = 4'b00000;
    end
      

endmodule
