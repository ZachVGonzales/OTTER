`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gonzales
// 
// Create Date: 02/14/2024 03:29:21 PM
// Design Name: 
// Module Name: HAZARD_UNIT
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


module HAZARD_UNIT(
    input [4:0] rs1_D,
    input [4:0] rs2_D,
    input [4:0] rs1_E,
    input [4:0] rs2_E,
    input [4:0] rd_E,
    input [4:0] rd_M,
    input [4:0] rd_W,
    input regWrite_M,
    input regWrite_W,
    input memRDEN2_E,
    input branchEN,
    input jumpEN,
    input cache_miss,
    output logic [1:0] FA_SEL,
    output logic [1:0] FB_SEL,
    output logic [1:0] JALR_SEL,
    output logic PC_en,
    output logic FD_reg_en,
    output logic DE_reg_en,  // needed to stall for a cache miss
    output logic EM_reg_en,  // ""
    output logic MW_reg_en,  // ""
    output logic flush_DE,
    output logic flush_FD
    );
    
always_comb begin
    // schedule values to avoid latch
    FA_SEL = 2'b00;    FB_SEL = 2'b00;    PC_en = 1'b1;
    FD_reg_en = 1'b1;  flush_DE = 1'b0;   flush_FD = 1'b0;
    JALR_SEL = 2'b00;  DE_reg_en = 1'b1;  EM_reg_en = 1'b1;
    MW_reg_en = 1'b1;
    
    // if there is a load hazard
    if (memRDEN2_E && ((rd_E == rs1_D) || (rd_E == rs2_D))) begin
        //stall for 1cc
        PC_en = 1'b0;
        FD_reg_en = 1'b0;
        flush_DE = 1'b1;
    end
    
    // forwarding control; prioritize forwarding more recent data
    // set the mux selectors acording to dependencies 
    if (regWrite_M && (rd_M != 0) && (rd_M == rs1_E)) begin
        FA_SEL = 2'b01;
    end 
    else if (regWrite_W && (rd_W != 0) && (rd_W == rs1_E)) begin
        FA_SEL = 2'b10;
    end
    
    if (regWrite_M && (rd_M != 0) && (rd_M == rs2_E)) begin
        FB_SEL = 2'b01;
    end
    else if (regWrite_W && (rd_W != 0) && (rd_W == rs2_E)) begin
        FB_SEL = 2'b10;
    end
    
    // branch hazard control 
    if (branchEN) begin
        flush_FD = 1'b1;
        flush_DE = 1'b1;
    end
    if (jumpEN) begin
        flush_FD = 1'b1;
    end
    
    // jalr hazard detection / forwarding
    if (rs1_D == rd_E) begin 
        JALR_SEL = 2'b01;
    end else if (rs1_D == rd_M) begin 
        JALR_SEL = 2'b10;
    end else if (rs1_D == rd_W) begin 
        JALR_SEL = 2'b11;
    end
    
    // on a cache miss, whole pipeline must stall 
    if (cache_miss) begin
        PC_en = 1'b0;
        FD_reg_en = 1'b0;
        DE_reg_en = 1'b0;
        EM_reg_en = 1'b0;
        MW_reg_en = 1'b0;
        // no flush since we are just making the whole pipeline wait
        flush_FD = 1'b0;
        flush_DE = 1'b0;
    end
end
endmodule
