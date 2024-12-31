`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: Zachary Gonzales
// 
// Create Date: 01/29/2024 04:56:13 PM
// Design Name: 
// Module Name: CU_Decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies:
// 
// CU_DCDR my_cu_dcdr(
//   .br_eq     (), 
//   .br_lt     (), 
//   .br_ltu    (),
//   .opcode    (),    //-  ir[6:0]
//   .func7     (),    //-  ir[30]
//   .func3     (),    //-  ir[14:12] 
//   .alu_fun   (),
//   .pcSource  (),
//   .alu_srcA  (),
//   .alu_srcB  (), 
//   .rf_wr_sel ()   );
//
// 
// Revision:
// Revision 1.00 - File Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed unneeded else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (05-01-2023) - reindent and fix formatting
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module PIPE_DCDR(
    input [6:0] opcode,   //-  ir[6:0]
    input func7,          //-  ir[30]
    input [2:0] func3,    //-  ir[14:12] 
    output logic [3:0] alu_fun,
    output logic [1:0] pcSource,
    output logic alu_srcA,
    output logic [1:0] alu_srcB, 
    output logic [1:0] rf_wr_sel,
    output logic regWrite,
    output logic memWE2,
    output logic memRDEN2,
    output logic branchEN,
    output logic i_type_sel,
    output logic u_type_sel,
    output logic jump   );
    
    //- datatypes for RISC-V opcode types
    typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        SYS    = 7'b1110011
    } opcode_t;
    opcode_t OPCODE; //- define variable of new opcode type
    
    assign OPCODE = opcode_t'(opcode); //- Cast input enum 

    //- datatype for func3Symbols tied to values
    typedef enum logic [2:0] {
        //BRANCH labels
        BEQ = 3'b000,
        BNE = 3'b001,
        BLT = 3'b100,
        BGE = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
    } func3_t;    
    func3_t FUNC3; //- define variable of new opcode type
    
    assign FUNC3 = func3_t'(func3); //- Cast input enum 
       
    always_comb begin 
        //- schedule all values to avoid latch
        // automaticaly does: next addr, sel rs2, write next addr
        // sel rs1, add opperation
        pcSource = 2'b00;   alu_srcB = 2'b00;     rf_wr_sel = 2'b00; 
        alu_srcA = 1'b0;    alu_fun  = 4'b0000;   regWrite = 1'b0;
        memWE2 = 1'b0;      memRDEN2 = 1'b0;      branchEN = 1'b0;
        jump = 1'b0;        i_type_sel = 1'b0;    u_type_sel = 1'b0;
        
        case(OPCODE)
            AUIPC: begin
                alu_srcA = 1'b1;
                alu_srcB = 2'b11;
                rf_wr_sel = 2'b11;
                regWrite = 1'b1;
                u_type_sel = 1'b1;  // necessary to avoid fp data dependency
                i_type_sel = 1'b1;  // necessary to avoid fp data dependency
            end
            
            JALR: begin
                pcSource = 2'b01;  // sel JAL
                regWrite = 1'b1;   // writing to a reg
                i_type_sel = 1'b1;  // necessary to avoid fp data dependency
                jump = 1'b1;
            end
            
            LUI: begin
                alu_fun = 4'b1001;  // lui alu opp
                alu_srcA = 1'b1;    // u-imm selected
                rf_wr_sel = 2'b11;  // writing the result of alu opp
                regWrite = 1'b1;    // writing to a reg
                u_type_sel = 1'b1;  // necessary to avoid fp data dependency
                i_type_sel = 1'b1;  // necessary to avoid fp data dependency
            end
            
            JAL: begin
                pcSource = 2'b10;  // write result of alu opp
                regWrite = 1'b1;    // writing to a reg
                jump = 1'b1;
                u_type_sel = 1'b1;  // necessary to avoid fp data dependency
                i_type_sel = 1'b1;  // necessary to avoid fp data dependency
            end
            
            LOAD: begin
                alu_srcB = 2'b01;   // i-imm selected
                rf_wr_sel = 2'b10;  // write data to reg
                regWrite = 1'b1;    // writing to a reg
                memRDEN2 = 1'b1;    // reading data mem
                i_type_sel = 1'b1;  // necessary to avoid fp data dependency
            end
            
            STORE: begin
                alu_srcB = 2'b00;   // just keep rs2 selected since now 
                                    // s-imm passed to execute state
                memWE2 = 1'b1;      // writing data mem
            end
            
            OP_IMM: begin
                regWrite = 1'b1;    // writing to a reg
                alu_srcB = 2'b01;   // i-imm selected
                rf_wr_sel = 2'b11;  // write result to reg
                i_type_sel = 1'b1;  // i type instruction so don't incur fp
                                    // data hazard
                if ((func3 == 3'b001) || (func3 == 3'b101)) begin
                    alu_fun = {func7, func3};  // func7 -> signed / unsigned
                                               // func3 -> operation type
                end else begin
                    alu_fun = {1'b0, func3};   // no need for sign
                end
                
            end

            
            OP_RG3: begin
                regWrite = 1'b1;    // writing result to a register
                rf_wr_sel = 2'b11;  // write result to reg
                alu_fun = {func7, func3};  // func7 -> signed / unsigned
                                           // func3 -> operation type
            end
            
            BRANCH: begin
                branchEN = 1'b1;
                case (func3)
                BEQ: begin
                    alu_fun = 4'b1000;
                end
                BNE: begin
                    alu_fun = 4'b1000;
                end
                BLT: begin
                    alu_fun = 4'b0010;
                end
                BGE: begin
                    alu_fun = 4'b0010;
                end
                BLTU: begin
                    alu_fun = 4'b0011;
                end
                BGEU: begin
                    alu_fun = 4'b0011;
                end
                endcase
            end

            default: begin
            end
        endcase
    end

endmodule