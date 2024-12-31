`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gonzales
// 
// Create Date: 02/05/2024 09:49:53 AM
// Design Name: 
// Module Name: PIPE_BRANCH_GEN
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


module PIPE_BRANCH_GEN(
    input zero,
    input [31:0] result,
    input [2:0] func3,
    input branchEN,
    output logic branchSrc);
    
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
    branchSrc = 1'b0;
    if (branchEN) begin
    case (FUNC3)
        BEQ: begin  // branch if equal
            if (zero) begin
                branchSrc = 1'b1;
            end
        end        
        BNE: begin  // branch if not equal
            if (!zero) begin
                branchSrc = 1'b1;
            end
        end
        BLT: begin  // branch if less than
            if (result) begin
                branchSrc = 1'b1;
            end
        end
        BGE: begin  // branch if greater than or eq
            if (!result) begin
                branchSrc = 1'b1;
            end
        end          
        BLTU: begin  // branch if less than (unsigned)
            if (result) begin
                branchSrc = 1'b1;
            end
        end       
        BGEU: begin  // branch if greater or eq (unsigned)
            if (!result) begin
                branchSrc = 1'b1;
            end
        end    
        default: begin
        end
    endcase
    end
end
endmodule
