`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gonzales
// 
// Create Date: 10/11/2023 09:26:00 AM
// Design Name: 
// Module Name: ALU
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


module PIPE_ALU(
    input [31:0] srcA,
    input [31:0] srcB,
    input [3:0] alu_fun,
    output logic [31:0] result,
    output logic zero
    );
    
    always_comb begin        
        case (alu_fun)
            4'b0000: 
            begin
                result = srcA + srcB;
            end
            4'b1000:
            begin
                result = srcA - srcB;
            end
            4'b0110:
            begin
                result = srcA | srcB;
            end
            4'b0111:
            begin
                result = srcA & srcB;
            end
            4'b0100:
            begin
                result = srcA ^ srcB;
            end
            4'b0101:
            begin
                result = srcA >> srcB[4:0];
            end
            4'b0001:
            begin
                result = srcA << srcB[4:0];
            end
            4'b1101:
            begin
                result = $signed(srcA) >>> srcB[4:0];
            end
            4'b0010:
            begin
                result = ($signed(srcA) < $signed(srcB)) ? 1 : 0;
            end
            4'b0011:
            begin
                result = (srcA < srcB) ? 1 : 0;
            end
            4'b1001:
            begin
                result = srcA;
            end
            default:
            begin
                result = 32'hDEAD_BEEF;
            end
        endcase
        
        zero = ~(32'h0000_0000 || result);
    end
    
    
endmodule