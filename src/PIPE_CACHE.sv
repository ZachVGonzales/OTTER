`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Gpnzales
// 
// Create Date: 03/13/2024 07:35:42 AM
// Design Name: 
// Module Name: PIPE_CACHE
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


module PIPE_CACHE(
    input mem_clk,
    input mem_rden1,        // read enable Instruction
    input mem_rden2,        // read enable data
    input mem_we2,          // write enable.
    input [31:0] mem_addr1, // Instruction Memory Addr
    input [31:0] mem_addr2, // Data Memory Addr
    input [31:0] mem_din2,  // Data to save
    input [1:0] mem_size,   // 0-Byte, 1-Half, 2-Word
    input mem_sign,         // 1-unsigned 0-signed
    input [31:0] io_in,     // Data from IO     
    output logic stall,     // accessing MM so need to stall pipeline (send this to hazard unit)
    //output ERR,
    output logic io_wr,     // IO 1-write 0-read
    output logic [31:0] mem_dout1,  // Instruction
    output logic [31:0] mem_dout2); // Data
    
    // miss signals for fetching from MM
    logic wb, ld1, ld2;
    
    logic [31:0] memReadWord, ioBuffer, memReadSized;
    logic [1:0] wordOffset1,  // need 2 for B/O and W/O since we have dual
                wordOffset2;  // port memory so need to accesses both at the
    logic [1:0] byteOffset1,  // same time so need 2 seperate caches
                byteOffset2;
    logic [1:0] index1,
                index2;
    logic [26:0] tag1,
                 tag2;
    logic weAddrValid;      // active when saving (WE) to valid memory address
    
    // initialize main memory
    (* rom_style="{distributed | block}" *)
    (* ram_decomp = "power" *) logic [31:0] main_memory [0:16383];
    
    // Declare the memory variable
    // 160 bits for each block, 16 blocks total, 2 caches (one for instruct
    // memory and one for data memory since need to access simultaniously)
	reg [157:0] memory1[15:0]; 
	reg [157:0] memory2[15:0];
    
    // initalize the caches to 0 so we know that nothing is valid at first
    initial begin
        memory1[0] = 0; memory1[1] = 0; memory1[2] = 0; memory1[3] = 0;
        memory1[4] = 0; memory1[5] = 0; memory1[6] = 0; memory1[7] = 0;
        memory1[8] = 0; memory1[9] = 0; memory1[10] = 0; memory1[11] = 0;
        memory1[12] = 0; memory1[13] = 0; memory1[14] = 0; memory1[15] = 0;
        memory2[0] = 0; memory2[1] = 0; memory2[2] = 0; memory2[3] = 0;
        memory2[4] = 0; memory2[5] = 0; memory2[6] = 0; memory2[7] = 0;
        memory2[8] = 0; memory2[9] = 0; memory2[10] = 0; memory2[11] = 0;
        memory2[12] = 0; memory2[13] = 0; memory2[14] = 0; memory2[15] = 0;
        $readmemh("matmul_50_rep.mem", main_memory, 0, 16383);
    end
    
    // for finding the words we need in the caches
    assign byteOffset1 = 2'b00;  // MUST be word aligned so just keep at 00
    assign byteOffset2 = mem_addr2[1:0];
    assign wordOffset1 = mem_addr1[3:2];
    assign wordOffset2 = mem_addr2[3:2];
    assign index1 = mem_addr1[5:4];
    assign index2 = mem_addr2[5:4];
    assign tag1 = mem_addr1[31:6];
    assign tag2 = mem_addr2[31:6];
    
    // try reading/writing from cache, report hit or miss
    // on miss pipeline must stall so can access L2
    always_ff @(negedge mem_clk) begin 
        // set stall and writeback signals to 0 since only want them high for 1cc
        stall = 1'b0;
        wb = 1'b0;
        ld1 = 1'b0;
        ld2 = 1'b0;
        if (weAddrValid == 1) begin // doing a write (only care about data mem)
            // if to check for valid and if propper tag is there (do it for each block in set)
            if ((memory2[{index2, 2'b11}][157] == 1) && (mem_addr2[31:6] == memory2[{index2, 2'b11}][153:128])) begin
                // since writing to cache must set dirty bit
                memory2[{index2, 2'b11}][156] <= 1'b1; 
                
                // need to update LRU replacement values
                case (memory2[{index2, 2'b11}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                    case (wordOffset2) // go to specific word being written
                        2'b11: begin
                            case ({mem_size, byteOffset2}) // go to byte being written
                                4'b0000: memory2[{index2,2'b11}][103:96]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b11}][111:104] <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b11}][119:112] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b11}][127:120] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b11}][111:96]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b11}][119:104] <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b11}][127:112] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b11}][127:96]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b10: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b11}][71:64]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b11}][79:72]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b11}][87:80]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b11}][95:88]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b11}][79:64]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b11}][87:72]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b11}][95:80]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b11}][95:64]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b01: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b11}][39:32]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b11}][47:40]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b11}][55:48]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b11}][63:56]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b11}][47:32]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b11}][55:40]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b11}][63:48]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b11}][63:32]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b00: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b11}][7:0]   <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b11}][15:8]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b11}][23:16] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b11}][31:24] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b11}][15:0]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b11}][23:8]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b11}][31:16] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b11}][31:0]  <= mem_din2;          // sw
                            endcase
                        end
                    endcase
            end else if ((memory2[{index2, 2'b10}][157] == 1) && (mem_addr2[31:6] == memory2[{index2, 2'b10}][153:128])) begin
                // since writing to cache must set dirty bit
                memory2[{index2, 2'b10}][156] <= 1'b1; 
                
                // need to update LRU replacement values
                case (memory2[{index2, 2'b10}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                
                    case (wordOffset2) // go to specific word being written
                        2'b11: begin
                            case ({mem_size, byteOffset2}) // go to byte being written
                                4'b0000: memory2[{index2,2'b10}][103:96]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b10}][111:104] <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b10}][119:112] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b10}][127:120] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b10}][111:96]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b10}][119:104] <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b10}][127:112] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b10}][127:96]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b10: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b10}][71:64]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b10}][79:72]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b10}][87:80]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b10}][95:88]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b10}][79:64]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b10}][87:72]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b10}][95:80]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b10}][95:64]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b01: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b10}][39:32]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b10}][47:40]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b10}][55:48]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b10}][63:56]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b10}][47:32]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b10}][55:40]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b10}][63:48]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b10}][63:32]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b00: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b10}][7:0]   <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b10}][15:8]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b10}][23:16] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b10}][31:24] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b10}][15:0]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b10}][23:8]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b10}][31:16] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b10}][31:0]  <= mem_din2;          // sw
                            endcase
                        end
                    endcase
            end else if ((memory2[{index2, 2'b01}][157] == 1) && (mem_addr2[31:6] == memory2[{index2, 2'b01}][153:128])) begin
                // since writing to cache must set dirty bit
                memory2[{index2, 2'b01}][156] <= 1'b1; 
                
                // need to update LRU replacement values
                case (memory2[{index2, 2'b01}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                
                    case (wordOffset2) // go to specific word being written
                        2'b11: begin
                            case ({mem_size, byteOffset2}) // go to byte being written
                                4'b0000: memory2[{index2,2'b01}][103:96]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b01}][111:104] <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b01}][119:112] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b01}][127:120] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b01}][111:96]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b01}][119:104] <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b01}][127:112] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b01}][127:96]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b10: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b01}][71:64]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b01}][79:72]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b01}][87:80]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b01}][95:88]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b01}][79:64]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b01}][87:72]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b01}][95:80]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b01}][95:64]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b01: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b01}][39:32]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b01}][47:40]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b01}][55:48]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b01}][63:56]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b01}][47:32]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b01}][55:40]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b01}][63:48]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b01}][63:32]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b00: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b01}][7:0]   <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b01}][15:8]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b01}][23:16] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b01}][31:24] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b01}][15:0]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b01}][23:8]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b01}][31:16] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b01}][31:0]  <= mem_din2;          // sw
                            endcase
                        end
                    endcase
            end else if ((memory2[{index2, 2'b00}][157] == 1) && (mem_addr2[31:6] == memory2[{index2, 2'b00}][153:128])) begin
                // since writing to cache must set dirty bit
                memory2[{index2, 2'b00}][156] <= 1'b1; 
                
                // need to update LRU replacement values
                case (memory2[{index2, 2'b00}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                    end
                endcase
                
               
                    case (wordOffset2) // go to specific word being written
                        2'b11: begin
                            case ({mem_size, byteOffset2}) // go to byte being written
                                4'b0000: memory2[{index2,2'b00}][103:96]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b00}][111:104] <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b00}][119:112] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b00}][127:120] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b00}][111:96]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b00}][119:104] <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b00}][127:112] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b00}][127:96]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b10: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b00}][71:64]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b00}][79:72]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b00}][87:80]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b00}][95:88]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b00}][79:64]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b00}][87:72]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b00}][95:80]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b00}][95:64]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b01: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b00}][39:32]  <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b00}][47:40]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b00}][55:48]  <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b00}][63:56]  <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b00}][47:32]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b00}][55:40]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b00}][63:48]  <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b00}][63:32]  <= mem_din2;          // sw
                            endcase
                        end
                        2'b00: begin
                            case ({mem_size, byteOffset2}) 
                                4'b0000: memory2[{index2,2'b00}][7:0]   <= mem_din2[7:0];     // sb at byte offsets
                                4'b0001: memory2[{index2,2'b00}][15:8]  <= mem_din2[7:0];
                                4'b0010: memory2[{index2,2'b00}][23:16] <= mem_din2[7:0];
                                4'b0011: memory2[{index2,2'b00}][31:24] <= mem_din2[7:0];
                                4'b0100: memory2[{index2,2'b00}][15:0]  <= mem_din2[15:0];    // sh at byte offsets
                                4'b0101: memory2[{index2,2'b00}][23:8]  <= mem_din2[15:0];
                                4'b0110: memory2[{index2,2'b00}][31:16] <= mem_din2[15:0];
                                4'b1000: memory2[{index2,2'b00}][31:0]  <= mem_din2;          // sw
                            endcase
                        end
                    endcase
            end else begin
                // otherwise miss occurs and need to access main memory 
                stall = 1'b1;
                ld2 = 1'b1;
            end
        end
        
        // instr read
        if (mem_rden1) begin 
            // check if the data in cache is valid and if there is a tag that matches the address
            if ((memory1[{index1, 2'b11}][157] == 1'b1) && (memory1[{index1, 2'b11}][153:128] == tag1)) begin
                 // need to update LRU replacement values
                case (memory1[{index1, 2'b11}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB hold, XOR bits for new LSB)
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], (memory1[{index1, 2'b10}][154] ^ 1'b1) & memory1[{index1, 2'b10}][155]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], (memory1[{index1, 2'b01}][154] ^ 1'b1) & memory1[{index1, 2'b01}][155]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], (memory1[{index1, 2'b00}][154] ^ 1'b1) & memory1[{index1, 2'b00}][155]};
                    end
                endcase
                
                // output the correct word
                case (wordOffset1)
                    2'b11: begin
                        mem_dout1 <= memory1[{index1, 2'b11}][127:96];
                    end
                    2'b10: begin
                        mem_dout1 <= memory1[{index1, 2'b11}][95:64];
                    end
                    2'b01: begin
                        mem_dout1 <= memory1[{index1, 2'b11}][63:32];
                    end
                    2'b00: begin
                        mem_dout1 <= memory1[{index1, 2'b11}][31:0];
                    end
                endcase
            end else if ((memory1[{index1, 2'b10}][157] == 1'b1) && (memory1[{index1, 2'b10}][153:128] == tag1)) begin
                // update LRU values
                case (memory1[{index1, 2'b10}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                     2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], (memory1[{index1, 2'b11}][154] ^ 1'b1) & memory1[{index1, 2'b11}][155]}; 
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], (memory1[{index1, 2'b01}][154] ^ 1'b1) & memory1[{index1, 2'b01}][155]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], (memory1[{index1, 2'b00}][154] ^ 1'b1) & memory1[{index1, 2'b00}][155]};
                    end
                endcase
                
                case (wordOffset1)
                    2'b11: begin
                        mem_dout1 <= memory1[{index1, 2'b10}][127:96];
                    end
                    2'b10: begin
                        mem_dout1 <= memory1[{index1, 2'b10}][95:64];
                    end
                    2'b01: begin
                        mem_dout1 <= memory1[{index1, 2'b10}][63:32];
                    end
                    2'b00: begin
                        mem_dout1 <= memory1[{index1, 2'b10}][31:0];
                    end
                endcase
            end else if ((memory1[{index1, 2'b01}][157] == 1'b1) && (memory1[{index1, 2'b01}][153:128] == tag1)) begin
                // update LRU values
                case (memory1[{index1, 2'b01}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], memory1[{index1, 2'b00}][155] ^ memory1[{index1, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], (memory1[{index1, 2'b11}][154] ^ 1'b1) & memory1[{index1, 2'b11}][155]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], (memory1[{index1, 2'b10}][154] ^ 1'b1) & memory1[{index1, 2'b10}][155]};
                        memory1[{index1, 2'b00}][155:154] = {memory1[{index1, 2'b00}][155] & memory1[{index1, 2'b00}][154], (memory1[{index1, 2'b00}][154] ^ 1'b1) & memory1[{index1, 2'b00}][155]};
                    end
                endcase
                
                case (wordOffset1)
                    2'b11: begin
                        mem_dout1 <= memory1[{index1, 2'b01}][127:96];
                    end
                    2'b10: begin
                        mem_dout1 <= memory1[{index1, 2'b01}][95:64];
                    end
                    2'b01: begin
                        mem_dout1 <= memory1[{index1, 2'b01}][63:32];
                    end
                    2'b00: begin
                        mem_dout1 <= memory1[{index1, 2'b01}][31:0];
                    end
                endcase
            end else if ((memory1[{index1, 2'b00}][157] == 1'b1) && (memory1[{index1, 2'b00}][153:128] == tag1)) begin
                // need to update LRU replacement values
                case (memory1[{index1, 2'b00}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]};
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], memory1[{index1, 2'b11}][155] ^ memory1[{index1, 2'b11}][154]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], memory1[{index1, 2'b10}][155] ^ memory1[{index1, 2'b10}][154]};
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], memory1[{index1, 2'b01}][155] ^ memory1[{index1, 2'b01}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory1[{index1, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory1[{index1, 2'b11}][155:154] = {memory1[{index1, 2'b11}][155] & memory1[{index1, 2'b11}][154], (memory1[{index1, 2'b11}][154] ^ 1'b1) & memory1[{index1, 2'b11}][155]}; 
                        memory1[{index1, 2'b10}][155:154] = {memory1[{index1, 2'b10}][155] & memory1[{index1, 2'b10}][154], (memory1[{index1, 2'b10}][154] ^ 1'b1) & memory1[{index1, 2'b10}][155]};
                        memory1[{index1, 2'b01}][155:154] = {memory1[{index1, 2'b01}][155] & memory1[{index1, 2'b01}][154], (memory1[{index1, 2'b01}][154] ^ 1'b1) & memory1[{index1, 2'b01}][155]};
                    end
                endcase
                
                case (wordOffset1)
                    2'b11: begin
                        mem_dout1 <= memory1[{index1, 2'b00}][127:96];
                    end
                    2'b10: begin
                        mem_dout1 <= memory1[{index1, 2'b00}][95:64];
                    end
                    2'b01: begin
                        mem_dout1 <= memory1[{index1, 2'b00}][63:32];
                    end
                    2'b00: begin
                        mem_dout1 <= memory1[{index1, 2'b00}][31:0];
                    end
                endcase
            end else begin
                // if we get here then we have missed and need to stall for a load from MM
                stall = 1'b1;
                ld1 = 1'b1;
            end
        end
        
        // reading data
        if (mem_rden2) begin 
            if ((memory2[{index2, 2'b11}][157] == 1'b1) && (memory2[{index2, 2'b11}][153:128] == mem_addr2[31:6])) begin
                // need to update LRU replacement values
                case (memory2[{index2, 2'b11}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b11}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                case (wordOffset2)
                    2'b11: begin
                        memReadWord <= memory2[{index2, 2'b11}][127:96];
                    end
                    2'b10: begin
                        memReadWord <= memory2[{index2, 2'b11}][95:64];
                    end
                    2'b01: begin
                        memReadWord <= memory2[{index2, 2'b11}][63:32];
                    end
                    2'b00: begin
                        memReadWord <= memory2[{index2, 2'b11}][31:0];
                    end
                endcase
            end else if ((memory2[{index2, 2'b10}][157] == 1'b1) && (memory2[{index2, 2'b10}][153:128] == mem_addr2[31:6])) begin
                 // need to update LRU replacement values
                case (memory2[{index2, 2'b10}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                     2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b10}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                case (wordOffset2)
                    2'b11: begin
                        memReadWord <= memory2[{index2, 2'b10}][127:96];
                    end
                    2'b10: begin
                        memReadWord <= memory2[{index2, 2'b10}][95:64];
                    end
                    2'b01: begin
                        memReadWord <= memory2[{index2, 2'b10}][63:32];
                    end
                    2'b00: begin
                        memReadWord <= memory2[{index2, 2'b10}][31:0];
                    end
                endcase
            end else if ((memory2[{index2, 2'b01}][157] == 1'b1) && (memory2[{index2, 2'b01}][153:128] == mem_addr2[31:6])) begin
                // need to update LRU replacement values
                case (memory2[{index2, 2'b01}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], memory2[{index2, 2'b00}][155] ^ memory2[{index2, 2'b00}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b01}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]};
                        memory2[{index2, 2'b00}][155:154] = {memory2[{index2, 2'b00}][155] & memory2[{index2, 2'b00}][154], (memory2[{index2, 2'b00}][154] ^ 1'b1) & memory2[{index2, 2'b00}][155]};
                    end
                endcase
                
                case (wordOffset2)
                    2'b11: begin
                        memReadWord <= memory2[{index2, 2'b01}][127:96];
                    end
                    2'b10: begin
                        memReadWord <= memory2[{index2, 2'b01}][95:64];
                    end
                    2'b01: begin
                        memReadWord <= memory2[{index2, 2'b01}][63:32];
                    end
                    2'b00: begin
                        memReadWord <= memory2[{index2, 2'b01}][31:0];
                    end
                endcase
            end else if ((memory2[{index2, 2'b00}][157] == 1'b1) && (memory2[{index2, 2'b00}][153:128] == mem_addr2[31:6])) begin
                // need to update LRU replacement values
                case (memory2[{index2, 2'b00}][155:154])
                    2'b11: begin // nothing needs to be done (this is still the most recent)
                    end 
                    2'b10: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (MSB stay, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                    end
                    2'b01: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR bits for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], memory2[{index2, 2'b11}][155] ^ memory2[{index2, 2'b11}][154]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], memory2[{index2, 2'b10}][155] ^ memory2[{index2, 2'b10}][154]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], memory2[{index2, 2'b01}][155] ^ memory2[{index2, 2'b01}][154]};
                    end
                    2'b00: begin // this now the most recent, bit manipulation to keep relative order of others
                        // simple replacement
                        memory2[{index2, 2'b00}][155:154] = 2'b11; 
                        // don't know specifc value so weird bit manip (AND bits for new MSB, XOR LSB w/ 1 for new LSB)
                        memory2[{index2, 2'b11}][155:154] = {memory2[{index2, 2'b11}][155] & memory2[{index2, 2'b11}][154], (memory2[{index2, 2'b11}][154] ^ 1'b1) & memory2[{index2, 2'b11}][155]}; 
                        memory2[{index2, 2'b10}][155:154] = {memory2[{index2, 2'b10}][155] & memory2[{index2, 2'b10}][154], (memory2[{index2, 2'b10}][154] ^ 1'b1) & memory2[{index2, 2'b10}][155]};
                        memory2[{index2, 2'b01}][155:154] = {memory2[{index2, 2'b01}][155] & memory2[{index2, 2'b01}][154], (memory2[{index2, 2'b01}][154] ^ 1'b1) & memory2[{index2, 2'b01}][155]};
                    end
                endcase
                
                case (wordOffset2)
                    2'b11: begin
                        memReadWord <= memory2[{index2, 2'b00}][127:96];
                    end
                    2'b10: begin
                        memReadWord <= memory2[{index2, 2'b00}][95:64];
                    end
                    2'b01: begin
                        memReadWord <= memory2[{index2, 2'b00}][63:32];
                    end
                    2'b00: begin
                        memReadWord <= memory2[{index2, 2'b00}][31:0];
                    end
                endcase
            end else begin
                // if we get here then we have missed and need to stall for a load from MM
                stall = 1'b1;
                ld2 = 1'b1;
            end
        end
    end
    
    // need to format the data memory (right size + sign extend) before outputting
    always_comb begin
        case({mem_sign,mem_size,byteOffset2})
            5'b00011: memReadSized = {{24{memReadWord[31]}},memReadWord[31:24]};    // signed byte
            5'b00010: memReadSized = {{24{memReadWord[23]}},memReadWord[23:16]};
            5'b00001: memReadSized = {{24{memReadWord[15]}},memReadWord[15:8]};
            5'b00000: memReadSized = {{24{memReadWord[7]}},memReadWord[7:0]};
                                    
            5'b00110: memReadSized = {{16{memReadWord[31]}},memReadWord[31:16]};    // signed half
            5'b00101: memReadSized = {{16{memReadWord[23]}},memReadWord[23:8]};
            5'b00100: memReadSized = {{16{memReadWord[15]}},memReadWord[15:0]};
            
            5'b01000: memReadSized = memReadWord;                   // word
               
            5'b10011: memReadSized = {24'd0,memReadWord[31:24]};    // unsigned byte
            5'b10010: memReadSized = {24'd0,memReadWord[23:16]};
            5'b10001: memReadSized = {24'd0,memReadWord[15:8]};
            5'b10000: memReadSized = {24'd0,memReadWord[7:0]};
               
            5'b10110: memReadSized = {16'd0,memReadWord[31:16]};    // unsigned half
            5'b10101: memReadSized = {16'd0,memReadWord[23:8]};
            5'b10100: memReadSized = {16'd0,memReadWord[15:0]};
            
            default:  memReadSized = 32'b0;     // unsupported size, byte offset combination 
        endcase
    end
    
    // buffer the IO input for reading
    always_ff @(negedge mem_clk) begin
        if(mem_rden2)
            ioBuffer <= io_in;
    end
    
    // Memory Mapped IO 
    always_comb begin
        if(mem_addr2 >= 32'h00010000) begin  // MMIO address range
            io_wr = mem_we2;                 // IO Write
            mem_dout2 = ioBuffer;            // IO read from buffer
            weAddrValid = 0;                 // address beyond memory range
        end 
        else begin
            io_wr = 0;                  // not MMIO
            mem_dout2 = memReadSized;   // output sized and sign extended data
            weAddrValid = mem_we2;      // address in valid memory range
        end
    end
	
	// for misses on cache
	always_ff @(negedge mem_clk) begin
	   // need to replace the LRU in the set mapped to by the index values of the address
	   if (ld2) begin
	       // check each one in set for 00 LRU bits (this is the one being replaced)
	       if (memory2[{index2, 2'b11}][155:154] == 2'b00) begin
	           // need to check if writeback is neccessary first with dirty bit
	           if (memory2[{index2, 2'b11}][156]) begin
	               main_memory[{tag2, index2, 2'b11}] = memory2[{index2, 2'b11}][127:96];
	               main_memory[{tag2, index2, 2'b10}] = memory2[{index2, 2'b11}][95:64];
	               main_memory[{tag2, index2, 2'b01}] = memory2[{index2, 2'b11}][63:32];
	               main_memory[{tag2, index2, 2'b00}] = memory2[{index2, 2'b11}][31:0];
	               memory2[{index2, 2'b11}][156] = 1'b0; //turn off dirty bit
	           end
	           memory2[{index2, 2'b11}][127:0] <= {main_memory[{tag2, index2, 2'b11}], main_memory[{tag2, index2, 2'b10}], main_memory[{tag2, index2, 2'b01}], main_memory[{tag2, index2, 2'b00}]};
	           memory2[{index2, 2'b11}][153:128] = tag2;
	           memory2[{index2, 2'b11}][157] = 1'b1;  // set the valid bit
	       end else if (memory2[{index2, 2'b10}][155:154] == 2'b00) begin
	           // need to check if writeback is neccessary first with dirty bit
	           if (memory2[{index2, 2'b10}][156]) begin
	               main_memory[{tag2, index2, 2'b11}] = memory2[{index2, 2'b10}][127:96];
	               main_memory[{tag2, index2, 2'b10}] = memory2[{index2, 2'b10}][95:64];
	               main_memory[{tag2, index2, 2'b01}] = memory2[{index2, 2'b10}][63:32];
	               main_memory[{tag2, index2, 2'b00}] = memory2[{index2, 2'b10}][31:0];
	               memory2[{index2, 2'b10}][156] = 1'b0; //turn off dirty bit
	           end
	           memory2[{index2, 2'b10}][127:0] <= {main_memory[{tag2, index2, 2'b11}], main_memory[{tag2, index2, 2'b10}], main_memory[{tag2, index2, 2'b01}], main_memory[{tag2, index2, 2'b00}]};
	           memory2[{index2, 2'b10}][153:128] = tag2;
	           memory2[{index2, 2'b10}][157] = 1'b1;  // set the valid bit
	       end else if (memory2[{index2, 2'b01}][155:154] == 2'b00) begin
	           // need to check if writeback is neccessary first with dirty bit
	           if (memory2[{index2, 2'b01}][156]) begin
	               main_memory[{tag2, index2, 2'b11}] = memory2[{index2, 2'b01}][127:96];
	               main_memory[{tag2, index2, 2'b10}] = memory2[{index2, 2'b01}][95:64];
	               main_memory[{tag2, index2, 2'b01}] = memory2[{index2, 2'b01}][63:32];
	               main_memory[{tag2, index2, 2'b00}] = memory2[{index2, 2'b01}][31:0];
	               memory2[{index2, 2'b01}][156] = 1'b0; //turn off dirty bit
	           end
	           memory2[{index2, 2'b01}][127:0] <= {main_memory[{tag2, index2, 2'b11}], main_memory[{tag2, index2, 2'b10}], main_memory[{tag2, index2, 2'b01}], main_memory[{tag2, index2, 2'b00}]};
	           memory2[{index2, 2'b01}][153:128] = tag2;
	           memory2[{index2, 2'b01}][157] = 1'b1;  // set the valid bit
	       end else if (memory2[{index2, 2'b00}][155:154] == 2'b00) begin
	           // need to check if writeback is neccessary first with dirty bit
	           if (memory2[{index2, 2'b00}][156]) begin
	               main_memory[{tag2, index2, 2'b11}] = memory2[{index2, 2'b00}][127:96];
	               main_memory[{tag2, index2, 2'b10}] = memory2[{index2, 2'b00}][95:64];
	               main_memory[{tag2, index2, 2'b01}] = memory2[{index2, 2'b00}][63:32];
	               main_memory[{tag2, index2, 2'b00}] = memory2[{index2, 2'b00}][31:0];
	               memory2[{index2, 2'b00}][156] = 1'b0; //turn off dirty bit
	           end
	           memory2[{index2, 2'b00}][127:0] <= {main_memory[{tag2, index2, 2'b11}], main_memory[{tag2, index2, 2'b10}], main_memory[{tag2, index2, 2'b01}], main_memory[{tag2, index2, 2'b00}]};
	           memory2[{index2, 2'b00}][153:128] = tag2;
	           memory2[{index2, 2'b00}][157] = 1'b1;  // set the valid bit
	       end
	   end
	   
	   if (ld1) begin
	       // check each one in set for 00 LRU bits (this is the one being replaced)
	       // once found then insert new words, new tag and new valid bit
	       if (memory1[{index1, 2'b11}][155:154] == 2'b00) begin
	           memory1[{index1, 2'b11}][127:0] <= {main_memory[{tag1, index1, 2'b11}], main_memory[{tag1, index1, 2'b10}], main_memory[{tag1, index1, 2'b01}], main_memory[{tag1, index1, 2'b00}]};
	           memory1[{index1, 2'b11}][153:128] = tag1;
	           memory1[{index1, 2'b11}][157] = 1'b1;
	       end else if (memory1[{index1, 2'b10}][155:154] == 2'b00) begin
	           memory1[{index1, 2'b10}][127:0] <= {main_memory[{tag1, index1, 2'b11}], main_memory[{tag1, index1, 2'b10}], main_memory[{tag1, index1, 2'b01}], main_memory[{tag1, index1, 2'b00}]};
	           memory1[{index1, 2'b10}][153:128] = tag1;
	           memory1[{index1, 2'b10}][157] = 1'b1;
	       end else if (memory1[{index1, 2'b01}][155:154] == 2'b00) begin
	           memory1[{index1, 2'b01}][127:0] <= {main_memory[{tag1, index1, 2'b11}], main_memory[{tag1, index1, 2'b10}], main_memory[{tag1, index1, 2'b01}], main_memory[{tag1, index1, 2'b00}]};
	           memory1[{index1, 2'b01}][153:128] = tag1;
	           memory1[{index1, 2'b01}][157] = 1'b1;
	       end else if (memory1[{index1, 2'b00}][155:154] == 2'b00) begin
	           memory1[{index1, 2'b00}][127:0] <= {main_memory[{tag1, index1, 2'b11}], main_memory[{tag1, index1, 2'b10}], main_memory[{tag1, index1, 2'b01}], main_memory[{tag1, index1, 2'b00}]};
	           memory1[{index1, 2'b00}][153:128] = tag1;
	           memory1[{index1, 2'b00}][157] = 1'b1;
	       end
	   end
	end
endmodule
