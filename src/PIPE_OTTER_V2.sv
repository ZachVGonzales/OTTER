`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer: Zachary Gonzales
// Create Date: 02/04/2024 01:47:51 PM
// Module Name: PIPELINED_OTTER_MCU
// Description: A pipelined version of the multicyle otter from 233. Does not 
//              contain control or data hazard protection (programmer responsible)
//////////////////////////////////////////////////////////////////////////////////

module PIPE_OTTERV2(
    input RST,                // reset (button to resart program)
    input INTR,               // not used currently
    input [31:0] IOBUS_IN,    // input data
    input CLK,                // clock
    output IOBUS_WR,          // signal telling the wrapper to store the output
    output [31:0] IOBUS_OUT,  // output data
    output [31:0] IOBUS_ADDR  // address signal (input or output)
    );
    
typedef enum logic [6:0] {
    LUI      = 7'b0110111,
    AUIPC    = 7'b0010111,
    JAL      = 7'b1101111,
    JALR     = 7'b1100111,
    BRANCH   = 7'b1100011,
    LOAD     = 7'b0000011,
    STORE    = 7'b0100011,
    OP_IMM   = 7'b0010011,
    OP       = 7'b0110011,
    SYSTEM   = 7'b1110011
 } opcode_t;

typedef struct packed{
    opcode_t opcode;
    logic [31:0] srcA;
    logic [31:0] srcB;
    logic [3:0] alu_fun;
    logic [31:0] ir;
    logic [31:0] pc;
    logic [31:0] pc_next;
    logic [31:0] branch;
    logic [2:0] func3;
    logic branchEN;
    logic regWrite;
    logic memWE2;
    logic memRDEN2;
    logic [31:0] rs2;
    logic [4:0] adr1;
    logic [4:0] adr2;
    logic [4:0] rd;
    logic [1:0] rf_wr_sel;
    logic branchSrc;
    logic [31:0] result;
    logic [31:0] data;
    logic [31:0] s_imm;
    logic [31:0] fwdB;
    logic io_wr;
} instr_t;
    
//===signal=declarations====================================================
    
    //Control signals
    logic PCWrite,   // pc reg en signal
          regWrite,  // reg en signal
          memWE2,    // mem write data en signal
          memRDEN1,  // mem read instruct en
          memRDEN2,  // mem read data en
          csr_WE,    //
          int_taken, //
          mret_exec, //
          reset,     // pc counter reset
          jump;      // set high if preforming a jump instruction
    
    //CU_DCDR signals
    logic [3:0] alu_fun;    // function sel
    logic [1:0] pcSource,   // pc mux sel
                alu_srcB;   // srcB sel
    logic [1:0] rf_wr_sel;  // reg file mux sel
    logic alu_srcA;   // srcA sel
    
    //PC_MUX signals
    logic [31:0] pcDin, noBranch;  // output from pc mux
    
    //PC_REG signals
    logic [31:0] PC,       // current address being accessed by pc
                 PC_next;  // instruction directly following curent
    
    //MEMORY signals
    logic [31:0] data,  // data signal from mem
                 ir;    // instruction signal from mem 
    logic io_wr;        // write signal assigned to IOBUS_WR
                 
    //REG_FILE_MUX signals
    logic [31:0] regDin;  // output from reg file mux
    
    //REG_FILE signals
    logic [31:0] rs1,  // register 1 value
                 rs2;  // register 2 value
                 
    //IMMED_GEN signals
    logic [31:0] u_type,  // u-immediate
                 i_type,  // i-immediate
                 s_type,  // s-immediate
                 b_type,  // b-immediate
                 j_type;  // j-immediate
                 
    //BRANCH_ADDR_GEN signals
    logic [31:0] jal,     // assigned by branch addr
                 branch,  // assigned by branch addr
                 jalr,    // assigned by branch addr
                 jalr_in;    
                 
    //BRANCH_COND GEN signals
    logic branchSrc, branchEN;
                 
    //ALU_MUX_A signals
    logic [31:0] srcA_D,  // data selected by ALU mux A
                 srcA_E;  // data selected by FWD MUX A
    
    //ALU_MUX_B signals
    logic [31:0] srcB_D,  // data selected by ALU mux B
                 srcB_E;  // data selected by FWD MUX B
    
    //ALU signals
    logic [31:0] result;  //r reseult of ALU opperation on src A and B
    logic zero;
    
    //I_TYPE_MUX signals
    logic i_type_sel;
    logic [4:0] i_type_out;
    
    //U_TYPE_MUX signals
    logic u_type_sel;
    logic [4:0] u_type_out;
    
    //S_TYPE_MUX signals
    logic [31:0] fwdB;
    
    //HAZARD_UNIT signals
    logic [1:0] fwdA_sel,
                fwdB_sel,
                jalr_sel;
    logic FD_reg_en,
          DE_reg_en,
          EM_reg_en,
          MW_reg_en,
          flush_DE,
          flush_FD;
    logic cache_miss;
    

//===IF=STATE===============================================================

assign memRDEN1 = 1'b1;
assign reset = RST;
assign PC_next = PC+4;

//---PC-MUX---------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) PC_MUX  (
     .SEL   (pcSource), 
     .D0    (PC_next),         // address of next instruction
     .D1    (jalr),            // from end of DEC state
     .D2    (jal),             // from end of DEC state
     .D3    (0),               // branch not controlled here
     .D_OUT (noBranch));       // to BRANCH_MUX
     
//---BRANCH-MUX-----------------------------------------------------------
    mux_2t1_nb  #(.n(32)) BRANCH_MUX  (
     .SEL   (branchSrc), 
     .D0    (noBranch),        // if not branching use this
     .D1    (DEC_EX.branch),   // from end of EX state
     .D_OUT (pcDin));          // to PC REG

//---PC-REG-----------------------------------------------------------------
    reg_nb_sclr #(.n(32)) PC_REG (
     .data_in  (pcDin),     // input data (selected by MUX)
     .ld       (PCWrite),   // only load when not stalling
     .clk      (CLK),       //
     .clr      (reset),     // 
     .data_out (PC));       // memory address currently being accessed
     
//---MEMORY-----------------------------------------------------------------
    PIPE_CACHE PIPE_CACHE(
     .mem_clk   (CLK),
     .mem_rden1 (1'b1),               // read enable Instruction
     .mem_rden2 (EX_MEM.memRDEN2),    // read enable data
     .mem_we2   (EX_MEM.memWE2),      // write enable.
     .mem_addr1 (PC),           // Instruction Memory word Addr (Connect to PC[15:2])
     .mem_addr2 (EX_MEM.result),      // Data Memory Addr
     .mem_din2  (EX_MEM.fwdB),        // Data to save
     .mem_size  (EX_MEM.func3[1:0]),  // 0-Byte, 1-Half, 2-Word
     .mem_sign  (EX_MEM.func3[2]),    // 1-unsigned 0-signed
     .io_in     (IOBUS_IN),           // Data from IO     
     .stall     (cache_miss),         // accessing MM so need to stall pipeline (send this to hazard unit)
     .io_wr     (io_wr),              // IO 1-write 0-read
     .mem_dout1 (ir),                 // Instruction
     .mem_dout2 (data));              // Data

//==========================================================================
    
    instr_t IF_DEC;
    always_ff @(posedge CLK) begin
        // if flush then set everythin to 0 ie. nop
        if (flush_FD) begin
            IF_DEC.pc <= 0; 
            IF_DEC.pc_next <= 0;
            IF_DEC.ir <= 0;
        end
        else if (FD_reg_en) begin // only load new instruction if not stalling
            IF_DEC.pc <= PC; 
            IF_DEC.pc_next <= PC_next;
            IF_DEC.ir <= ir;
        end
    end

//===DEC=STATE==============================================================

//---DECODER----------------------------------------------------------------
    PIPE_DCDR my_cu_dcdr(
     .opcode     (IF_DEC.ir[6:0]),     // instruction type
     .func7      (IF_DEC.ir[30]),      // instruction spec
     .func3      (IF_DEC.ir[14:12]),   // instruction spec
     .alu_fun    (alu_fun),     // dcdr control signal
     .pcSource   (pcSource),    // "
     .alu_srcA   (alu_srcA),    // "
     .alu_srcB   (alu_srcB),    // "
     .rf_wr_sel  (rf_wr_sel),
     .regWrite   (regWrite),
     .memWE2     (memWE2),
     .memRDEN2   (memRDEN2),
     .branchEN   (branchEN),
     .i_type_sel (i_type_sel),
     .u_type_sel (u_type_sel),
     .jump       (jump));       // "
     
//---REG-FILE---------------------------------------------------------------
    REG_FILE_NEG_WRITE REG_FILE (
     .wd   (regDin),     //
     .clk  (CLK),               //
     .en   (MEM_WB.regWrite),   //
     .adr1 (IF_DEC.ir[19:15]),  // address of rs1
     .adr2 (IF_DEC.ir[24:20]),  // address of rs2
     .wa   (MEM_WB.rd),   // address of rd
     .rs1  (rs1),        // data in rs1
     .rs2  (rs2));       // data in rs2
     
//---I-TYPE-MUX-------------------------------------------------------------
    mux_2t1_nb  #(.n(5)) I_TYPE_MUX  (  // necessary to avoid fp data depen
     .SEL   (i_type_sel),  // dcdr control sig
     .D0    (IF_DEC.ir[24:20]),
     .D1    (0),
     .D_OUT (i_type_out)
    );
    
//---U-TYPE-MUX-------------------------------------------------------------
    mux_2t1_nb  #(.n(5)) U_TYPE_MUX  (  // necessary to avoid fp data depen
     .SEL   (u_type_sel),  // dcdr control sig
     .D0    (IF_DEC.ir[19:15]),
     .D1    (0),
     .D_OUT (u_type_out)
    );
     
//---IMMED-GEN--------------------------------------------------------------
    // assign statements representing the functionality of the IMMED_GEN:
    // Format's the instruction given from the memory module into all
    // possible immediate types.
    assign i_type = {{21{IF_DEC.ir[31]}}, IF_DEC.ir[30:25], IF_DEC.ir[24:20]};
    assign s_type = {{21{IF_DEC.ir[31]}}, IF_DEC.ir[30:25], IF_DEC.ir[11:7]};
    assign b_type = {{20{IF_DEC.ir[31]}}, IF_DEC.ir[7], IF_DEC.ir[30:25], IF_DEC.ir[11:8], 1'b0};
    assign u_type = {IF_DEC.ir[31:12], {12{1'b0}}};
    assign j_type = {{12{IF_DEC.ir[31]}}, IF_DEC.ir[19:12], IF_DEC.ir[20], IF_DEC.ir[30:21], 1'b0};
    
//---JALR-MUX---------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) JALR_MUX  (
     .SEL  (jalr_sel),
     .D0   (rs1),
     .D1   (result),
     .D2   (EX_MEM.result),
     .D3   (MEM_WB.result),
     .D_OUT (jalr_in)
    );
    
//---BRANCH-ADDR-GEN--------------------------------------------------------
    // assign statements representing the funcitonality of the BRANCH_ADDR_GEN:
    // Creates the jump / branch signals using the respective immediate values
    // and either the current instruction address or the source register (rs1).
    assign jal = IF_DEC.pc + j_type; 
    assign jalr = jalr_in + i_type;
    assign branch = IF_DEC.pc + b_type;
     
//---ALU-IN-A---------------------------------------------------------------
    mux_2t1_nb  #(.n(32)) ALU_MUX_A  (
     .SEL   (alu_srcA),  //
     .D0    (rs1),       // from reg file
     .D1    (u_type),    // from immed gen
     .D_OUT (srcA_D));     // to ALU
     
//---ALU-IN-B---------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) ALU_MUX_B  (
     .SEL   (alu_srcB), 
     .D0    (rs2),     // from reg file
     .D1    (i_type),  // from immed gen
     .D2    (0),       // no longer need s-type imm passed here since goes
                       // to pipe reg now
     .D3    (IF_DEC.pc),      // from pc
     .D_OUT (srcB_D));   // to ALU
     
//==========================================================================

    instr_t DEC_EX;
    always_ff @(posedge CLK) begin
        // if we are stalling for load: load no-op into execute register
        // otherwise proceed as normal
        if (flush_DE) begin
            DEC_EX.pc_next <= 0;
            DEC_EX.srcA <= 0;
            DEC_EX.srcB <= 0;
            DEC_EX.alu_fun <= 0;
            DEC_EX.branch <= 0;
            DEC_EX.func3 <= 0;
            DEC_EX.branchEN <= 0;
            DEC_EX.regWrite <= 0;
            DEC_EX.memWE2 <= 0;
            DEC_EX.memRDEN2 <= 0;
            DEC_EX.rs2 <= 0;
            DEC_EX.adr1 <= 0;
            DEC_EX.adr2 <= 0;
            DEC_EX.rd <= 0;
            DEC_EX.rf_wr_sel <= 0;
            DEC_EX.s_imm <= 0;
        end
        else if (DE_reg_en) begin
            DEC_EX.pc_next <= IF_DEC.pc_next;
            DEC_EX.srcA <= srcA_D;
            DEC_EX.srcB <= srcB_D;
            DEC_EX.alu_fun <= alu_fun;
            DEC_EX.branch <= branch;
            DEC_EX.func3 <= IF_DEC.ir[14:12];
            DEC_EX.branchEN <= branchEN;
            DEC_EX.regWrite <= regWrite;
            DEC_EX.memWE2 <= memWE2;
            DEC_EX.memRDEN2 <= memRDEN2;
            DEC_EX.rs2 <= rs2;
            DEC_EX.adr1 <= u_type_out;
            DEC_EX.adr2 <= i_type_out;
            DEC_EX.rd <= IF_DEC.ir[11:7];
            DEC_EX.rf_wr_sel <= rf_wr_sel;
            DEC_EX.s_imm <= s_type;
        end
    end

//===EX=STATE===============================================================

//---BRANCH-COND-GEN--------------------------------------------------------
    PIPE_BRANCH_GEN pipe_branch_gen(
     .zero      (zero),
     .result    (result),
     .func3     (DEC_EX.func3),
     .branchEN  (DEC_EX.branchEN),
     .branchSrc (branchSrc));
     
//---FWD-A-MUX--------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) FWD_A_MUX  (
     .SEL   (fwdA_sel), 
     .D0    (DEC_EX.srcA),    // dec / ex reg
     .D1    (EX_MEM.result),  // ex / mem reg
     .D2    (regDin),  // mem / wb reg
     .D3    (),               // unused output
     .D_OUT (srcA_E));        // to ALU
     
//---FWD-B-MUX--------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) FWD_B_MUX  (
     .SEL   (fwdB_sel), 
     .D0    (DEC_EX.srcB),    // dec / ex reg
     .D1    (EX_MEM.result),  // ex / mem reg
     .D2    (regDin),  // mem / wb reg
     .D3    (),               // unused output
     .D_OUT (fwdB));        // to ALU
     
//---S-TYPE-MUX-------------------------------------------------------------
    mux_2t1_nb  #(.n(32)) S_TYPE_MUX  (
     .SEL   (DEC_EX.memWE2), 
     .D0    (fwdB),    // dec / ex reg
     .D1    (DEC_EX.s_imm),  // ex / mem reg
     .D_OUT (srcB_E));        // to ALU
     
//---ALU--------------------------------------------------------------------
    PIPE_ALU ALU (
     .srcA    (srcA_E),     //
     .srcB    (srcB_E),     //
     .alu_fun (DEC_EX.alu_fun),  // selected by dcdr
     .result  (result),
     .zero    (zero));  // output to various modules and IOBUS_ADDR

//---Hazard-Unit------------------------------------------------------------
    HAZARD_UNIT hazard_unit (
     .rs1_D      (IF_DEC.ir[19:15]),
     .rs2_D      (IF_DEC.ir[24:20]),
     .rs1_E      (DEC_EX.adr1),
     .rs2_E      (DEC_EX.adr2),
     .rd_E       (DEC_EX.rd),
     .rd_M       (EX_MEM.rd),
     .rd_W       (MEM_WB.rd),
     .regWrite_M (EX_MEM.regWrite),
     .regWrite_W (MEM_WB.regWrite),
     .memRDEN2_E (DEC_EX.memRDEN2),
     .branchEN   (branchSrc),
     .jumpEN     (jump),
     .cache_miss (cache_miss),
     .FA_SEL     (fwdA_sel),
     .FB_SEL     (fwdB_sel),
     .JALR_SEL   (jalr_sel),
     .FD_reg_en  (FD_reg_en),
     .DE_reg_en  (DE_reg_en),
     .EM_reg_en  (EM_reg_en),
     .MW_reg_en  (MW_reg_en),
     .flush_DE   (flush_DE),
     .flush_FD   (flush_FD),
     .PC_en      (PCWrite));
     
//==========================================================================

    instr_t EX_MEM;
    always_ff @(posedge CLK) begin
        if (EM_reg_en) begin
            EX_MEM.pc_next <= DEC_EX.pc_next;
            EX_MEM.regWrite <= DEC_EX.regWrite;
            EX_MEM.memWE2 <= DEC_EX.memWE2;
            EX_MEM.memRDEN2 <= DEC_EX.memRDEN2;
            EX_MEM.result <= result;
            EX_MEM.rs2 <= DEC_EX.rs2;
            EX_MEM.func3 <= DEC_EX.func3;
            EX_MEM.rd <= DEC_EX.rd;
            EX_MEM.rf_wr_sel <= DEC_EX.rf_wr_sel;
            EX_MEM.fwdB <= fwdB;
        end
    end

//===MEM=STATE==============================================================

// memory module is a dual port single module that controls both data and
// instruction memory, so the instantiation is placed in the IF state but
// the signals for the data ports come from the EX_MEM pipeline register.

//==========================================================================

    instr_t MEM_WB;
    always_ff @(posedge CLK) begin
        if (MW_reg_en) begin
            MEM_WB.pc_next <= EX_MEM.pc_next;
            MEM_WB.regWrite <= EX_MEM.regWrite;
            MEM_WB.result <= EX_MEM.result;
            MEM_WB.rd <= EX_MEM.rd;
            MEM_WB.data <= data;
            MEM_WB.rf_wr_sel <= EX_MEM.rf_wr_sel;
            MEM_WB.io_wr <= io_wr;
            MEM_WB.fwdB <= EX_MEM.fwdB;
        end
    end

//===WB=STATE===============================================================
    
//---REG-MUX----------------------------------------------------------------
    mux_4t1_nb  #(.n(32)) REG_FILE_MUX  (
     .SEL   (MEM_WB.rf_wr_sel),   // controlled by dcdr
     .D0    (MEM_WB.pc_next),     // next instruct addr 
     .D1    (0),           // not using interupts
     .D2    (MEM_WB.data),        // 
     .D3    (MEM_WB.result),      //
     .D_OUT (regDin));     // 
     
// REG_FILE module is instantiated in the DEC STATE but is also updated 
// here in the WB state
    
//==========================================================================
     
    assign IOBUS_OUT = MEM_WB.fwdB;      // assign stmts for outputs
    assign IOBUS_ADDR = MEM_WB.result;  // 
    assign IOBUS_WR = MEM_WB.io_wr;     //
    
/*
old memory instantiation (incase i need it again)
    Memory_Neg_Read OTTER_MEMORY (
     .MEM_CLK   (CLK),
     .MEM_RDEN1 (1'b1),  // currently always reading from instr mem
     .MEM_RDEN2 (EX_MEM.memRDEN2),  // "
     .MEM_WE2   (EX_MEM.memWE2),    // "
     .MEM_ADDR1 (PC[15:2]),    // instruct addres
     .MEM_ADDR2 (EX_MEM.result),            // data address
     .MEM_DIN2  (EX_MEM.fwdB),            // data being written to data mem
     .MEM_SIZE  (EX_MEM.func3[1:0]), // 
     .MEM_SIGN  (EX_MEM.func3[2]),    //
     .IO_IN     (IOBUS_IN),  // input from I/O
     .IO_WR     (io_wr),     // I/O write signal
     .MEM_DOUT1 (ir),        // instruction 
     .MEM_DOUT2 (data));     // data
*/
endmodule
