`timescale 1ns/1ps

`define FF 1
// Packed structures for pipeline registers
// COMPLETE THE PIPELINE INTERFACES USING PACKED STRUCTURES
// Pipe reg: IF/ID
typedef struct packed {
    logic   [31:0]  pc;
    logic   [31:0]  inst;
} pipe_if_id;

// Pipe reg: ID/EX
typedef struct packed {
    logic   [31:0]  pc;
    logic   [31:0]  rs1_dout;
    logic   [31:0]  rs2_dout;
    logic   [31:0]  imm32;
    logic   [31:0]  imm32_branch;
    logic   [2:0]   funct3;
    logic   [6:0]   funct7;
    logic   [5:0]   branch;     // branch[0] = beq, branch[1] = bne, branch[2] = blt, branch[3] = bge
    logic           alu_src;
    logic   [1:0]   alu_op;
    logic           mem_read;
    logic           mem_write;
    logic   [6:0]   opcode;
    logic   [4:0]   rs1;
    logic   [4:0]   rs2;
    logic   [4:0]   rd;         // rd for regfile
    logic           reg_write;
    logic           mem_to_reg;
} pipe_id_ex;

// Pipe reg: EX/MEM
typedef struct packed {
    logic   [31:0]  pc;
    logic   [31:0]  alu_result; // for address
    logic   [31:0]  rs2_dout;   // for store
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
    logic   [6:0]   opcode ;
    logic   [2:0]   funct3;
 //   logic         imm32;
} pipe_ex_mem;

// Pipe reg: MEM/WB
typedef struct packed {
    logic   [31:0]  pc;
    logic   [31:0]  alu_result;
    logic   [31:0]  dmem_dout;
    logic   [6:0]   opcode;
    logic   [2:0]   funct3;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_mem_wb;

/* verilator lint_off UNUSED */
module pipeline_cpu
#(  parameter   IMEM_DEPTH      = 1024  ,    // imem depth (default: 1024 entries = 4 KB)
                IMEM_ADDR_WIDTH = 10    ,
                REG_WIDTH       = 32    ,
                DMEM_DEPTH      = 1024  ,    // dmem depth (default: 1024 entries = 8 KB)
                DMEM_ADDR_WIDTH = 10 )
(
    input           clk                 ,            // System clock
    input           reset_b                          // Asychronous negative reset
);
 // Program counter
    logic                       pc_write                        ;   // enable PC updates
    logic       [31:0]          pc_curr, pc_next                ;
    logic       [31:0]          pc_next_plus4, pc_next_branch   ;
    logic                       pc_next_sel                     ;
    logic                       branch_taken                    ;
    //logic           regfile_zero;   // zero detection from regfile, REMOVED

    assign      pc_next_plus4 = pc_curr + 4                     ;
    assign      pc_next_sel = branch_taken                      ;  // conditional jump, unconditional jump
    assign      pc_next = (pc_next_sel == 1'b1) ?  pc_next_branch :  pc_next_plus4      ;

    // program counter
    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            if (pc_write == 1'b1) pc_curr <= pc_next    ;
        end
    end

    // imem
    logic       [IMEM_ADDR_WIDTH-1:0]   imem_addr       ;
    logic       [31:0]                  inst            ;   // instructions = an output of instruction memory

    assign      imem_addr = pc_curr[IMEM_ADDR_WIDTH+1:2];  // 10bit

    // instantiation: instruction memory
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr ),
        .dout               (   inst   )
    );
  
  /* IF/ID pipeline register
     * - Supporting pipeline stalls and flush
     */
    pipe_if_id      id  ;         // THINK WHY THIS IS ID...

    logic           if_flush, if_stall;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            id <= 'b0                   ;
        end
        else begin
            if (  if_flush == 1'b1 ) begin  // flush
                id <= 'b0               ; // nop
            end
            else if (if_stall == 1'b0 ) begin // nothing happen
                    id.pc <= pc_curr    ;      //
                    id.inst <= inst     ;  //
            end
            // if if_stall, read same inst in if stage and read same rs1, rs2 in id
        end
    end
  /* Main control */
    logic   [6:0]   opcode              ;
    logic   [5:0]   branch              ;
    logic           alu_src             ;
    logic           mem_to_reg          ;
    logic   [1:0]   alu_op              ;
    logic           mem_read            ;
    logic           mem_write           ;
    logic           reg_write           ; // declared above
    logic   [6:0]   funct7              ;
    logic   [2:0]   funct3              ;

    // COMPLETE THE MAIN CONTROL UNIT HERE

    assign              opcode    = id.inst[6:0]                                                        ;
    assign              branch[0] = ( opcode == 7'b1100011 && funct3 == 3'b000 ) ? 1'b1 : 1'b0          ;       // BEQ
    assign              branch[1] = ( opcode == 7'b1100011 && funct3 == 3'b001 ) ? 1'b1 : 1'b0          ;       // BNE
    assign              branch[2] = ( opcode == 7'b1100011 && funct3 == 3'b100 ) ? 1'b1 : 1'b0          ;       // BLT
    assign              branch[3] = ( opcode == 7'b1100011 && funct3 == 3'b101 ) ? 1'b1 : 1'b0          ;       // BGE
    assign              branch[4] = ( opcode == 7'b1100011 && funct3 == 3'b110 ) ? 1'b1 : 1'b0          ;       // BLTU
    assign              branch[5] = ( opcode == 7'b1100011 && funct3 == 3'b111 ) ? 1'b1 : 1'b0          ;       // BGEU
  
    assign              mem_read   = ( opcode == 7'b0000011 )  ? 1'b1 : 1'b0                    ;    // lw
    assign              mem_write  = ( opcode == 7'b0100011 )  ? 1'b1 : 1'b0                    ;    // sw
    assign              mem_to_reg = ( opcode == 7'b0000011 )  ? 1'b1 : 1'b0                    ;    // lw
   // assign              reg_write  = ( opcode == 7'b1100011 )  ? 1'b0 : 1'b1                  ;   // except sw
    assign              reg_write  = ( opcode == 7'b0000011 || opcode == 7'b0110011 || opcode == 7'b0010011|| opcode == 7'b1100011||opcode==7'b1100111||opcode==7'b1100111||opcode==7'b0110111||opcode==7'b0010111) ? 1: 0;
   // assign              alu_src    = ( opcode == 7'b0110011 || opcode == 7'b1100011 ) ? 1'b0 : 1'b1  ;   // excpet r-type, beq
    assign              alu_src = (opcode == 7'b0000011||opcode==7'b0100011||opcode==7'b0010011||opcode==7'b1100111||opcode==7'b1101111||opcode==7'b0110111||opcode==7'b0010111) ? 1:0;

    always_comb begin
            if      (opcode == 7'b0000011) alu_op = 2'b00                           ; // lw
            else if (opcode == 7'b0100011) alu_op = 2'b00                           ; // sw
            else if (opcode == 7'b1100011) alu_op = 2'b01                           ; // sb-type
            else if (opcode == 7'b0110011) alu_op = 2'b10                           ; // r-type
            else if (opcode == 7'b0010011) alu_op = 2'b10                           ; // i-type
            else if (opcode == 7'b0110111) alu_op = 2'b11                           ; // lui
            else                           alu_op = 2'b00                           ; // auipc, jal, jalr
    end

    assign              funct3 = (opcode == 7'b0110111 || opcode == 7'b0010111) ? 32'b0 : id.inst[14:12]        ;
    assign              funct7 = (opcode == 7'b0110011) ? id.inst[31:25] : 0                                    ;
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic       [31:0]          imm32           ;
    logic       [31:0]          imm32_branch    ;  // imm32 left shifted by 1

    // COMPLETE IMMEDIATE GENERATOR HERE
    logic       [11:0]          imm12           ;
    logic       [19:0]          imm20           ;

    always_comb begin
            if (opcode == 7'b0000011) imm12[11:0] = id.inst[31:20] ; // load
            else if (opcode == 7'b0100011) begin                  // store
                    imm12[11:5] = id.inst[31:25]   ;
                    imm12[4:0]  = id.inst[11:7]    ;
            end
            else if (opcode == 7'b0010011) begin                  // i-type
                    imm12[11:0] = id.inst[31:20]   ;
            end
            else if (opcode == 7'b1100011) begin                      // sb-type
                    imm12[11] = id.inst[31]        ;
                    imm12[9:4] = id.inst[30:25]    ;
                    imm12[3:0] = id.inst[11:8]     ;
                    imm12[10] = id.inst[7]         ;
            end
            else if (opcode == 7'b0110111) imm20 = id.inst[31:12]; // lui
            else if (opcode == 7'b0010111) imm20 = id.inst[31:12]; // auipc
            else if (opcode == 7'b1101111) begin                //jal
                    imm20[19] = id.inst[31]         ;
                    imm20[9:0] = id.inst[30:21]     ;
                    imm20[10] = id.inst[20]         ;
                    imm20[18:11] = id.inst[19:12]   ;
            end
            else if (opcode == 7'b1100111) begin               // jalr
                    imm12[11:0] = id.inst[31:20]    ;

            end
    end
  
    always_comb begin
            if (opcode == 7'b0000011)                           imm32 ={{20{imm12[11]}},imm12};  // load
            else if (opcode == 7'b0100011)                      imm32 = {{20{imm12[11]}},imm12}; // store
            else if (opcode == 7'b0010011) begin         // i-type
                        if (funct3 == 3'b011)   imm32 = {{20{1'b0}},imm12};  //  sltiu
                        else imm32 ={{20{imm12[11]}},imm12};
            end
            else if (opcode == 7'b1100011) begin        // sb-type
                if (funct3 == (3'b110|3'b111)) imm32 = {{20{1'b0}},imm12};   // bltu, bgeu
                else                           imm32 = {{20{imm12[11]}},imm12};
            end
            else if (opcode == 7'b0110111)                               imm32 = {imm20,{12{1'b0}}};  // lui
            else if (opcode == 7'b0010111)                               imm32 = {imm20,{12{1'b0}}};  // auipc
            else if (opcode == 7'b1101111)                               imm32 = {{12{imm20[19]}},imm20}; // jal
            else if (opcode == 7'b1100111)                               imm32 = {{20{imm12[11]}},imm12}; // jalr
            else                                                         imm32 = 32'b0  ;
    end

    assign      imm32_branch = {imm32[30:0], 1'b0}; // << 1 for branch

    // Computing branch target
    always_comb begin
                    if (opcode == 7'b1100111) pc_next_branch = mem.alu_result                   ; // jalr
                    else if (opcode == 7'b1101111) pc_next_branch = id.pc + imm32_branch        ; // jal
                    else if (opcode == 7'b1100011) pc_next_branch = id.pc + imm32_branch        ; // sb-type
    end
    
  logic   [4:0]   rs1, rs2, rd        ;

    logic           stall_by_load_use   ;//pc, id fix
    logic           flush_by_branch     ;
    logic           id_stall, id_flush  ;

    assign      rs1 = (opcode == 7'b0000011 || opcode == 7'b0110011 || opcode == 7'b0010011 || opcode == 7'b0100011 || opcode == 7'b1100011 || opcode == 7'b1100111|| opcode == 7'b1100111) ? id.inst[19:15] : 5'b0     ; // load, store, r-type, i-type, sb-type, jalr
    assign      rs2 = (opcode == 7'b0100011 || opcode == 7'b0110011 || opcode == 7'b1100011) ? id.inst[24:20] : 5'b0    ; // store, r-type, sb-type
    assign      rd  = (opcode == 7'b0000011 || opcode == 7'b0110011 || opcode == 7'b0010011 || opcode == 7'b0110111 || opcode ==  7'b0010111 || opcode ==  7'b1100111 || opcode == 7'b1101111) ? id.inst[11:7] : 5'b0   ;
//    assign    rs1 = id.inst[19:15]    ;
//    assign    rs2 = id.inst[24:20]    ;
//    assign    rd = id.inst[11:7]      ;

    assign      stall_by_load_use =  (ex.mem_read == 1'b1) && ((ex.rd == rs1) || (ex.rd == rs2)) ? 1'b1 : 1'b0  ; // hazard detected, 1 clock delay & control signal become 0
    assign      flush_by_branch = branch_taken ;

    assign      id_flush =  flush_by_branch     ;
    assign      id_stall =  stall_by_load_use   ;

    assign      if_flush =  flush_by_branch     ;
    assign      if_stall =  stall_by_load_use   ;// if id stall, if stall too
    assign      pc_write =  ((if_stall) | (if_flush))  ? 1'b0 : 1'b1  ;
  
   // regfile/

    logic       [REG_WIDTH-1:0]         rd_din                  ;
    logic       [REG_WIDTH-1:0]         rs1_dout, rs2_dout      ;

  //  assign    rs1 =  id.inst[19:15]           ;     // our processor does NOT support U and UJ types
   // assign    rs2 =  (opcode != 7'b0000011 || opcode != 7'b0010011) ? id.inst[24:20] : 5'b00000       ;     // consider ld and i-type
    // rd, rd_din, and reg_write will be determined in WB stage

    // instnatiation of register file
    regfile #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_regfile_0 (
        .clk                ( clk )             ,
        .rs1                ( rs1 )             ,
        .rs2                ( rs2 )             ,
        .rd                 ( wb.rd )           , // for imporove load
        .rd_din             ( rd_din )          ,
        .reg_write          ( wb.reg_write )    ,
        .rs1_dout           ( rs1_dout )        ,
        .rs2_dout           ( rs2_dout )
    );
  /* ID/EX pipeline register
     * - Supporting pipeline stalls
     */
    pipe_id_ex      ex;         // THINK WHY THIS IS EX...

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
                ex <= 'b0;
        end
        else if (id_flush == 1'b1) begin
                ex <= 'b0;
        end
        else if (id_stall == 1'b1) begin // 1 clock delay and control unit become nop
                ex.branch <= 6'b0;
                ex.reg_write <= 1'b0; // necessary
                ex.mem_read <= 1'b0;
                ex.mem_write <= 1'b0;  // necessary
                ex.mem_to_reg <= 1'b0;
                ex.alu_op <= 2'b00;
                ex.alu_src <= 1'b0;
                ex.rd <= rd;
        end
        else begin // nothing happen
                ex.pc           <= id.pc        ;
                ex.rs1_dout     <= rs1_dout     ;
                ex.rs2_dout     <= rs2_dout     ;//for store
                ex.imm32        <= imm32        ;
                ex.imm32_branch <= imm32_branch ;
                ex.opcode       <= opcode       ;
                ex.funct3       <= funct3       ;
                ex.funct7       <= funct7       ;
                ex.rs1          <= rs1          ;
                ex.rs2          <= rs2          ;
                ex.rd           <= rd           ;
                ex.reg_write    <= reg_write    ;
                ex.mem_to_reg   <= mem_to_reg   ;
                ex.branch       <= branch       ;
                ex.alu_src      <= alu_src      ;
                ex.alu_op       <= alu_op       ;
                ex.mem_read     <= mem_read     ;
                ex.mem_write    <= mem_write    ;
        end
    end
  logic       [3:0]   alu_control     ;    // ALU control signal

    // COMPLETE ALU CONTROL UNIT

    always_comb begin
        if (ex.alu_op == 2'b00)                                 alu_control = 4'b0010   ; // add - etc
        else if (ex.alu_op == 2'b01)                            alu_control = 4'b0111   ; // sub - sb-type
        else if (ex.alu_op == 2'b11)                            alu_control = 4'b1010   ; // lui
        else if (ex.alu_op == 2'b10) begin // i-type(except jalr),r-type
                if (ex.funct3 == 3'b001)                        alu_control = 4'b0100   ; // sll, slli
                else if (ex.funct3 == 3'b101 && ex.funct7 == 7'b0000000) alu_control = 4'b0101; // srl, srli
                else if (ex.funct3 == 3'b101 && ex.funct7 == 7'b0100000) alu_control = 4'b0110; // sra, srai
                else if (ex.funct3 == 3'b000 ) begin
                        if (ex.funct7 == 7'b0100000)            alu_control = 4'b0111; // sub
                        else                                    alu_control = 4'b0010   ;       // add, addi
                end
                else if (ex.funct3 == 3'b100)                   alu_control = 4'b0011   ; // xor, xori
                else if (ex.funct3 == 3'b110)                   alu_control = 4'b0001   ; // or, ori
                else if (ex.funct3 == 3'b111)                   alu_control = 4'b0000   ; // and, andi
                else if (ex.funct3 == 3'b010)                   alu_control = 4'b1000   ; // slt, slti
                else if (ex.funct3 == 3'b011)                   alu_control = 4'b1001   ; // sltu, sltiu
        end
    end
  
  /* Forwarding unit:
     * - Forwarding from EX/MEM and MEM/WB
     */
    logic       [1:0]   forward_a       ;
    logic       [1:0]   forward_b       ;
    logic   [REG_WIDTH-1:0]  alu_fwd_in1, alu_fwd_in2;   // outputs of forward MUXes

        /* verilator lint_off CASEX */
   // COMPLETE FORWARDING MUXES
    always_comb begin
            if      (forward_a == 2'b00) alu_fwd_in1 = ex.rs1_dout      ;
            else if (forward_a == 2'b10) alu_fwd_in1 = mem.alu_result   ;
            else if (forward_a == 2'b01) alu_fwd_in1 = (mem.mem_to_reg == 1'b1) ? wb.dmem_dout : (mem.reg_write == 1'b1) ? wb.alu_result : 32'b0 ;

            if      (forward_b == 2'b00) alu_fwd_in2 = ex.rs2_dout      ;
            else if (forward_b == 2'b10) alu_fwd_in2 = mem.alu_result   ;
            else if (forward_b == 2'b01) alu_fwd_in2 = (mem.mem_to_reg == 1'b1) ? wb.dmem_dout : (mem.reg_write == 1'b1) ? wb.alu_result : 32'b0 ;
    end

        /* verilator lint_on CASEX */
        // COMPLETE THE FORWARDING UNIT
    // Need to prioritize forwarding conditions
    always_comb begin
         if(mem.reg_write == 1'b1 && mem.rd != 5'b0 && mem.rd == ex.rs1) forward_a = 2'b10; // ex hazard
         else if (wb.reg_write == 1'b1 && (wb.rd != 5'b0) && !(mem.reg_write == 1'b1 && (mem.rd != 5'b0) && (mem.rd == ex.rs1))&&(wb.rd == ex.rs1)) forward_a = 2'b01; // mem hazard
         else                   forward_a = 2'b00;
    end

    always_comb begin
         if(mem.reg_write == 1'b1 && mem.rd != 5'b0 && mem.rd == ex.rs2) forward_b = 2'b10; // ex hazard
         else if (wb.reg_write == 1'b1 && (wb.rd !=5'b0) && !(mem.reg_write == 1'b1 && (mem.rd != 5'b0) && (mem.rd == ex.rs2))&&(wb.rd == ex.rs2)) forward_b = 2'b01; // mem hazard
         else                   forward_b = 2'b00;
    end
  
    // ALU
    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [REG_WIDTH-1:0] alu_result;
    //logic           alu_zero;   // will not be used

    assign      alu_in1 = (ex.opcode == 7'b0010111 || ex.opcode == 7'b1101111) ? ex.pc : (ex.opcode == 7'b0110111) ? 32'b0 :  alu_fwd_in1       ;
    assign      alu_in2 =  (ex.alu_src == 1'b1 && ex.opcode != 7'b0010111) ? ex.imm32 : (ex.alu_src == 1'b1 && ex.opcode == 7'b0010111) ? ex.imm32_branch : alu_fwd_in2 ;

    // instantiation: ALU
    alu #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_alu_0 (
        .in1                (alu_in1),
        .in2                (alu_in2),
        .alu_control        (alu_control),
        .result             (alu_result)
    );

    // branch unit (BU) -> branch result are not determined in id, it is determined at exe.
    logic       [REG_WIDTH-1:0]         sub_for_branch                          ;
    logic                               bu_zero                                 ;
    logic                               bu_sign                                 ;
   // logic                             branch_taken                            ;

    assign      sub_for_branch =  alu_result    ;
    assign      bu_zero =  (sub_for_branch == 32'b0) ? 1'b1 : 1'b0      ;
    assign      bu_sign =  (sub_for_branch[31])                         ;

    always_comb begin
            if      (ex.branch[0] == 1'b1 && bu_zero == 1'b1)                   branch_taken = 1'b1     ; // beq
            else if (ex.branch[1] == 1'b1 && bu_zero == 1'b0)                   branch_taken = 1'b1     ; // bne
            else if (ex.branch[2] == 1'b1 && bu_zero == 1'b0 && bu_sign == 1'b1)        branch_taken = 1'b1     ; // blt
            else if (ex.branch[3] == 1'b1 && bu_sign == 1'b0)                   branch_taken = 1'b1     ;  // bge
            else if (ex.branch[4] == 1'b1 && bu_zero == 1'b0 && bu_sign == 1'b1)        branch_taken = 1'b1     ; // bltu
            else if (ex.branch[5] == 1'b1 && bu_sign == 1'b0)                   branch_taken = 1'b1     ; // bgeu
            else if (ex.opcode == 7'b1101111 || ex.opcode == 7'b1100111)                branch_taken = 1'b1     ; // jal, jalr
            else                                                                branch_taken = 1'b0     ;
    end
  
    /* Ex/MEM pipeline register
     */
    pipe_ex_mem     mem         ;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
           mem <= 'b0;
        end else begin
                mem.pc          <= ex.pc                ;
                mem.alu_result  <= alu_result           ;
                mem.rs2_dout    <= alu_fwd_in2          ;
                mem.opcode      <= ex.opcode            ;
                mem.funct3      <= ex.funct3            ;
                mem.rd          <= ex.rd                ; // for load
                mem.mem_read    <= ex.mem_read          ;
                mem.mem_write   <= ex.mem_write         ;
                mem.reg_write   <= ex.reg_write         ;
                mem.mem_to_reg  <= ex.mem_to_reg        ;
        end
    end
  /* Memory stage
     * - Data memory accesses
     */

    // dmem
    logic       [DMEM_ADDR_WIDTH-1:0]           dmem_addr               ;
    logic       [31:0]                          dmem_din, dmem_dout     ;

    assign      dmem_addr = mem.alu_result[11:2]                ;

    always_comb begin
            if (mem.mem_write)begin
                    if      (mem.funct3 == 3'b000)  dmem_din = {24'b0, mem.rs2_dout[7:0]}       ; //sb
                    else if (mem.funct3 == 3'b001)  dmem_din = {16'b0, mem.rs2_dout[15:0]}      ; // sh
                    else if (mem.funct3 == 3'b010)  dmem_din = mem.rs2_dout                     ; //sw
            end
    end    // if instruction is store, the processs end.


    // instantiation: data memory
    dmem #(
        .DMEM_DEPTH         (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH    (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk                (clk),
        .addr               (dmem_addr),
        .din                (dmem_din),
        .mem_read           ( mem.mem_read ),
        .mem_write          ( mem.mem_write ),
        .dout               (dmem_dout)
    );
  
  // -----------------------------------------------------------------------
    /* MEM/WB pipeline register
     */

    pipe_mem_wb         wb      ;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            wb <= 'b0;
        end else begin
             wb.pc              <= mem.pc               ;
             wb.opcode          <= mem.opcode           ;
             wb.funct3          <= mem.funct3           ;
             wb.alu_result      <= mem.alu_result       ; // for forwarding
             wb.dmem_dout       <= dmem_dout            ;
             wb.rd              <= mem.rd               ;// for load
             wb.reg_write       <= mem.reg_write        ;
             wb.mem_to_reg      <= mem.mem_to_reg       ;
        end
    end
  
  /* Writeback stage
     * - Write results to regsiter file
     */
    always_comb begin
           if (wb.mem_to_reg) begin
                    if      (wb.funct3 == 3'b000) rd_din = {{24{wb.dmem_dout[7]}},wb.dmem_dout[7:0]}    ;  // lb
                    else if (wb.funct3 == 3'b001) rd_din = {{16{wb.dmem_dout[15]}},wb.dmem_dout[15:0]}   ; // lh
                    else if (wb.funct3 == 3'b010) rd_din = wb.dmem_dout                 ; // lw
                    else if (wb.funct3 == 3'b100) rd_din = {24'b0, wb.dmem_dout[7:0]}   ; // lbu
                    else if (wb.funct3 == 3'b101) rd_din = {16'b0, wb.dmem_dout[15:0]}  ; // lhu
          end
         // else (wb.reg_write) begin
                    if      (wb.opcode == 7'b0110011) rd_din = wb.alu_result    ; // r-type
                    else if (wb.opcode == 7'b0010011) rd_din = wb.alu_result    ; // i-type
                    else if (wb.opcode == 7'b1100011) rd_din = wb.pc + 4        ; // sb-type
                    else if (wb.opcode == 7'b1101111) rd_din = wb.pc + 4        ; // jal
                    else if (wb.opcode == 7'b1100111) rd_din = wb.pc + 4        ; // jalr
                    else if (wb.opcode == 7'b0110111) rd_din = wb.alu_result    ; // lui
                    else if (wb.opcode == 7'b0010111) rd_din = wb.alu_result    ; // auipc
          // end
    end


endmodule
  
  
  
