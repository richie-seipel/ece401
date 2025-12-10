module riscv_core
#(
    parameter SUPPORT_MUL = 1,
    parameter SUPPORT_DIV = 1,
    parameter SUPPORT_TRAP_INVALID_OPC = 1,
    parameter SUPPORT_CSR = 1
    ,parameter SUPPORT_MTVEC    = 0
    ,parameter SUPPORT_MTVAL    = 0
    ,parameter SUPPORT_MIP_MIE  = 0
    ,parameter SUPPORT_MSCRATCH = 0
    ,parameter SUPPORT_MCYCLE   = 1
    ,parameter SUPPORT_MTIMECMP = 0
    ,parameter SUPPORT_BRAM_REGFILE = 0
    ,parameter ISR_VECTOR       = 32'h00000010

)
(
    input           clk_i,
    input           rst_i,
    input           intr_i,
    input  [31:0]   reset_vector_i,
    
    // MHARTID value
    input  [ 31:0]  cpu_id_i,

    // Instruction Memory
    output          mem_i_rd_o,
    output [31:0]   mem_i_pc_o,
    input           mem_i_accept_i,
    input           mem_i_valid_i,
    input  [31:0]   mem_i_inst_i,

    // Data Memory
    output [31:0]   mem_d_addr_o,
    output [31:0]   mem_d_data_wr_o,
    output          mem_d_rd_o,
    output [ 3:0]   mem_d_wr_o,
    input  [31:0]   mem_d_data_rd_i,
    input           mem_d_accept_i,
    input           mem_d_ack_i

);

//TODO:
// add actual branching
//      if branch taken flush
// add memory access
//      load memory access
// finish hazard detection
//      check rs1 and rs2
//      r-type and r-type (if src and destination match stall IF)
//      check just rs1
//      load and store (rs1 matches rd in later stage, stall IF, flush ID)
//      branches (operand matches rd)
//      immediate (operand matches rd)
// ensure flushing and stalling is working as intended

`include "uriscv_defs.v"

`define PC_W        32
`define ADDR_W      32

localparam           ADDR_W              = `ADDR_W;
localparam           ADDR_PAD_W          = 0;

// Execute exception (or interrupt)
wire            exception_w;
wire [5:0]      exception_type_w;
wire [31:0]     exception_target_w;

wire [31:0]     csr_mepc_w;

// CSR read data
wire [31:0]     csr_data_w;

//-----------------------------------------------------------------
// 1. Pipeline Register Definitions
//-----------------------------------------------------------------

// --- IF/ID Register ---
reg [31:0] if_id_pc_q;
reg [31:0] if_id_instr_q;
reg        if_id_valid_q;

// --- ID/EX Register ---
reg [31:0] id_ex_pc_q;
reg [31:0] id_ex_rs1_val_q;
reg [31:0] id_ex_rs2_val_q;
reg [31:0] id_ex_imm_q;
reg [4:0]  id_ex_rd_q;
reg [31:0] id_ex_opcode_q; // Carries opcode info for ALU/Branch
reg        id_ex_reg_wr_q; // Writeback control
reg        id_ex_mem_rd_q; // Memory Read control
reg        id_ex_mem_wr_q; // Memory Write control
reg        id_ex_valid_q;

// --- EX/MEM Register ---
reg [31:0] ex_mem_alu_res_q;
reg [31:0] ex_mem_store_val_q; // Data to store to memory
reg [4:0]  ex_mem_rd_q;
reg [31:0] ex_mem_opcode_q;    // For LSU
reg [31:0] ex_mem_pc_q;
reg        ex_mem_reg_wr_q;
reg        ex_mem_valid_q;
reg        ex_mem_exception_q; // store exception from csr
reg [31:0] ex_mem_mem_addr_q;
reg [31:0] ex_mem_mem_data_q;
reg ex_mem_mem_wr_q;
reg [3:0] ex_mem_mem_rd_q;
  
// --- MEM/WB Register ---
reg [31:0] mem_wb_result_q;    // Final data to write to register file
reg [4:0]  mem_wb_rd_q;
reg        mem_wb_reg_wr_q;
reg        mem_wb_valid_q;

//-----------------------------------------------------------------
// 2. Global Control Signals (Hazards / Stalls)
//-----------------------------------------------------------------
wire stall_w; // Freezes PC and IF/ID
wire flush_w; // Clears IF/ID and ID/EX (for branches)

// Implement Hazard Unit here
assign stall_w = 1'b0;
assign flush_w = 1'b0;

wire flush_if_id_w; 
wire flush_id_ex_w;
wire flush_ex_mem_w;
wire flush_mem_wb_w; 
wire stall_if_id_w; 
wire stall_id_ex_w;
wire stall_ex_mem_w;
wire stall_mem_wb_w; 

hazard_detection hazard (
    .if_id_instruction (if_id_instr_q),
    .id_ex_instruction (id_ex_instr_q),
    .ex_mem_instruction(ex_mem_instr_q),
    .mem_wb_instruction(mem_wb_instr_q),
    .flush_if_id       (flush_if_id_w),
    .flush_id_ex       (flush_id_ex_w), 
    .flush_ex_mem      (flush_ex_mem_w), 
    .flush_mem_wb      (flush_mem_wb_w),
    .stall_if_id       (stall_if_id),
    .stall_id_ex       (stall_id_ex),
    .stall_ex_mem      (stall_ex_mem),
    .stall_mem_wb      (stall_mem_wb)
);


//-----------------------------------------------------------------
// STAGE 1: INSTRUCTION FETCH (IF)
//-----------------------------------------------------------------
reg  [31:0] pc_q;
wire [31:0] pc_next_w;
wire        branch_taken_w; // From EX stage
wire [31:0] branch_target_w; // From EX stage

assign pc_next_w = branch_taken_w ? branch_target_w : (pc_q + 4);

always @(posedge clk_i) begin
    if (rst_i)          pc_q <= reset_vector_i;
    else if (!stall_w)  pc_q <= pc_next_w;
    else pc_q <= pc_q; 
end

assign mem_i_rd_o = 1'b1; // Always fetch
assign mem_i_pc_o = pc_q;

// IF/ID Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i || flush_w) begin
        if_id_instr_q  <= 32'b0; // NOP
        if_id_valid_q <= 1'b0;
        if_id_pc_q    <= 32'b0;
    end else if (!stall_w) begin
        if_id_instr_q  <= mem_i_inst_i;
        if_id_valid_q <= mem_i_valid_i; // Wait for memory valid
        if_id_pc_q    <= pc_q;
    end
end

//-----------------------------------------------------------------
// STAGE 2: INSTRUCTION DECODE (ID)
//-----------------------------------------------------------------
// Register File
reg [31:0] reg_file [0:31];
reg [31:0] id_ex_instr_q; //pass  last inst.
wire [4:0] rs1_addr_w = if_id_instr_q[19:15];
wire [4:0] rs2_addr_w = if_id_instr_q[24:20];
wire [4:0] rd_addr_w  = if_id_instr_q[11:7];

// Immediate Generation (Simplified logic from original code)
wire [31:0] imm_w;
assign imm_w = (if_id_instr_q[6:2] == 5'b01000) ? // Store
               {{20{if_id_instr_q[31]}}, if_id_instr_q[31:25], if_id_instr_q[11:7]} :
               {{20{if_id_instr_q[31]}}, if_id_instr_q[31:20]}; // Others

// Register Read (Async)
wire [31:0] reg_rs1_data_w = (rs1_addr_w == 0) ? 0 : reg_file[rs1_addr_w];
wire [31:0] reg_rs2_data_w = (rs2_addr_w == 0) ? 0 : reg_file[rs2_addr_w];

// TODO : Control Unit Decoding (Partial) 
wire is_store_w = (if_id_instr_q[6:2] == 5'b01000);
wire is_load_w  = (if_id_instr_q[6:2] == 5'b00000);
wire is_branch_w= (if_id_instr_q[6:2] == 5'b11000);

// ID/EX Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i || flush_id_ex_w) begin //  Potentially make other stages not flush
        id_ex_valid_q   <= 1'b0;
        id_ex_reg_wr_q  <= 1'b0;
        id_ex_mem_wr_q  <= 1'b0; 
        id_ex_mem_rd_q  <= 1'b0;
        id_ex_pc_q <= 1'b0; 
        id_ex_rs1_val_q <= 0; 
        id_ex_rs2_val_q <= 0; 
        id_ex_imm_q <= '0; 
        id_ex_rd_q <= 0;
        id_ex_opcode_q <= 0;
        id_ex_instr_q <= 0;
        id_ex_valid_q <= 0;
    end else if (!stall_w) begin 
        id_ex_pc_q      <= if_id_pc_q;
        id_ex_rs1_val_q <= reg_rs1_data_w; 
        id_ex_rs2_val_q <= reg_rs2_data_w;
        id_ex_imm_q     <= imm_w;
        id_ex_rd_q      <= rd_addr_w;
        id_ex_opcode_q  <= if_id_instr_q;
        id_ex_instr_q <= if_id_instr_q; //pass last val
        id_ex_valid_q   <= if_id_valid_q;
        
        // Control Signals (Simple Decoder)
        id_ex_reg_wr_q  <= (rd_addr_w != 0) && !is_store_w && !is_branch_w; 
        id_ex_mem_wr_q  <= is_store_w;
        id_ex_mem_rd_q  <= is_load_w;
    end 
end


//-----------------------------------------------------------------
// STAGE 3: EXECUTE (EX)
//-----------------------------------------------------------------
// Forwarding Muxes (Placeholder)
wire [31:0] alu_in_a_w;
wire [31:0] alu_in_b_w;
wire [31:0] forwarded_rs2_w;
reg [31:0] ex_mem_instr_q;
wire [31:0] ex_forwarded_rs1_w; 
wire [31:0] ex_forwarded_rs2_w; 


// TODO : Forwarding logic 
assign ex_forwarded_rs1_w = id_ex_rs1_val_q;
assign ex_forwarded_rs2_w = id_ex_rs2_val_q;

// TODO: Implement Forwarding Unit Here
// For now, straight from ID/EX register
// Control signal rs2_ex_fw_ex_mem, rs2_ex_fw_mem_wb
// assign forwarded_rs1_w = (rs1_ex_fw_ex_mem) ? ex_mem_alu_res_q : (rs1_ex_fw_mem_wb) ? mem_wb_result_q : id_ex_rs1_val_q
// assign forwarded_rs2_w = (rs2_ex_fw_ex_mem) ? ex_mem_alu_res_q : (rs2_ex_fw_mem_wb) ? mem_wb_result_q : id_ex_rs2_val_q
assign forwarded_rs2_w = id_ex_rs2_val_q;

// ALU Instantiation
wire [31:0] alu_result_w;
wire [3:0]  alu_op_w; 
wire write_rd_r;

// Decode ALU Op (may need a small module or logic block here to map opcode -> alu_op_w)
// Using simplified decoder logic from original core:

wire invalid_inst_r;
wire id_ex_is_muldiv_w; 

alu_decode decoder(
    .instruction_w(id_ex_opcode_q),
    .rs1_val_w(ex_forwarded_rs1_w), 
    .rs2_val_w(ex_forwarded_rs2_w), // POTENTIAL ISSUE 
    .alu_func_r(alu_op_w), 
    .csr_data_w(csr_data_w),
    .pc_q(id_ex_pc_q),
    .alu_input_a_r(alu_in_a_w), 
    .alu_input_b_r(alu_in_b_w), 
    .write_rd_r(write_rd_r),
    .inst_mul_i(inst_mul_w),
    .inst_mulh_i(inst_mulh_w),
    .inst_mulhsu_i(inst_mulhsu_w),
    .inst_mulhu_i(inst_mulhu_w),
    .inst_divu_i(inst_divu_w),
    .inst_rem_i(inst_rem_w),
    .inst_remu_i(inst_remu_w),
    .invalid_inst_i(invalid_inst_r),
    .is_muldiv_i(id_ex_is_muldiv_w)
);

//additional module for forwarding

uriscv_alu u_alu (
    .op_i(alu_op_w),
    .a_i(alu_in_a_w),
    .b_i(alu_in_b_w),
    .p_o(alu_result_w)
);

// Branch Unit Instantiation
// handle all branch instructions in module
uriscv_branch u_branch (
    .pc_i(id_ex_pc_q),
    .opcode_i(id_ex_opcode_q),
    .rs1_val_i(ex_forwarded_rs1_w),     // Must use forwarded value!
    .rs2_val_i(ex_forwarded_rs2_w), // Must use forwarded value!, can't use immediate so use forwarded reg value
    .branch_o(branch_taken_w),
    .branch_target_o(branch_target_w)
);

// Mul/div
wire muldiv_stall_w;
wire muldiv_ready_w;
wire [31:0] muldiv_result_w;


generate
if (SUPPORT_MUL || SUPPORT_DIV)
begin : g_muldiv
    uriscv_muldiv u_muldiv
    (
        .clk_i        (clk_i),
        .rst_i        (rst_i),
        .valid_i      (id_ex_valid_q && id_ex_is_muldiv_w),
        .inst_mul_i   (inst_mul_w),
        .inst_mulh_i  (inst_mulh_w),
        .inst_mulhsu_i(inst_mulhsu_w),
        .inst_mulhu_i (inst_mulhu_w),
        .inst_div_i   (inst_div_w),
        .inst_divu_i  (inst_divu_w),
        .inst_rem_i   (inst_rem_w),
        .inst_remu_i  (inst_remu_w),
        .operand_ra_i (ex_forwarded_rs1_w),
        .operand_rb_i (ex_forwarded_rs2_w),
        .stall_o      (muldiv_stall_w),
        .ready_o      (muldiv_ready_w),
        .result_o     (muldiv_result_w)
    );
end
else
begin
    assign muldiv_stall_w  = 1'b0;
    assign muldiv_ready_w  = 1'b0;
    assign muldiv_result_w = 32'b0;
end
endgenerate

wire muldiv_busy_stall_w = id_ex_valid_q && id_ex_is_muldiv_w &&
                           !muldiv_ready_w;

wire type_auipc_w   = (id_ex_instr_q[6:2] == 5'b00101);
wire type_lui_w     = (id_ex_instr_q[6:2] == 5'b01101);

// Choose final EX result
wire [31:0] ex_result_w =
    id_ex_is_muldiv_w ? muldiv_result_w :
    (type_auipc_w ? (id_ex_pc_q + id_ex_imm_q) :
     type_lui_w   ? id_ex_imm_q :
     alu_result_w);




wire mem_misaligned_w;

//-----------------------------------------------------------------
// Execute: CSR Access
//-----------------------------------------------------------------
uriscv_csr
#(
     .SUPPORT_CSR(SUPPORT_CSR)
    ,.SUPPORT_MCYCLE(SUPPORT_MCYCLE)
    ,.SUPPORT_MTIMECMP(SUPPORT_MTIMECMP)
    ,.SUPPORT_MSCRATCH(SUPPORT_MSCRATCH)
    ,.SUPPORT_MIP_MIE(SUPPORT_MIP_MIE)
    ,.SUPPORT_MTVEC(SUPPORT_MTVEC)
    ,.SUPPORT_MTVAL(SUPPORT_MTVAL)
    ,.SUPPORT_MULDIV(SUPPORT_MUL || SUPPORT_DIV)
)
u_csr
(
     .clk_i(clk_i)
    ,.rst_i(rst_i)

    // Reset vector (only used if SUPPORT_MTVEC=0)
    ,.isr_vector_i(reset_vector_i + ISR_VECTOR)

    // HartID
    ,.cpu_id_i(cpu_id_i)

    // External interrupt
    ,.intr_i(intr_i)

    // Executing instruction
    ,.valid_i(id_ex_valid_q)
    ,.opcode_i(id_ex_instr_q)
    ,.pc_i(id_ex_pc_q)
    ,.rs1_val_i(ex_forwarded_rs1_w) // TODO : forwarded value
    ,.rs2_val_i(ex_forwarded_rs2_w) // TODO : forwarded value

    // CSR read result
    ,.csr_rdata_o(csr_data_w)

    // Exception sources
    ,.excpn_invalid_inst_i(invalid_inst_r) // TODO : constrain to stage
    ,.excpn_lsu_align_i(mem_misaligned_w) //TODO : constrain to stage

    // Used on memory alignment errors
    ,.mem_addr_i(lsu_addr_w)

    // CSR registers
    ,.csr_mepc_o(csr_mepc_w)

    // Exception entry
    ,.exception_o(exception_w)
    ,.exception_type_o(exception_type_w)
    ,.exception_pc_o(exception_target_w)
);

wire [31:0] lsu_addr_w;
wire [31:0] lsu_data_w;
wire        mem_rd_req_w;
wire [3:0]  mem_wr_req_w;

// In ex stage (maybe?), use forwarded rs1 and rs2 values for address calculation 
uriscv_lsu u_lsu (
    .opcode_i(id_ex_opcode_q),
    .rs1_val_i(ex_forwarded_rs1_w), // Address is pre-calculated in ALU for simple LSU
    .rs2_val_i(ex_forwarded_rs2_w),
    .mem_rd_o(mem_rd_req_w),
    .mem_wr_o(mem_wr_req_w),
    .mem_addr_o(lsu_addr_w), // Physical memory address
    .mem_data_o(lsu_data_w), // Write data
    .mem_misaligned_o(mem_misaligned_w)
);


// EX/MEM Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i) begin
        ex_mem_mem_rd_q <= 4'b0;
        ex_mem_mem_wr_q <= 1'b0;
        ex_mem_mem_data_q <= 1'b0;
        ex_mem_mem_addr_q <= 1'b0;
        ex_mem_exception_q <= 1'b0;
        ex_mem_rd_q <= 1'b0;
        ex_mem_store_val_q <= 1'b0;
        ex_mem_pc_q <= 1'b0;
        ex_mem_alu_res_q <= 1'b0;
        ex_mem_valid_q  <= 1'b0;
        ex_mem_reg_wr_q <= 1'b0;
        ex_mem_opcode_q <= 32'b0;
        ex_mem_instr_q <= 32'b0;
    end else if (!stall_w) begin
        ex_mem_alu_res_q   <= alu_result_w;
        ex_mem_instr_q <= id_ex_instr_q; //match names
        ex_mem_pc_q        <= id_ex_pc_q;
        ex_mem_store_val_q <= forwarded_rs2_w;
        ex_mem_rd_q        <= id_ex_rd_q;
        ex_mem_opcode_q    <= id_ex_opcode_q; // Pass opcode for LSU
        ex_mem_reg_wr_q    <= id_ex_reg_wr_q;
        ex_mem_valid_q     <= id_ex_valid_q;
        ex_mem_exception_q <= exception_w; 
        ex_mem_mem_addr_q <= lsu_addr_w; 
        ex_mem_mem_data_q <= lsu_data_w; 
        ex_mem_mem_wr_q <= mem_wr_req_w; 
        ex_mem_mem_rd_q <= mem_rd_req_w; 
    end 
    //flush
end

//-----------------------------------------------------------------
// STAGE 4: MEMORY (MEM)
//-----------------------------------------------------------------

reg [31:0] mem_wb_instr_q;

//Memory interface
reg [ADDR_W-1:0] mem_addr_q;
reg [31:0]       mem_data_q;
reg [3:0]        mem_wr_q;
reg              mem_rd_q;

wire [31:0] mem_addr_w;
wire [31:0] mem_data_w;
wire [3:0] mem_wr_w;
wire mem_rd_w;

// Assign values from pipeline reg to wires 

//Real MEM stage (memory access, addr and flags calculated in EX)

// Loose decoding - gate with type_load_w on use
wire mem_inst_lb_w       = (ex_mem_instr_q[14:12] == 3'b000);
wire mem_inst_lh_w       = (ex_mem_instr_q[14:12] == 3'b001);
wire mem_inst_lbu_w      = (ex_mem_instr_q[14:12] == 3'b100);
wire mem_inst_lhu_w      = (ex_mem_instr_q[14:12] == 3'b101);

wire load_signed_q;
wire load_byte_q;
wire load_half_q;
wire [2:0] load_offset_q;

assign load_signed_q  = mem_inst_lh_w | mem_inst_lb_w;
assign load_byte_q    = mem_inst_lb_w | mem_inst_lbu_w;
assign load_half_q    = mem_inst_lh_w | mem_inst_lhu_w;
assign load_offset_q  = ex_mem_mem_addr_q[1:0];

reg [31:0] load_result_r;

//-------------------------------------------------------------------
// Load result resolve
//-------------------------------------------------------------------
always @ *
begin
    load_result_r = 32'b0;

    if (load_byte_q)
    begin
        case (load_offset_q)
            2'h3:
                load_result_r = {24'b0, mem_d_data_rd_i[31:24]};
            2'h2:
                load_result_r = {24'b0, mem_d_data_rd_i[23:16]};
            2'h1:
                load_result_r = {24'b0, mem_d_data_rd_i[15:8]};
            2'h0:
                load_result_r = {24'b0, mem_d_data_rd_i[7:0]};
        endcase

        if (load_signed_q && load_result_r[7])
            load_result_r = {24'hFFFFFF, load_result_r[7:0]};
    end
    else if (load_half_q)
    begin
        if (load_offset_q[1])
            load_result_r = {16'b0, mem_d_data_rd_i[31:16]};
        else
            load_result_r = {16'b0, mem_d_data_rd_i[15:0]};

        if (load_signed_q && load_result_r[15])
            load_result_r = {16'hFFFF, load_result_r[15:0]};
    end
    else
        load_result_r = mem_d_data_rd_i;
end


assign mem_d_addr_o    = {{ADDR_PAD_W{1'b0}}, ex_mem_mem_addr_q};
assign mem_d_data_wr_o = ex_mem_mem_data_q;
assign mem_d_wr_o      = ex_mem_mem_wr_q;
assign mem_d_rd_o      = ex_mem_mem_rd_q;


// MEM/WB Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i) begin
        mem_wb_valid_q  <= 1'b0;
        mem_wb_reg_wr_q <= 1'b0;
        mem_wb_result_q <= 1'b0;
        mem_wb_instr_q <= 1'b0;
        mem_wb_rd_q <= 1'b0;
        mem_wb_reg_wr_q <= 1'b0;
        mem_wb_valid_q <= 1'b0;
    end else if (!stall_w) begin
        // Select between Memory Data (Load) or ALU Result (Arithmetic)
        if (ex_mem_mem_rd_q) 
             mem_wb_result_q <= load_result_r; // Assumes memory is 0-cycle latency (Magic RAM)
        else 
        mem_wb_result_q <= ex_mem_alu_res_q;
        mem_wb_instr_q <= ex_mem_instr_q;
        mem_wb_rd_q     <= ex_mem_rd_q;
        mem_wb_reg_wr_q <= ex_mem_reg_wr_q;
        mem_wb_valid_q  <= ex_mem_valid_q;
    end
end


//-----------------------------------------------------------------
// STAGE 5: WRITEBACK (WB)
//-----------------------------------------------------------------
// Writing to Register File (Sync or Async depending on RAM type)
// We use synchronous write at posedge
always @(posedge clk_i) begin
    if (mem_wb_valid_q && mem_wb_reg_wr_q && (mem_wb_rd_q != 0)) begin
        reg_file[mem_wb_rd_q] <= mem_wb_result_q;
    end
    else if(rst_i) begin
        //don't write if reset
    end
    
end

endmodule
