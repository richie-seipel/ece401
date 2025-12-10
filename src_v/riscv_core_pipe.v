module riscv_core_pipelined
#(
    parameter SUPPORT_MUL = 1,
    parameter SUPPORT_DIV = 1,
    parameter SUPPORT_CSR = 1 // Simplified for skeleton: CSRs usually live in ID or WB
)
(
    input           clk_i,
    input           rst_i,
    input           intr_i,
    input  [31:0]   reset_vector_i,

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

`include "uriscv_defs.v"


//-----------------------------------------------------------------
// 1. Pipeline Register Definitions
//-----------------------------------------------------------------

// --- IF/ID Register ---
reg [31:0] if_id_pc_q;
reg [31:0] if_id_inst_q;
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
reg        ex_mem_reg_wr_q;
reg        ex_mem_valid_q;

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

// TODO: Implement Hazard Unit here
assign stall_w = 1'b0; 
assign flush_w = 1'b0;


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
        if_id_inst_q  <= 32'b0; // NOP
        if_id_valid_q <= 1'b0;
        if_id_pc_q    <= 32'b0;
    end else if (!stall_w) begin
        if_id_inst_q  <= mem_i_inst_i;
        if_id_valid_q <= mem_i_valid_i; // Wait for memory valid
        if_id_pc_q    <= pc_q;
    end
end


//-----------------------------------------------------------------
// STAGE 2: INSTRUCTION DECODE (ID)
//-----------------------------------------------------------------
// Register File
reg [31:0] reg_file [0:31];
wire [4:0] rs1_addr_w = if_id_inst_q[19:15];
wire [4:0] rs2_addr_w = if_id_inst_q[24:20];
wire [4:0] rd_addr_w  = if_id_inst_q[11:7];

// Immediate Generation (Simplified logic from original code)
wire [31:0] imm_w;
assign imm_w = (if_id_inst_q[6:2] == 5'b01000) ? // Store
               {{20{if_id_inst_q[31]}}, if_id_inst_q[31:25], if_id_inst_q[11:7]} :
               {{20{if_id_inst_q[31]}}, if_id_inst_q[31:20]}; // Others

// Register Read (Async)
wire [31:0] reg_rs1_data_w = (rs1_addr_w == 0) ? 0 : reg_file[rs1_addr_w];
wire [31:0] reg_rs2_data_w = (rs2_addr_w == 0) ? 0 : reg_file[rs2_addr_w];

// TODO : Control Unit Decoding (Partial) 
wire is_store_w = (if_id_inst_q[6:2] == 5'b01000);
wire is_load_w  = (if_id_inst_q[6:2] == 5'b00000);
wire is_branch_w= (if_id_inst_q[6:2] == 5'b11000);

// ID/EX Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i || flush_w) begin // TODO : Potentially make other stages not flush
        id_ex_valid_q   <= 1'b0;
        id_ex_reg_wr_q  <= 1'b0;
        id_ex_mem_wr_q  <= 1'b0; id_ex_mem_rd_q  <= 1'b0;
    end else if (!stall_w) begin
        id_ex_pc_q      <= if_id_pc_q;
        id_ex_rs1_val_q <= reg_rs1_data_w; 
        id_ex_rs2_val_q <= reg_rs2_data_w;
        id_ex_imm_q     <= imm_w;
        id_ex_rd_q      <= rd_addr_w;
        id_ex_opcode_q  <= if_id_inst_q;
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

// TODO: Implement Forwarding Unit Here
// For now, straight from ID/EX register
assign alu_in_a_w = id_ex_rs1_val_q; 
assign alu_in_b_w = (id_ex_opcode_q[6:2] == 5'b01100) ? id_ex_rs2_val_q : id_ex_imm_q; // R-Type vs I-Type
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

alu_decode decoder(
    .instruction_w(id_ex_opcode_q),
    .rs1_val_w(alu_in_a_w), 
    .rs2_val_w(alu_in_b_w), 
    .alu_func_r(alu_op_w), 
    .pc_q(id_ex_pc_q),
    .alu_input_a_r(alu_in_a_w), 
    .alu_input_b_r(alu_in_b_w), 
    .write_rd_r(write_rd_r)
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
    .rs1_val_i(alu_in_a_w),     // Note: Must use forwarded value!
    .rs2_val_i(forwarded_rs2_w), // Note: Must use forwarded value!, can't use immediate so use forwarded reg value
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
        .valid_i      (id_ex_valid_q && id_ex_is_muldiv_q),
        .inst_mul_i   (inst_mul_w),
        .inst_mulh_i  (inst_mulh_w),
        .inst_mulhsu_i(inst_mulhsu_w),
        .inst_mulhu_i (inst_mulhu_w),
        .inst_div_i   (inst_div_w),
        .inst_divu_i  (inst_divu_w),
        .inst_rem_i   (inst_rem_w),
        .inst_remu_i  (inst_remu_w),
        .operand_ra_i (rs1_ex_w),
        .operand_rb_i (rs2_ex_w),
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

wire muldiv_busy_stall_w = id_ex_valid_q && id_ex_is_muldiv_q &&
                           !muldiv_ready_w;

// Choose final EX result
wire [31:0] ex_result_w =
    id_ex_is_muldiv_q ? muldiv_result_w :
    (type_auipc_w ? (id_ex_pc_q + id_ex_imm_q) :
     type_lui_w   ? id_ex_imm_q :
     alu_core_result_w);



// EX/MEM Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i) begin
        ex_mem_valid_q  <= 1'b0;
        ex_mem_reg_wr_q <= 1'b0;
        ex_mem_opcode_q <= 32'b0;
    end else if (!stall_w) begin
        ex_mem_alu_res_q   <= alu_result_w;
        ex_mem_store_val_q <= forwarded_rs2_w;
        ex_mem_rd_q        <= id_ex_rd_q;
        ex_mem_opcode_q    <= id_ex_opcode_q; // Pass opcode for LSU
        ex_mem_reg_wr_q    <= id_ex_reg_wr_q;
        ex_mem_valid_q     <= id_ex_valid_q;
    end
    //flush
end


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
    ,.valid_i(opcode_valid_w)
    ,.opcode_i(opcode_w)
    ,.pc_i(pc_q)
    ,.rs1_val_i(rs1_val_w) // TODO : forwarded value
    ,.rs2_val_i(rs2_val_w) // TODO : forwarded value

    // CSR read result
    ,.csr_rdata_o(csr_data_w) // INPUT TO ALU DECODER

    // Exception sources
    ,.excpn_invalid_inst_i(invalid_inst_r) // TODO : constrain to stage
    ,.excpn_lsu_align_i(mem_misaligned_w) //TODO : constrain to stage

    // Used on memory alignment errors
    ,.mem_addr_i(mem_addr_w) // TODO : //

    // CSR registers
    ,.csr_mepc_o(csr_mepc_w)

    // Exception entry
    ,.exception_o(exception_w)
    ,.exception_type_o(exception_type_w)
    ,.exception_pc_o(exception_target_w)
);


//-----------------------------------------------------------------
// STAGE 4: MEMORY (MEM)
//-----------------------------------------------------------------
wire [31:0] lsu_addr_w;
wire [31:0] lsu_data_w;
wire        mem_rd_req_w;
wire [3:0]  mem_wr_req_w;

uriscv_lsu u_lsu (
    .opcode_i(ex_mem_opcode_q),
    .rs1_val_i(ex_mem_alu_res_q), // Address is pre-calculated in ALU for simple LSU
    .rs2_val_i(ex_mem_store_val_q),
    .mem_rd_o(mem_rd_req_w),
    .mem_wr_o(mem_wr_req_w),
    .mem_addr_o(lsu_addr_w), // Physical memory address
    .mem_data_o(lsu_data_w), // Write data
    .mem_misaligned_o()
);

// Connect to Top-Level Ports
assign mem_d_addr_o    = lsu_addr_w;
assign mem_d_data_wr_o = lsu_data_w;
assign mem_d_rd_o      = mem_rd_req_w;
assign mem_d_wr_o      = mem_wr_req_w;

// MEM/WB Pipeline Register Logic
always @(posedge clk_i) begin
    if (rst_i) begin
        mem_wb_valid_q  <= 1'b0;
        mem_wb_reg_wr_q <= 1'b0;
    end else begin
        // Select between Memory Data (Load) or ALU Result (Arithmetic)
        if (mem_rd_req_w) 
             mem_wb_result_q <= mem_d_data_rd_i; // Assumes memory is 0-cycle latency (Magic RAM)
        else 
             mem_wb_result_q <= ex_mem_alu_res_q;
             
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
end

endmodule