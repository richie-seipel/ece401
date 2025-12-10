//-----------------------------------------------------------------
//                          uRISC-V CPU (pipelined)
//                            Simple 5-stage
//-----------------------------------------------------------------

module riscv_core
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
#(
     parameter SUPPORT_MUL      = 1
    ,parameter SUPPORT_DIV      = 1
    ,parameter SUPPORT_CSR      = 1
    ,parameter SUPPORT_TRAP_LSU_ALIGN = 0
    ,parameter SUPPORT_MTVEC    = 0
    ,parameter SUPPORT_MTVAL    = 0
    ,parameter SUPPORT_MIP_MIE  = 0
    ,parameter SUPPORT_MSCRATCH = 0
    ,parameter SUPPORT_MCYCLE   = 1
    ,parameter SUPPORT_MTIMECMP = 0
    ,parameter SUPPORT_TRAP_INVALID_OPC = 0
    ,parameter SUPPORT_BRAM_REGFILE = 0
    ,parameter ISR_VECTOR       = 32'h00000010
)
//-----------------------------------------------------------------
// Ports
//-----------------------------------------------------------------
(
    input           clk_i
   ,input           rst_i

    // Interrupt
   ,input           intr_i

    // Debug - Halt
   ,input           dbg_stall_i
   ,output          dbg_stall_o

    // Debug - Register access
   ,input  [4:0]    dbg_reg_addr_i
   ,input           dbg_reg_wr_i
   ,input  [31:0]   dbg_reg_wdata_i
   ,output [31:0]   dbg_reg_rdata_o

    // Debug - PC access
   ,input           dbg_pc_wr_i
   ,input  [31:0]   dbg_pc_wdata_i
   ,output [31:0]   dbg_pc_rdata_o

    // Debug - Single step
   ,input           dbg_step_i
   ,input           dbg_ebreakm_i

    // Debug - Trap
   ,output          dbg_trap_o

    // Instruction fetch
   ,output          mem_i_rd_o
   ,output [31:0]   mem_i_pc_o
   ,input           mem_i_accept_i
   ,input           mem_i_valid_i
   ,input  [31:0]   mem_i_inst_i

   ,output          mem_i_flush_o
   ,output          mem_i_invalidate_o
   ,input           mem_i_error_i

    // Data access
   ,output [31:0]   mem_d_addr_o
   ,output [31:0]   mem_d_data_wr_o
   ,output          mem_d_rd_o
   ,output [3:0]    mem_d_wr_o
   ,input  [31:0]   mem_d_data_rd_i
   ,input           mem_d_accept_i
   ,input           mem_d_ack_i

   ,output          mem_d_cacheable_o
   ,output [10:0]   mem_d_req_tag_o
   ,output          mem_d_invalidate_o
   ,output          mem_d_writeback_o
   ,output          mem_d_flush_o
   ,input           mem_d_error_i

    // CSR / timer (unused in this simplified version)
   ,input  [63:0]   mtime_i
   ,input  [63:0]   mtimecmp_i
   ,output          timer_irq_o
   ,output          ext_irq_o
   ,output [31:0]   retired_o
);

//-----------------------------------------------------------------
// Includes
//-----------------------------------------------------------------
`include "uriscv_defs.v"

//-----------------------------------------------------------------
// Local parameters
//-----------------------------------------------------------------
localparam PC_W      = 32;
localparam ADDR_W    = 32;

//-----------------------------------------------------------------
// Register file
//-----------------------------------------------------------------
reg [31:0] reg_file [0:31];

//-----------------------------------------------------------------
// Debug wires (minimal, mostly stubbed)
//-----------------------------------------------------------------
assign dbg_trap_o      = 1'b0;
assign dbg_stall_o     = dbg_stall_i;
assign dbg_pc_rdata_o  = 32'b0;
assign dbg_reg_rdata_o = reg_file[dbg_reg_addr_i];

// Simple retired instruction counter (not exact, but enough)
reg [31:0] retired_q;
assign retired_o = retired_q;

always @(posedge clk_i)
begin
    if (rst_i)
        retired_q <= 32'b0;
    else if (mem_wb_valid_q && !stall_wb)
        retired_q <= retired_q + 32'd1;
end

//-----------------------------------------------------------------
// Pipeline registers and control
//-----------------------------------------------------------------

// IF/ID
reg        if_id_valid_q;
reg [31:0] if_id_pc_q;
reg [31:0] if_id_instr_q;

// ID/EX
reg        id_ex_valid_q;
reg [31:0] id_ex_pc_q;
reg [31:0] id_ex_instr_q;
reg [31:0] id_ex_rs1_val_q;
reg [31:0] id_ex_rs2_val_q;
reg [4:0]  id_ex_rs1_addr_q;
reg [4:0]  id_ex_rs2_addr_q;
reg [4:0]  id_ex_rd_addr_q;
reg [31:0] id_ex_imm_q;
reg [3:0]  id_ex_alu_op_q;
reg        id_ex_mem_read_q;
reg        id_ex_mem_write_q;
reg        id_ex_reg_write_q;
reg        id_ex_branch_q;
reg        id_ex_jump_q;
reg        id_ex_is_load_q;
reg        id_ex_is_store_q;
reg        id_ex_is_muldiv_q;

// EX/MEM
reg        ex_mem_valid_q;
reg [31:0] ex_mem_pc_q;
reg [31:0] ex_mem_alu_res_q;
reg [31:0] ex_mem_store_data_q;
reg [4:0]  ex_mem_rd_addr_q;
reg        ex_mem_mem_read_q;
reg        ex_mem_mem_write_q;
reg        ex_mem_reg_write_q;

// MEM/WB
reg        mem_wb_valid_q;
reg [31:0] mem_wb_pc_q;
reg [31:0] mem_wb_result_q;
reg [4:0]  mem_wb_rd_addr_q;
reg        mem_wb_reg_write_q;

// Global PC register
reg [31:0] pc_q;

// Stall/flush
wire stall_if;
wire stall_id;
wire stall_ex;
wire stall_mem;
wire stall_wb;

wire flush_if;
wire flush_id;
wire flush_ex;

// For simplicity, we never stall MEM/WB explicitly here:
assign stall_mem = 1'b0;
assign stall_wb  = 1'b0;

//-----------------------------------------------------------------
// IF stage: PC & fetch
//-----------------------------------------------------------------
wire       ex_branch_taken_w;
wire [31:0] ex_branch_target_w;

reg [31:0] pc_next_r;

always @*
begin
    pc_next_r = pc_q + 32'd4;
    if (ex_branch_taken_w)
        pc_next_r = ex_branch_target_w;
end

always @(posedge clk_i)
begin
    if (rst_i)
        pc_q <= 32'b0; // go to 800000
    else if (!stall_if)
        pc_q <= pc_next_r;
end

assign mem_i_pc_o       = pc_q;
assign mem_i_rd_o       = !stall_if;
assign mem_i_flush_o    = 1'b0;
assign mem_i_invalidate_o = 1'b0;

// IF/ID register
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        if_id_valid_q <= 1'b0;
        if_id_pc_q    <= 32'b0;
        if_id_instr_q <= 32'b0;
    end
    else if (!stall_id)
    begin
        if (flush_if)
        begin
            if_id_valid_q <= 1'b0;
            if_id_pc_q    <= 32'b0;
            if_id_instr_q <= 32'b0;
        end
        else if (mem_i_valid_i)
        begin
            if_id_valid_q <= 1'b1;
            if_id_pc_q    <= pc_q;
            if_id_instr_q <= mem_i_inst_i;
        end
    end
end

//-----------------------------------------------------------------
// ID stage: decode, regfile read, ID/EX
//-----------------------------------------------------------------
wire [31:0] id_instr_w = if_id_instr_q;

wire [6:0]  id_opcode_w = id_instr_w[6:0];
wire [6:0]  id_op_6_2_w = id_instr_w[6:2];
wire [2:0]  id_funct3_w = id_instr_w[14:12];
wire [6:0]  id_funct7_w = id_instr_w[31:25];

wire [4:0]  id_rs1_addr_w = id_instr_w[19:15];
wire [4:0]  id_rs2_addr_w = id_instr_w[24:20];
wire [4:0]  id_rd_addr_w  = id_instr_w[11:7];

// Immediate decode
wire [31:0] imm_i_w = {{20{id_instr_w[31]}}, id_instr_w[31:20]};
wire [31:0] imm_s_w = {{20{id_instr_w[31]}},
                       id_instr_w[31:25],
                       id_instr_w[11:7]};
wire [31:0] imm_b_w = {{19{id_instr_w[31]}},
                       id_instr_w[31],
                       id_instr_w[7],
                       id_instr_w[30:25],
                       id_instr_w[11:8],
                       1'b0};
wire [31:0] imm_u_w = {id_instr_w[31:12], 12'b0};
wire [31:0] imm_j_w = {{11{id_instr_w[31]}},
                       id_instr_w[31],
                       id_instr_w[19:12],
                       id_instr_w[20],
                       id_instr_w[30:21],
                       1'b0};

// Type decode (basic RV32I)
wire type_load_w   = (id_op_6_2_w == 5'b00000);
wire type_store_w  = (id_op_6_2_w == 5'b01000);
wire type_opimm_w  = (id_op_6_2_w == 5'b00100);
wire type_op_w     = (id_op_6_2_w == 5'b01100);
wire type_lui_w    = (id_op_6_2_w == 5'b01101);
wire type_auipc_w  = (id_op_6_2_w == 5'b00101);
wire type_jal_w    = (id_op_6_2_w == 5'b11011);
wire type_jalr_w   = (id_op_6_2_w == 5'b11001);
wire type_branch_w = (id_op_6_2_w == 5'b11000);

// M extension: mul/div under OP opcode, funct7==1
wire m_opcode_w    = type_op_w && (id_funct7_w == 7'b0000001);
wire inst_mul_w    = m_opcode_w && (id_funct3_w == 3'b000);
wire inst_mulh_w   = m_opcode_w && (id_funct3_w == 3'b001);
wire inst_mulhsu_w = m_opcode_w && (id_funct3_w == 3'b010);
wire inst_mulhu_w  = m_opcode_w && (id_funct3_w == 3'b011);
wire inst_div_w    = m_opcode_w && (id_funct3_w == 3'b100);
wire inst_divu_w   = m_opcode_w && (id_funct3_w == 3'b101);
wire inst_rem_w    = m_opcode_w && (id_funct3_w == 3'b110);
wire inst_remu_w   = m_opcode_w && (id_funct3_w == 3'b111);

wire muldiv_inst_w = inst_mul_w | inst_mulh_w | inst_mulhsu_w | inst_mulhu_w |
                     inst_div_w | inst_divu_w | inst_rem_w    | inst_remu_w;

// ALU operation select
reg [3:0] alu_op_id_r;
always @*
begin
    alu_op_id_r = `RV_ALU_NONE;

    if (type_lui_w)
        alu_op_id_r = `RV_ALU_ADD; // reg = imm_u
    else if (type_auipc_w)
        alu_op_id_r = `RV_ALU_ADD; // reg = pc + imm_u
    else if (type_opimm_w || type_op_w)
    begin
        case (id_funct3_w)
            3'b000: alu_op_id_r = (type_op_w && id_funct7_w[5]) ? `RV_ALU_SUB :
                                                                     `RV_ALU_ADD;
            3'b001: alu_op_id_r = `RV_ALU_SHIFTL;
            3'b010: alu_op_id_r = `RV_ALU_LESS_THAN_SIGNED;
            3'b011: alu_op_id_r = `RV_ALU_LESS_THAN;
            3'b100: alu_op_id_r = `RV_ALU_XOR;
            3'b101: alu_op_id_r = id_funct7_w[5] ? `RV_ALU_SHIFTR_ARITH :
                                                    `RV_ALU_SHIFTR;
            3'b110: alu_op_id_r = `RV_ALU_OR;
            3'b111: alu_op_id_r = `RV_ALU_AND;
            default: alu_op_id_r = `RV_ALU_NONE;
        endcase
    end
    else if (type_branch_w)
    begin
        alu_op_id_r = `RV_ALU_SUB; // often used for compares
    end
end

// Which immediate to use for EX?
reg [31:0] imm_id_r;
always @*
begin
    imm_id_r = 32'b0;
    if (type_opimm_w || type_load_w || type_jalr_w)
        imm_id_r = imm_i_w;
    else if (type_store_w)
        imm_id_r = imm_s_w;
    else if (type_lui_w || type_auipc_w)
        imm_id_r = imm_u_w;
    else if (type_jal_w)
        imm_id_r = imm_j_w;
    else if (type_branch_w)
        imm_id_r = imm_b_w;
end

// Does this instruction write rd?
wire reg_write_id_w = type_op_w | type_opimm_w | type_lui_w | type_auipc_w |
                      type_load_w | type_jal_w | type_jalr_w;

// RS usage for hazard detection
wire id_uses_rs1_w = type_op_w | type_opimm_w | type_load_w |
                     type_store_w | type_branch_w | type_jalr_w;
wire id_uses_rs2_w = type_op_w | type_store_w | type_branch_w;

// Register file read
wire [31:0] rs1_val_id_w = (id_rs1_addr_w != 5'd0) ? reg_file[id_rs1_addr_w] : 32'b0;
wire [31:0] rs2_val_id_w = (id_rs2_addr_w != 5'd0) ? reg_file[id_rs2_addr_w] : 32'b0;

// ID/EX register
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        id_ex_valid_q      <= 1'b0;
        id_ex_pc_q         <= 32'b0;
        id_ex_instr_q      <= 32'b0;
        id_ex_rs1_val_q    <= 32'b0;
        id_ex_rs2_val_q    <= 32'b0;
        id_ex_rs1_addr_q   <= 5'b0;
        id_ex_rs2_addr_q   <= 5'b0;
        id_ex_rd_addr_q    <= 5'b0;
        id_ex_imm_q        <= 32'b0;
        id_ex_alu_op_q     <= `RV_ALU_NONE;
        id_ex_mem_read_q   <= 1'b0;
        id_ex_mem_write_q  <= 1'b0;
        id_ex_reg_write_q  <= 1'b0;
        id_ex_branch_q     <= 1'b0;
        id_ex_jump_q       <= 1'b0;
        id_ex_is_load_q    <= 1'b0;
        id_ex_is_store_q   <= 1'b0;
        id_ex_is_muldiv_q  <= 1'b0;
    end
    else if (!stall_ex)
    begin
        if (flush_id)
        begin
            id_ex_valid_q      <= 1'b0;
            id_ex_mem_read_q   <= 1'b0;
            id_ex_mem_write_q  <= 1'b0;
            id_ex_reg_write_q  <= 1'b0;
            id_ex_branch_q     <= 1'b0;
            id_ex_jump_q       <= 1'b0;
            id_ex_is_load_q    <= 1'b0;
            id_ex_is_store_q   <= 1'b0;
            id_ex_is_muldiv_q  <= 1'b0;
        end
        else
        begin
            id_ex_valid_q      <= if_id_valid_q;
            id_ex_pc_q         <= if_id_pc_q;
            id_ex_instr_q      <= if_id_instr_q;
            id_ex_rs1_val_q    <= rs1_val_id_w;
            id_ex_rs2_val_q    <= rs2_val_id_w;
            id_ex_rs1_addr_q   <= id_rs1_addr_w;
            id_ex_rs2_addr_q   <= id_rs2_addr_w;
            id_ex_rd_addr_q    <= id_rd_addr_w;
            id_ex_imm_q        <= imm_id_r;
            id_ex_alu_op_q     <= alu_op_id_r;
            id_ex_mem_read_q   <= type_load_w;
            id_ex_mem_write_q  <= type_store_w;
            id_ex_reg_write_q  <= reg_write_id_w;
            id_ex_branch_q     <= type_branch_w;
            id_ex_jump_q       <= type_jal_w | type_jalr_w;
            id_ex_is_load_q    <= type_load_w;
            id_ex_is_store_q   <= type_store_w;
            id_ex_is_muldiv_q  <= muldiv_inst_w;
        end
    end
end

//-----------------------------------------------------------------
// EX stage: ALU, branch, mul/div, forwarding
//-----------------------------------------------------------------

// Forwarding sources
wire ex_mem_wb_en_w   = ex_mem_valid_q && ex_mem_reg_write_q &&
                        (ex_mem_rd_addr_q != 5'd0);
wire [4:0] ex_mem_wb_addr_w = ex_mem_rd_addr_q;
wire [31:0] ex_mem_wb_data_w = ex_mem_alu_res_q; // for ALU ops / stores

wire mem_wb_wb_en_w   = mem_wb_valid_q && mem_wb_reg_write_q &&
                        (mem_wb_rd_addr_q != 5'd0);
wire [4:0] mem_wb_wb_addr_w = mem_wb_rd_addr_q;
wire [31:0] mem_wb_wb_data_w = mem_wb_result_q;

// Raw operands
wire [31:0] rs1_ex_raw_w = id_ex_rs1_val_q;
wire [31:0] rs2_ex_raw_w = id_ex_rs2_val_q;

// Forwarded operands
reg [31:0] rs1_ex_w;
reg [31:0] rs2_ex_w;

always @*
begin
    rs1_ex_w = rs1_ex_raw_w;
    rs2_ex_w = rs2_ex_raw_w;

    // rs1
    if (ex_mem_wb_en_w && (ex_mem_wb_addr_w == id_ex_rs1_addr_q))
        rs1_ex_w = ex_mem_wb_data_w;
    else if (mem_wb_wb_en_w && (mem_wb_wb_addr_w == id_ex_rs1_addr_q))
        rs1_ex_w = mem_wb_wb_data_w;

    // rs2
    if (ex_mem_wb_en_w && (ex_mem_wb_addr_w == id_ex_rs2_addr_q))
        rs2_ex_w = ex_mem_wb_data_w;
    else if (mem_wb_wb_en_w && (mem_wb_wb_addr_w == id_ex_rs2_addr_q))
        rs2_ex_w = mem_wb_wb_data_w;
end

// ALU operand B selection
wire use_imm_b_w = id_ex_is_load_q | id_ex_is_store_q | type_opimm_w |
                   id_ex_jump_q | id_ex_branch_q | id_ex_is_muldiv_q; // rough

wire [31:0] alu_a_ex_w = (id_ex_is_muldiv_q) ? rs1_ex_w : rs1_ex_w;
wire [31:0] alu_b_ex_w = (id_ex_is_muldiv_q) ? rs2_ex_w :
                          (id_ex_is_load_q | id_ex_is_store_q |
                           id_ex_jump_q | type_opimm_w) ? id_ex_imm_q :
                                                          rs2_ex_w;

// ALU core
wire [31:0] alu_core_result_w;
uriscv_alu u_alu
(
    .op_i (id_ex_alu_op_q),
    .a_i  (alu_a_ex_w),
    .b_i  (alu_b_ex_w),
    .p_o  (alu_core_result_w)
);

// Branch unit (EX stage)
wire branch_taken_w;
wire [31:0] branch_target_w;
uriscv_branch u_branch
(
    .pc_i          (id_ex_pc_q),
    .opcode_i      (id_ex_instr_q),
    .rs1_val_i     (rs1_ex_w),
    .rs2_val_i     (rs2_ex_w),
    .branch_o      (branch_taken_w),
    .branch_target_o (branch_target_w)
);

assign ex_branch_taken_w  = id_ex_valid_q && (id_ex_branch_q || id_ex_jump_q) && branch_taken_w;
assign ex_branch_target_w = branch_target_w;

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

// EX/MEM register
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        ex_mem_valid_q      <= 1'b0;
        ex_mem_pc_q         <= 32'b0;
        ex_mem_alu_res_q    <= 32'b0;
        ex_mem_store_data_q <= 32'b0;
        ex_mem_rd_addr_q    <= 5'b0;
        ex_mem_mem_read_q   <= 1'b0;
        ex_mem_mem_write_q  <= 1'b0;
        ex_mem_reg_write_q  <= 1'b0;
    end
    else
    begin
        if (flush_ex)
        begin
            ex_mem_valid_q      <= 1'b0;
            ex_mem_mem_read_q   <= 1'b0;
            ex_mem_mem_write_q  <= 1'b0;
            ex_mem_reg_write_q  <= 1'b0;
        end
        else
        begin
            ex_mem_valid_q      <= id_ex_valid_q;
            ex_mem_pc_q         <= id_ex_pc_q;
            ex_mem_alu_res_q    <= ex_result_w;
            ex_mem_store_data_q <= rs2_ex_w;
            ex_mem_rd_addr_q    <= id_ex_rd_addr_q;
            ex_mem_mem_read_q   <= id_ex_mem_read_q;
            ex_mem_mem_write_q  <= id_ex_mem_write_q;
            ex_mem_reg_write_q  <= id_ex_reg_write_q;
        end
    end
end

//-----------------------------------------------------------------
// MEM stage: simple LW/SW
//-----------------------------------------------------------------
assign mem_d_addr_o      = ex_mem_alu_res_q;
assign mem_d_data_wr_o   = ex_mem_store_data_q;
assign mem_d_rd_o        = ex_mem_valid_q && ex_mem_mem_read_q;
assign mem_d_wr_o        = (ex_mem_valid_q && ex_mem_mem_write_q) ? 4'b1111 : 4'b0000;

assign mem_d_cacheable_o = 1'b0;
assign mem_d_req_tag_o   = 11'b0;
assign mem_d_invalidate_o= 1'b0;
assign mem_d_writeback_o = 1'b0;
assign mem_d_flush_o     = 1'b0;

wire [31:0] mem_load_result_w = mem_d_data_rd_i;

wire [31:0] mem_stage_result_w =
    ex_mem_mem_read_q ? mem_load_result_w : ex_mem_alu_res_q;

// MEM/WB register
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        mem_wb_valid_q     <= 1'b0;
        mem_wb_pc_q        <= 32'b0;
        mem_wb_result_q    <= 32'b0;
        mem_wb_rd_addr_q   <= 5'b0;
        mem_wb_reg_write_q <= 1'b0;
    end
    else
    begin
        mem_wb_valid_q     <= ex_mem_valid_q;
        mem_wb_pc_q        <= ex_mem_pc_q;
        mem_wb_result_q    <= mem_stage_result_w;
        mem_wb_rd_addr_q   <= ex_mem_rd_addr_q;
        mem_wb_reg_write_q <= ex_mem_reg_write_q;
    end
end

//-----------------------------------------------------------------
// WB stage: register file writeback
//-----------------------------------------------------------------
wire rf_we_w    = mem_wb_valid_q && mem_wb_reg_write_q && (mem_wb_rd_addr_q != 5'd0);
wire [4:0] rf_waddr_w = mem_wb_rd_addr_q;
wire [31:0] rf_wdata_w = mem_wb_result_q;

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        reg_file[0] <= 32'b0;
    end
    else
    begin
        if (rf_we_w)
            reg_file[rf_waddr_w] <= rf_wdata_w;

        // x0 is always 0
        reg_file[0] <= 32'b0;

        // Optional debug write
        if (dbg_reg_wr_i)
            reg_file[dbg_reg_addr_i] <= dbg_reg_wdata_i;
    end
end

//-----------------------------------------------------------------
// Hazard detection: load-use + mul/div
//-----------------------------------------------------------------
wire load_in_ex_w =
    id_ex_valid_q && id_ex_mem_read_q && (id_ex_rd_addr_q != 5'd0);

wire load_use_hazard_w =
    load_in_ex_w &&
    (
        (id_uses_rs1_w && (id_ex_rd_addr_q == id_rs1_addr_w)) ||
        (id_uses_rs2_w && (id_ex_rd_addr_q == id_rs2_addr_w))
    );

assign stall_if = load_use_hazard_w | muldiv_busy_stall_w;
assign stall_id = load_use_hazard_w | muldiv_busy_stall_w;
assign stall_ex = muldiv_busy_stall_w;

// Branch flush: kill younger stages
assign flush_if = ex_branch_taken_w;
assign flush_id = ex_branch_taken_w;

// Insert bubble between load and dependent instruction
assign flush_ex = load_use_hazard_w;

//-----------------------------------------------------------------
// Timer / IRQ outputs (not implemented here)
//-----------------------------------------------------------------
assign timer_irq_o = 1'b0;
assign ext_irq_o   = 1'b0;

//-----------------------------------------------------------------
// End of module
//-----------------------------------------------------------------

endmodule

