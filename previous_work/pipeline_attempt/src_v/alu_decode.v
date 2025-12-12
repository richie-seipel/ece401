`include "uriscv_defs.v"

module alu_decode 
#(
    parameter SUPPORT_CSR = 1,
    parameter SUPPORT_TRAP_INVALID_OPC = 1,
    parameter SUPPORT_MUL = 1,
    parameter SUPPORT_DIV = 1
)
//-----------------------------------------------------------------
// I/O
//-----------------------------------------------------------------    
(
    input wire [31:0] instruction_w, 
    input wire [31:0] rs1_val_w,
    input wire [31:0] rs2_val_w, 
    input wire [31:0] pc_q, 
    input wire [31:0] csr_data_w,
    output reg [31:0] alu_input_a_r, 
    output reg [31:0] alu_input_b_r, 
    output reg [3:0] alu_func_r, 
    output reg write_rd_r, 
    output wire inst_mul_i,       
    output wire inst_mulh_i,  
    output wire inst_mulhsu_i,
    output wire inst_mulhu_i,
    output wire inst_div_i,   
    output wire inst_divu_i,  
    output wire inst_rem_i,   
    output wire inst_remu_i,
    output reg invalid_inst_i,
    output wire is_muldiv_i
);
    
//-----------------------------------------------------------------
// Params
//-----------------------------------------------------------------
`define PC_W        32
`define ADDR_W      32

localparam           PC_W                = `PC_W;
localparam           PC_PAD_W            = 0;
localparam           PC_EXT_W            = 0;
localparam           ADDR_W              = `ADDR_W;
localparam           ADDR_PAD_W          = 0;

reg [31:0]  imm20_r;
reg [31:0]  imm12_r;

always @ *
begin
    imm20_r     = {instruction_w[31:12], 12'b0};
    imm12_r     = {{20{instruction_w[31]}}, instruction_w[31:20]};
end


wire [2:0] func3_w  = instruction_w[14:12]; // R, I, S
wire [6:0] func7_w  = instruction_w[31:25]; // R

wire inst_csr_w     = SUPPORT_CSR && type_system_w && (func3_w != 3'b000 && func3_w != 3'b100);


wire inst_ecall_w    = SUPPORT_CSR && type_system_w && (instruction_w[31:7] == 25'h000000);
wire inst_ebreak_w   = SUPPORT_CSR && type_system_w && (instruction_w[31:7] == 25'h002000);
wire inst_mret_w     = SUPPORT_CSR && type_system_w && (instruction_w[31:7] == 25'h604000);

wire type_rvc_w     = (instruction_w[1:0] != 2'b11);


// ALU operations excluding mul/div
wire mul_inst_w      = SUPPORT_MUL && type_op_w && (func7_w == 7'b0000001) && ~func3_w[2];
wire div_inst_w      = SUPPORT_DIV && type_op_w && (func7_w == 7'b0000001) &&  func3_w[2];
wire muldiv_inst_w = mul_inst_w | div_inst_w;
assign is_muldiv_i = muldiv_inst_w;

wire type_alu_op_w  = (type_op_w && (func7_w == 7'b0000000)) ||
                      (type_op_w && (func7_w == 7'b0100000));
wire type_load_w    = (instruction_w[6:2] == 5'b00000);
wire type_opimm_w   = (instruction_w[6:2] == 5'b00100);
wire type_auipc_w   = (instruction_w[6:2] == 5'b00101);
wire type_store_w   = (instruction_w[6:2] == 5'b01000);
wire type_op_w      = (instruction_w[6:2] == 5'b01100);
wire type_lui_w     = (instruction_w[6:2] == 5'b01101);
wire type_branch_w  = (instruction_w[6:2] == 5'b11000);
wire type_jalr_w    = (instruction_w[6:2] == 5'b11001);
wire type_jal_w     = (instruction_w[6:2] == 5'b11011);
wire type_system_w  = (instruction_w[6:2] == 5'b11100);
wire type_miscm_w   = (instruction_w[6:2] == 5'b00011);

//-----------------------------------------------------------------
// ALU inputs
//-----------------------------------------------------------------
// ALU operation selection

assign inst_mul_i      = mul_inst_w && (func3_w == 3'b000); 
assign inst_mulh_i     = mul_inst_w && (func3_w == 3'b001);
assign inst_mulhsu_i   = mul_inst_w && (func3_w == 3'b010);
assign inst_mulhsu_i    = mul_inst_w && (func3_w == 3'b011);
assign inst_div_i      = div_inst_w && (func3_w == 3'b100);
assign inst_divu_i     = div_inst_w && (func3_w == 3'b101);
assign inst_rem_i      = div_inst_w && (func3_w == 3'b110);
assign inst_remu_i     = div_inst_w && (func3_w == 3'b111);
wire inst_nop_w      = (type_miscm_w && (func3_w == 3'b000)) | // fence
                       (type_miscm_w && (func3_w == 3'b001));  // fence.i

always @ *
begin
    alu_func_r     = `RV_ALU_NONE;
    alu_input_a_r  = rs1_val_w;
    alu_input_b_r  = rs2_val_w;
    write_rd_r     = 1'b0;

    case (1'b1)
    type_alu_op_w:
    begin
        alu_input_a_r  = rs1_val_w;
        alu_input_b_r  = rs2_val_w;
    end
    type_opimm_w:
    begin
        alu_input_a_r  = rs1_val_w;
        alu_input_b_r  = imm12_r;
    end
    type_lui_w:
    begin
        alu_input_a_r  = 32'b0;
        alu_input_b_r  = imm20_r;
    end
    type_auipc_w:
    begin
        alu_input_a_r[PC_W-1:0]  = pc_q;
        alu_input_b_r  = imm20_r;
    end
    type_jal_w,
    type_jalr_w:
    begin
        alu_input_a_r[PC_W-1:0]  = pc_q;
        alu_input_b_r  = 32'd4;
    end
    default : ;
    endcase

    if (muldiv_inst_w)
        write_rd_r     = 1'b1;
    else if (type_opimm_w || type_alu_op_w)
    begin
        case (func3_w)
        3'b000:  alu_func_r =  (type_op_w & instruction_w[30]) ? 
                              `RV_ALU_SUB:              // SUB
                              `RV_ALU_ADD;              // ADD  / ADDI
        3'b001:  alu_func_r = `RV_ALU_SHIFTL;           // SLL  / SLLI
        3'b010:  alu_func_r = `RV_ALU_LESS_THAN_SIGNED; // SLT  / SLTI
        3'b011:  alu_func_r = `RV_ALU_LESS_THAN;        // SLTU / SLTIU
        3'b100:  alu_func_r = `RV_ALU_XOR;              // XOR  / XORI
        3'b101:  alu_func_r = instruction_w[30] ? 
                              `RV_ALU_SHIFTR_ARITH:     // SRA  / SRAI
                              `RV_ALU_SHIFTR;           // SRL  / SRLI
        3'b110:  alu_func_r = `RV_ALU_OR;               // OR   / ORI
        3'b111:  alu_func_r = `RV_ALU_AND;              // AND  / ANDI
        endcase

        write_rd_r = 1'b1;
    end
    else if (inst_csr_w)
    begin
        alu_func_r     = `RV_ALU_ADD;
        alu_input_a_r  = 32'b0;
        alu_input_b_r  = csr_data_w;
        write_rd_r     = 1'b1;
    end
    else if (type_auipc_w || type_lui_w || type_jalr_w || type_jal_w)
    begin
        write_rd_r     = 1'b1;
        alu_func_r     = `RV_ALU_ADD;
    end
    else if (type_load_w)
        write_rd_r     = 1'b1;

    invalid_inst_i = SUPPORT_TRAP_INVALID_OPC;

    if (   type_load_w
         | type_opimm_w
         | type_auipc_w
         | type_store_w
         | type_alu_op_w
         | type_lui_w
         | type_branch_w
         | type_jalr_w
         | type_jal_w
         | inst_ecall_w 
         | inst_ebreak_w 
         | inst_mret_w 
         | inst_csr_w
         | inst_nop_w
         | muldiv_inst_w)
        invalid_inst_i = SUPPORT_TRAP_INVALID_OPC && type_rvc_w;
    

end

endmodule