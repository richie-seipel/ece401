//Instruction decoder module 

module decode(
    input wire [4:0] opcode, 
    input wire [2:0] funct3, 
    input wire [4:0] funct5, 
    input wire [6:0] funct7
    output reg [2:0] alu_func, // ALU op code
    output reg write_rd, //Write destination register
    output reg use_imm, // Use immediate for ALU operand B?
    output reg mem_wr, // Write to memory
    output reg mem_rd, //Reaad from memory
    output reg load_signed, // lh, and lb
    output reg load_byte, //lb, lbu
    output reg load_half, //lh, lhu
    output reg load_offset, // Mem offset for lsu
    output reg opcode_valid, // valid opcode
    //Multiplier signals
    output reg inst_mul, 
    output reg inst_mulh, 
    output reg inst_mulhsu, 
    output reg inst_div



);

    

endmodule
