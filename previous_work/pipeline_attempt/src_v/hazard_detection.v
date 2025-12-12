// Handle pipeline stalls, eventually forwarding 
module hazard_detection
//-----------------------------------------------------------------
// I/O
//-----------------------------------------------------------------    
( 
    input wire [31:0] if_id_instruction,
    input wire [31:0] id_ex_instruction, 
    input wire [31:0] ex_mem_instruction,
    input wire [31:0] mem_wb_instruction, 
    output reg flush_if_id,
    output reg flush_id_ex,
    output reg flush_ex_mem,
    output reg flush_mem_wb,
    output reg stall_if_id,
    output reg stall_id_ex,
    output reg stall_ex_mem,
    output reg stall_mem_wb
);

//-----------------------------------------------------------------
// Parameters
//-----------------------------------------------------------------    
parameter LOAD = 5'b00000;
parameter IMM  = 5'b00100;
parameter AUIPC = 5'b00101;
parameter STORE = 5'b01000;
parameter RTYPE = 5'b01100;
parameter BRANCH = 5'b11000;
parameter LUI    = 5'b01101;

//-----------------------------------------------------------------
// Internal Logic
//-----------------------------------------------------------------    
wire type_load_w    = (if_id_instruction[6:2] == LOAD);
wire type_opimm_w   = (if_id_instruction[6:2] == IMM);
wire type_auipc_w   = (if_id_instruction[6:2] == AUIPC);
wire type_store_w   = (if_id_instruction[6:2] == STORE);
wire type_op_w      = (if_id_instruction[6:2] == RTYPE);
wire type_lui_w     = (if_id_instruction[6:2] == LUI);
wire type_branch_w  = (if_id_instruction[6:2] == 5'b11000);
wire type_jalr_w    = (if_id_instruction[6:2] == 5'b11001);
wire type_jal_w     = (if_id_instruction[6:2] == 5'b11011);
wire type_system_w  = (if_id_instruction[6:2] == 5'b11100);
wire type_miscm_w   = (if_id_instruction[6:2] == 5'b00011);

wire [4:0] if_id_rs1 = if_id_instruction[19:15]; 
wire [4:0] if_id_rs2 = if_id_instruction[24:20];
wire [4:0] if_id_rd = if_id_instruction[11:7];
wire [4:0] id_ex_rs1 = id_ex_instruction[19:15]; 
wire [4:0] id_ex_rs2 = id_ex_instruction[24:20];
wire [4:0] id_ex_rd = id_ex_instruction[11:7];
wire [4:0] ex_mem_rd = ex_mem_instruction[11:7]; 

// Check for RAW hazards 

//check if id/ex destination is the

// i if/id
// j id/ex 
// instr in id/ex reads before instr in if/id writes 
// so check if if/id destination reg is either id/ex input


// add A, B, C (ID)
// stall (EX)
// stall (MEM) 
// add D, A, C (IF -> ID reads from WB)

//Load use hazards

wire if_id_has_rs1; //has rs1 if not lui, auipc, or jal
wire if_id_has_rs2; //if rtype, branch, or store -> reg value, otherwise -> immediate value
wire id_ex_has_rd;
wire ex_mem_has_rd; 

assign if_id_has_rs1 = ~(type_lui_w || type_auipc_w || type_jal_w);
assign if_id_has_rs2 = ~(type_op_w || type_store_w || type_branch_w);

assign id_ex_has_rd = ~(id_ex_instruction[6:2] == STORE || id_ex_instruction[6:2] == BRANCH);
assign ex_mem_has_rd = ~(ex_mem_instruction[6:2] == STORE || ex_mem_instruction[6:2] == BRANCH);

wire [4:0] rs1_addr_w = if_id_instruction[19:15];
wire [4:0] rs2_addr_w = if_id_instruction[24:20];
wire [4:0] rd_addr_w  = id_ex_instruction[11:7];

`define OP 6:2

wire [4:0] if_id_op;
assign if_id_op = if_id_instruction[`OP];  
always @(*) begin
    flush_if_id = 1'b0;
    flush_id_ex = 1'b0;
    flush_ex_mem = 1'b0;
    flush_mem_wb = 1'b0;
    stall_if_id = 1'b0;
    stall_id_ex = 1'b0;
    stall_ex_mem = 1'b0;
    stall_mem_wb = 1'b0;
    /*
    
    //potential hazard
    if ((if_id_has_rs1 && id_ex_has_rd) || (if_id_has_rs2 && id_ex_has_rd)) begin
        //check for hazard RAW (j reads before i writes)
        if ((if_id_rs1 == id_ex_rd) || (if_id_rs2 == id_ex_rd)) begin
            //stall if so id_ex can continue and calculate before instr arrives
            stall_if_id = 1'b1;
        end
        //other hazard?
    end

    //RS1 is a potential RAW
    if((if_id_op == IMM) || (if_id_op == LOAD) || (if_id_op == STORE) || (if_id_op == BRANCH)) begin
        
        //stall if_id and id_ex
        stall_if_id = 1'b1;
        stall_id_ex = 1'b1;
    end
    // two r type instructions
    else if (if_id_instruction[`OP] == RTYPE && id_ex_instruction[`OP] == RTYPE) begin
        //two r types so stall
        //check if dest of id_ex is src of if_id
        if ((if_id_rs1 == id_ex_rd) || (if_id_rs2 == id_ex_rd)) begin
            //stall if_id not id_ex so it has time to execute
            stall_if_id = 1'b1;
        end
        else if ((if_id_rs1 == ex_mem_rd) || (if_id_rs2 == ex_mem_rd)) begin
        
        end
    end
    else begin 
        //otherwise set all low
        
    end
*/
end

endmodule 

/* Gemini replacement
module hazard_detection
( 
    input wire [31:0] if_id_instruction,
    input wire [31:0] id_ex_instruction, 
    input wire [31:0] ex_mem_instruction,
    input wire [31:0] mem_wb_instruction, 
//new inputs go here
input muldiv_busy
    output reg flush_if_id,
    output reg flush_id_ex,
    output reg flush_ex_mem,
    output reg flush_mem_wb,
    output reg stall_if_id,
    output reg stall_id_ex,
    output reg stall_ex_mem,
    output reg stall_mem_wb
);

    //-----------------------------------------------------------------
    // Parameters (Standard RISC-V Opcodes)
    //-----------------------------------------------------------------    
    parameter LOAD   = 5'b00000;
    parameter IMM    = 5'b00100;
    parameter STORE  = 5'b01000;
    parameter RTYPE  = 5'b01100;
    parameter BRANCH = 5'b11000;
    parameter JALR   = 5'b11001;

    //-----------------------------------------------------------------
    // Decode Current Instruction (IF/ID Stage)
    //-----------------------------------------------------------------    
    wire [4:0] opcode_if_id = if_id_instruction[6:2];
    wire [4:0] rs1_if_id    = if_id_instruction[19:15];
    wire [4:0] rs2_if_id    = if_id_instruction[24:20];

    // Determine if current instruction *reads* RS1 or RS2
    wire uses_rs1 = (opcode_if_id == RTYPE) || (opcode_if_id == IMM) || 
                    (opcode_if_id == LOAD)  || (opcode_if_id == STORE) || 
                    (opcode_if_id == BRANCH)|| (opcode_if_id == JALR);

    wire uses_rs2 = (opcode_if_id == RTYPE) || (opcode_if_id == STORE) || 
                    (opcode_if_id == BRANCH);

    //-----------------------------------------------------------------
    // Decode Older Instructions (ID/EX and EX/MEM Stages)
    //-----------------------------------------------------------------
    wire [4:0] rd_id_ex  = id_ex_instruction[11:7];
    wire [4:0] opcode_id_ex = id_ex_instruction[6:2];
    
    wire [4:0] rd_ex_mem = ex_mem_instruction[11:7];
    wire [4:0] opcode_ex_mem = ex_mem_instruction[6:2];

    // Check if older instructions *write* to a register
    // (Everything except STORE and BRANCH writes to RD)
    wire id_ex_writes_rd  = (rd_id_ex != 0) && 
                            (opcode_id_ex != STORE) && (opcode_id_ex != BRANCH);
                            
    wire ex_mem_writes_rd = (rd_ex_mem != 0) && 
                            (opcode_ex_mem != STORE) && (opcode_ex_mem != BRANCH);

// ---------------------------------------------
        // 1. Handle Multi-Cycle Operations (Mul/Div)
        // ---------------------------------------------
        if (muldiv_busy) begin //doesnt exist yet
            // Freeze the whole pipeline until the result is ready
            stall_if_id  = 1'b1;
            stall_id_ex  = 1'b1;
            stall_ex_mem = 1'b1; 
            stall_mem_wb = 1'b1;
        end

        // ---------------------------------------------
        // 2. Handle Control Hazards (Branch Taken)
        // ---------------------------------------------
        // If branch is taken in EX, the instructions in ID and IF are wrong.
        else if (branch_taken_ex) begin //doesnt exist yet
            flush_if_id = 1'b1; // Kill instruction in Decode
            flush_id_ex = 1'b1; // Kill instruction in Execute (turn to bubble)
        end


    //-----------------------------------------------------------------
    // Hazard Detection Logic (RAW)
    //-----------------------------------------------------------------
    always @(*) begin
        // Default: No stall, no flush
        stall_if_id  = 1'b0;
        flush_id_ex  = 1'b0;
        
        // Unused signals
        stall_id_ex  = 1'b0;
        stall_ex_mem = 1'b0;
        stall_mem_wb = 1'b0;
        flush_if_id  = 1'b0;
        flush_ex_mem = 1'b0;
        flush_mem_wb = 1'b0;

        // Check for Data Hazards (Read-After-Write)
        // If we need a register that is currently being calculated in EX or MEM, STALL.
        if ( (uses_rs1 && id_ex_writes_rd  && (rs1_if_id == rd_id_ex)) ||
             (uses_rs1 && ex_mem_writes_rd && (rs1_if_id == rd_ex_mem)) ||
             (uses_rs2 && id_ex_writes_rd  && (rs2_if_id == rd_id_ex)) ||
             (uses_rs2 && ex_mem_writes_rd && (rs2_if_id == rd_ex_mem)) ) 
        begin
            stall_if_id = 1'b1; // Freeze PC and IF/ID
            flush_id_ex = 1'b1; // Insert Bubble into ID/EX
        end
    end

endmodule
*/