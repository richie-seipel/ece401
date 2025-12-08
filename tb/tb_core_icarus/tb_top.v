//======================================================================
// tb_top.v  -- Testbench for 5-stage uRISC-V core (Harvard interface)
//======================================================================
`timescale 1ns/1ps

module tb_top;

// ------------------------------------------------------------
// Clock / Reset
// ------------------------------------------------------------
reg clk = 0;
reg rst = 1;

always #5 clk = ~clk;   // 100 MHz


// ------------------------------------------------------------
// Instruction Memory Bus
// ------------------------------------------------------------
wire        mem_i_rd_w;
wire [31:0] mem_i_pc_w;
wire        mem_i_accept_w;
wire        mem_i_valid_w;
wire [31:0] mem_i_inst_w;


// ------------------------------------------------------------
// Data Memory Bus
// ------------------------------------------------------------
wire [31:0] mem_d_addr_w;
wire [31:0] mem_d_data_wr_w;
wire        mem_d_rd_w;
wire [3:0]  mem_d_wr_w;
wire [31:0] mem_d_data_rd_w;
wire        mem_d_accept_w;
wire        mem_d_ack_w;


// ------------------------------------------------------------
// Memory Model (Dual Port RAM)
// Port 0 = Instruction
// Port 1 = Data
// ------------------------------------------------------------
tcm_mem_ram u_mem (
    // Port 0 — Instruction
    .clk0_i(clk),
    .rst0_i(rst),
    .addr0_i(mem_i_pc_w[15:2]),   // Word index
    .data0_i(32'b0),
    .wr0_i(4'b0000),
    .data0_o(mem_i_inst_w),

    // Port 1 — Data
    .clk1_i(clk),
    .rst1_i(rst),
    .addr1_i(mem_d_addr_w[15:2]), // Word index
    .data1_i(mem_d_data_wr_w),
    .wr1_i(mem_d_wr_w),
    .data1_o(mem_d_data_rd_w)
);


// Memory acknowledge interface
assign mem_i_accept_w = 1'b1;
assign mem_i_valid_w  = mem_i_rd_w;   // Instruction available immediately
assign mem_d_accept_w = 1'b1;
assign mem_d_ack_w    = mem_d_rd_w | (mem_d_wr_w != 4'b0);


// ------------------------------------------------------------
// Initialize instruction memory
// ------------------------------------------------------------
initial begin
    $display("[TB] Loading tcm.hex...");
    $readmemh("tcm.hex", u_mem.ram);
end


// ------------------------------------------------------------
// Instantiate DUT
// ------------------------------------------------------------
riscv_core u_dut (
    .clk_i(clk),
    .rst_i(rst),

    .intr_i(1'b0),

    .dbg_stall_i(1'b0),
    .dbg_stall_o(),

    .dbg_reg_addr_i(5'b0),
    .dbg_reg_wr_i(1'b0),
    .dbg_reg_wdata_i(32'b0),
    .dbg_reg_rdata_o(),

    .dbg_pc_wr_i(1'b0),
    .dbg_pc_wdata_i(32'b0),
    .dbg_pc_rdata_o(),

    .dbg_step_i(1'b0),
    .dbg_ebreakm_i(1'b0),
    .dbg_trap_o(),

    // Instruction interface
    .mem_i_rd_o(mem_i_rd_w),
    .mem_i_pc_o(mem_i_pc_w),
    .mem_i_accept_i(mem_i_accept_w),
    .mem_i_valid_i(mem_i_valid_w),
    .mem_i_inst_i(mem_i_inst_w),

    .mem_i_flush_o(),
    .mem_i_invalidate_o(),
    .mem_i_error_i(1'b0),

    // Data interface
    .mem_d_addr_o(mem_d_addr_w),
    .mem_d_data_wr_o(mem_d_data_wr_w),
    .mem_d_rd_o(mem_d_rd_w),
    .mem_d_wr_o(mem_d_wr_w),
    .mem_d_data_rd_i(mem_d_data_rd_w),
    .mem_d_accept_i(mem_d_accept_w),
    .mem_d_ack_i(mem_d_ack_w),

    .mem_d_cacheable_o(),
    .mem_d_req_tag_o(),
    .mem_d_invalidate_o(),
    .mem_d_writeback_o(),
    .mem_d_flush_o(),
    .mem_d_error_i(1'b0),

    .mtime_i(64'b0),
    .mtimecmp_i(64'b0),
    .timer_irq_o(),
    .ext_irq_o(),

    .retired_o()
);


// ------------------------------------------------------------
// Waveform dump
// ------------------------------------------------------------
initial begin
    $dumpfile("wave.vcd");
    $dumpvars(0, tb_top);
end


// ------------------------------------------------------------
// Stop simulation on ECALL
// ------------------------------------------------------------
integer cycle_count = 0;

initial begin
    repeat (5) @(posedge clk);
    rst = 0;

    $display("[TB] Simulation started...");

    while (cycle_count < 200000) begin
        @(posedge clk);
        cycle_count++;

        // ECALL instruction = 0x00000073
        if (mem_i_inst_w == 32'h00000073 && mem_i_rd_w) begin
            $display("[TB] ECALL fetched at PC = %08x", mem_i_pc_w);
            $finish;
        end
    end

    $display("[TB] Timeout.");
    $finish;
end

endmodule

