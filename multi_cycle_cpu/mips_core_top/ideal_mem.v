/* =========================================
* Ideal Memory Module for MIPS CPU Core
* Synchronize write (clock enable)
* Asynchronize read (do not use clock signal)
*
* Author: Yisong Chang (changyisong@ict.ac.cn)
* Date: 31/05/2016
* Version: v0.0.1
*===========================================
*/

`timescale 1 ps / 1 ps

module ideal_mem #(
	parameter ADDR_WIDTH = 10,	// 1KB
	parameter MEM_WIDTH = 2 ** (ADDR_WIDTH - 2)
	) (
	input			clk,			//source clock of the MIPS CPU Evaluation Module

	input [ADDR_WIDTH - 1:0]	Waddr,			//Memory write port address
	input [ADDR_WIDTH - 1:0]	Raddr1,			//Read port 1 address
	input [ADDR_WIDTH - 1:0]	Raddr2,			//Read port 2 address

	input			Wren,			//write enable
	input			Rden1,			//port 1 read enable
	input			Rden2,			//port 2 read enable

	input [31:0]	Wdata,			//Memory write data
	output [31:0]	Rdata1,			//Memory read data 1
	output [31:0]	Rdata2			//Memory read data 2
);

reg [31:0]	mem [MEM_WIDTH - 1:0];

`define ADDIU(rt, rs, imm) {6'b001001, rs, rt, imm}
`define LW(rt, base, off) {6'b100011, base, rt, off}
`define SW(rt, base, off) {6'b101011, base, rt, off}
`define BNE(rs, rt, off) {6'b000101, rs, rt, off}
`define NOP 32'b0

`define ADDU(rd, rs, rt) {6'b000000, rs, rt, rd, 5'b00000, 6'b100001}
`define MOVE(rd, rs) `ADDU(rd, rs, 5'd0)
`define BEQ(rs, rt, off) {6'b000100, rs, rt, off}
`define BEQZ(rs, off) `BEQ(rs, 5'd0, off)
`define B(off) `BEQ(5'd0, 5'd0, off)
`define J(tar) {6'b000010, tar}
`define JAL(tar) {6'b000011, tar}
`define JR(rs) {6'b000000, rs, 10'b0000000000, 5'b00000, 6'b001000}
`define LUI(rt, imm) {6'b001111, 5'b00000, rt, imm}
`define SLL(rd, rt, sa) {6'b000000, 5'b00000, rt, rd, sa, 6'b000000}
`define SLT(rd, rs, rt) {6'b000000, rs, rt, rd, 5'b00000, 6'b101010}
`define SLTI(rt, rs, imm) {6'b001010, rs, rt, imm}
`define SLTIU(rt, rs, imm) {6'b001011, rs, rt, imm}

`ifdef MIPS_CPU_SIM
	//Add memory initialization here
	initial begin

mem[0] = 32'h00000000;	// addr = 0x0
mem[1] = 32'h08000004;	// addr = 0x4
mem[2] = 32'h00000000;	// addr = 0x8
mem[3] = 32'hffffffff;	// addr = 0xc
mem[4] = 32'h241d0400;	// addr = 0x10
mem[5] = 32'h0c00000b;	// addr = 0x14
mem[6] = 32'h00000000;	// addr = 0x18
mem[7] = 32'h3c010000;	// addr = 0x1c
mem[8] = 32'hac20000c;	// addr = 0x20
mem[9] = 32'h08000009;	// addr = 0x24
mem[10] = 32'h00000000;	// addr = 0x28
mem[11] = 32'h27bdfff8;	// addr = 0x2c
mem[12] = 32'h24020001;	// addr = 0x30
mem[13] = 32'h24040065;	// addr = 0x34
mem[14] = 32'hafa00000;	// addr = 0x38
mem[15] = 32'h8fa30000;	// addr = 0x3c
mem[16] = 32'h00621821;	// addr = 0x40
mem[17] = 32'h24420001;	// addr = 0x44
mem[18] = 32'hafa30000;	// addr = 0x48
mem[19] = 32'h1444fffb;	// addr = 0x4c
mem[20] = 32'h00000000;	// addr = 0x50
mem[21] = 32'h8fa30000;	// addr = 0x54
mem[22] = 32'h240213ba;	// addr = 0x58
mem[23] = 32'h10620006;	// addr = 0x5c
mem[24] = 32'h00000000;	// addr = 0x60
mem[25] = 32'h24030001;	// addr = 0x64
mem[26] = 32'h3c020000;	// addr = 0x68
mem[27] = 32'hac43000c;	// addr = 0x6c
mem[28] = 32'h08000009;	// addr = 0x70
mem[29] = 32'h00000000;	// addr = 0x74
mem[30] = 32'h00001025;	// addr = 0x78
mem[31] = 32'h27bd0008;	// addr = 0x7c
mem[32] = 32'h03e00008;	// addr = 0x80
mem[33] = 32'h00000000;	// addr = 0x84
mem[34] = 32'h01200000;	// addr = 0x88
mem[35] = 32'h01000101;	// addr = 0x8c
mem[36] = 32'h00000000;	// addr = 0x90
mem[37] = 32'h00000000;	// addr = 0x94
mem[38] = 32'h00000001;	// addr = 0x98
mem[39] = 32'h00000000;	// addr = 0x9c

	end
`endif

always @ (posedge clk)
begin
	if (Wren)
		mem[Waddr] <= Wdata;
end

assign Rdata1 = {32{Rden1}} & mem[Raddr1];
assign Rdata2 = {32{Rden2}} & mem[Raddr2];

always@(posedge clk) begin
    if(mem[3] == 32'b0) begin
        $display("pass");
        $finish;
    end
    else if(mem[3] == 32'b1) begin
        $display("fail");
        $finish;
    end
end

endmodule
