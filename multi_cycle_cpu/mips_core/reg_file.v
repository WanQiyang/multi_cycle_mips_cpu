
module reg_file(
	input clk,
	input rst,
	input [4:0] waddr,
	input [4:0] raddr1,
	input [4:0] raddr2,
	input wen,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);
    reg [31:0] mem [0:31];
    always@(posedge clk)
    if(wen)
        mem[waddr]<=wdata;
    
    assign rdata1={32{|raddr1}}&mem[raddr1];
    assign rdata2={32{|raddr2}}&mem[raddr2];

endmodule
