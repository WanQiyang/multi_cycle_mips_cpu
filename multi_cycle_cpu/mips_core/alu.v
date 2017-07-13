
module alu(
	input [31:0] A,
	input [31:0] B,
	input [4:0] shamt,
	input [2:0] ALUop,
	output Overflow,
	output CarryOut,
	output Zero,
	output reg [31:0] Result
);
	wire OF, CF;
    wire [31:0] B_OP, Result_AND, Result_OR, Result_PLUS, Result_LUI, Result_SLL, Result_SLTU;
    
    always@(*)
    begin
       case(ALUop)
       3'b000: Result<=Result_AND;
       3'b001: Result<=Result_OR;
       3'b010, 3'b110: Result<=Result_PLUS;
       3'b111: Result<=Result_SLT;
       3'b011: Result<=Result_LUI;
       3'b100: Result<=Result_SLL;
       3'b101: Result<=Result_SLTU;
       default: Result<=32'b0;
       endcase
    end
    
    assign Result_AND=A&B;
    assign Result_OR=A|B;
    assign B_OP=B^{32{ALUop[2]}};
    assign {CF, OF, Result_PLUS}={A[31], A}+{B_OP[31], B_OP}+ALUop[2];
    assign Result_SLT={31'b0, OF};
    assign Result_LUI={B[15:0], 16'b0};
    assign Result_SLL=B<<shamt;
    assign Result_SLTU={31'b0, ~CF};
    assign Overflow=OF^Result_PLUS[31];
    assign CarryOut=CF^ALUop[2];
    assign Zero=~|Result_PLUS;

endmodule
