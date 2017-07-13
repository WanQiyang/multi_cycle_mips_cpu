module mips_cpu(
	input  clk,
	input  rst,
	output reg [31:0] PC,
	input  [31:0] Instruction,
	output [31:0] Address,
	output MemWrite,
	output [31:0] Write_data,
	output MemRead,
	input  [31:0] Read_data
);
    wire [31:0] alu_in_A, alu_in_B;
    wire [4:0] alu_in_shamt;
    wire [2:0] alu_in_ALUop;
    wire alu_out_Overflow, alu_out_CarryOut, alu_out_Zero;
    wire [31:0] alu_out_Result;

    wire [4:0] rf_in_waddr, rf_in_raddr1, rf_in_raddr2;
    wire rf_in_wen;
    wire [31:0] rf_in_wdata;
    wire [31:0] rf_out_rdata1, rf_out_rdata2;
    
    wire [5:0] cu_in_Opcode;
    wire cu_out_RegWrite, cu_out_ALUSrcA, cu_out_MemRead, cu_out_MemWrite, cu_out_IRWrite, 
        cu_out_PCWrite, cu_out_PCWriteCond, cu_out_ALUCond;
    wire [1:0] cu_out_RegDst, cu_out_MemtoReg, cu_out_ALUSrcB, cu_out_PCSource;
    
    wire ac_in_ALUCond;
    wire [5:0] ac_in_Opcode, ac_in_Funct;
    wire [2:0] ac_out_ALUop;
    wire ac_out_PCWrite;
    
    wire [5:0] bc_in_Opcode;
    wire [31:0] bc_in_ALUResult;
    wire bc_in_Overflow, bc_in_CarryOut, bc_in_Zero;
    wire bc_out_Result;
    
    reg [31:0] Instruction_reg, Memory_Data_reg, ALUIn_A, ALUIn_B, ALUOut;
    wire PC_control;
    wire [31:0] PC_next, Sign_extend;
    
    alu alu_i(.A(alu_in_A), .B(alu_in_B), .shamt(alu_in_shamt), .ALUop(alu_in_ALUop), .Overflow(alu_out_Overflow), 
        .CarryOut(alu_out_CarryOut), .Zero(alu_out_Zero), .Result(alu_out_Result));
        
    reg_file rf_i(.rst(rst), .clk(clk), .waddr(rf_in_waddr), .raddr1(rf_in_raddr1), .raddr2(rf_in_raddr2), 
        .wen(rf_in_wen), .wdata(rf_in_wdata), .rdata1(rf_out_rdata1), .rdata2(rf_out_rdata2));
        
    control_unit cu_i(.clk(clk), .rst(rst), .Opcode(cu_in_Opcode), .RegDst(cu_out_RegDst), 
        .RegWrite(cu_out_RegWrite), .ALUSrcA(cu_out_ALUSrcA), .MemRead(cu_out_MemRead), 
        .MemWrite(cu_out_MemWrite), .MemtoReg(cu_out_MemtoReg), .IRWrite(cu_out_IRWrite), 
        .PCWrite(cu_out_PCWrite), .PCWriteCond(cu_out_PCWriteCond), .ALUCond(cu_out_ALUCond), 
        .ALUSrcB(cu_out_ALUSrcB), .PCSource(cu_out_PCSource));
    
    alu_control ac_i(.ALUCond(ac_in_ALUCond), .Opcode(ac_in_Opcode), .Funct(ac_in_Funct), 
        .ALUop(ac_out_ALUop), .PCWrite(ac_out_PCWrite));
    
    branch_control bc_i(.Opcode(bc_in_Opcode), .ALUResult(bc_in_ALUResult), .Overflow(bc_in_Overflow), 
        .CarryOut(bc_in_CarryOut), .Zero(bc_in_Zero), .BranchResult(bc_out_Result));

    assign alu_in_A = cu_out_ALUSrcA ? ALUIn_A : PC;
    assign alu_in_B = cu_out_ALUSrcB[1] ? 
        (cu_out_ALUSrcB[0] ? Sign_extend << 2 : Sign_extend) : 
        (cu_out_ALUSrcB[0] ? 32'd4 : ALUIn_B);
    assign alu_in_shamt = Instruction_reg[10:6];
    assign alu_in_ALUop = ac_out_ALUop;

    assign rf_in_waddr = cu_out_RegDst[1] ? 5'd31 :
        (cu_out_RegDst[0] ? Instruction_reg[15:11] : Instruction_reg[20:16]);
    assign rf_in_raddr1 = Instruction_reg[25:21];
    assign rf_in_raddr2 = Instruction_reg[20:16];
    assign rf_in_wen = cu_out_RegWrite;
    assign rf_in_wdata = cu_out_MemtoReg[1] ? PC : 
        (cu_out_MemtoReg[0] ? Memory_Data_reg : ALUOut);
    
    assign cu_in_Opcode = Instruction_reg[31:26];
    
    assign ac_in_ALUCond = cu_out_ALUCond;
    assign ac_in_Opcode = Instruction_reg[31:26];
    assign ac_in_Funct = Instruction_reg[5:0];
    
    assign bc_in_Opcode = Instruction_reg[31:26];
    assign bc_in_ALUResult = alu_out_Result;
    assign bc_in_Overflow = alu_out_Overflow;
    assign bc_in_CarryOut = alu_out_CarryOut;
    assign bc_in_Zero = alu_out_Zero;
    
    assign PC_control = ( bc_out_Result & cu_out_PCWriteCond ) | cu_out_PCWrite | ac_out_PCWrite;
    assign PC_next = cu_out_PCSource[1] ? {PC[31:28], Instruction_reg[25:0], 2'b0} :
        (cu_out_PCSource[0] ? ALUOut : alu_out_Result);
    assign Sign_extend = {{16{Instruction_reg[15]}}, Instruction_reg[15:0]};
    
    assign Address = ALUOut;
    assign MemWrite = cu_out_MemWrite;
    assign Write_data = ALUIn_B;
    assign MemRead = cu_out_MemRead;
    
    always @(posedge clk)
    begin
        if(rst)
            PC <= 32'd0;
        else if(PC_control)
            PC <= PC_next;

        if(cu_out_IRWrite)
            Instruction_reg <= Instruction;
        
        Memory_Data_reg <= Read_data;
        ALUIn_A <= rf_out_rdata1;
        ALUIn_B <= rf_out_rdata2;
        ALUOut <= alu_out_Result;
        
    end
    
endmodule

module control_unit(
    input clk,
    input rst,
    input [5:0] Opcode,
    output [1:0] RegDst, // 1 2
    output RegWrite, // 3
    output ALUSrcA, // 4
    output MemRead, // 5
    output MemWrite, // 6
    output [1:0] MemtoReg, // 7 8
    //output IorD, // 9
    output IRWrite, // 10
    output PCWrite, // 11
    output PCWriteCond, // 12
    output ALUCond, //13
    output [1:0] ALUSrcB, // 14 15
    output [1:0] PCSource // 16 17
);
    reg [3:0] State;
    reg [16:0] CtrlResult;
    always @(posedge clk)
    begin
        if(rst)
            State <= 0;
        else
            State <= State_next(State, Opcode);
    end
    
    always @(*)
        case(State)
        4'd0: CtrlResult <= 17'b00001_00001_100_0100; // Instruction Fetch
        4'd1: CtrlResult <= 17'b00000_00000_000_1100; // Instruction Decode / Register Fetch
        4'd2, 4'd10: CtrlResult <= 17'b00010_00000_000_1000; // Memory Address Computation (LW, SW)
        4'd3: CtrlResult <= 17'b00001_00010_000_0000; // Memory Access (LW)
        4'd4: CtrlResult <= 17'b00100_00100_000_0000; // Memory Read Completion Step (LW)
        4'd5: CtrlResult <= 17'b00000_10010_000_0000; // Memory Access (SW)
        4'd6: CtrlResult <= 17'b00010_00000_001_0000; // Execution (R-format)
        4'd7: CtrlResult <= 17'b01100_00000_000_0000; // R-type Completion (R-format)
        4'd8: CtrlResult <= 17'b00010_00000_011_0001; // Branch Completion (BEQ, BNE)
        4'd9: CtrlResult <= 17'b00000_00000_100_0010; // Jump Completion (J)
        4'd11: CtrlResult <= 17'b00010_00000_001_1000; // Execution (ADDIU, LUI, SLTI, SLTIU)
        4'd12: CtrlResult <= 17'b00100_00000_000_0000; // ADDIU Completion (ADDIU, LUI, SLTI, SLTIU)
        4'd13: CtrlResult <= 17'b10100_01000_100_0010; // Jump Completion (JAL)
        default: CtrlResult <= 17'b00000_00000_000_0000; // Default
        endcase

    
    assign {RegDst, RegWrite, ALUSrcA, MemRead, MemWrite, MemtoReg, IorD, IRWrite, PCWrite, PCWriteCond, 
        ALUCond, ALUSrcB, PCSource} = CtrlResult;
    
    function [3:0] State_next; 
    input [3:0] State_now;
    input [5:0] Opcode; 
    begin
        case(State_now)
        4'd1: State_next = Optype(Opcode); 
        4'd4, 4'd5, 4'd7, 4'd8, 4'd9, 4'd12, 4'd13: State_next = 4'd0;
        4'd10: State_next = 4'd5;
        default: State_next = State_now + 1;
        endcase
    end 
    endfunction
    
    function [3:0] Optype;
    input [5:0] Opcode;
    begin
        case(Opcode)
        6'b100011: Optype=4'd2; // LW
        6'b101011: Optype=4'd10; // SW
        6'b000000: Optype=4'd6; // R-format
        6'b000100, 6'b000101: Optype=4'd8; // BEQ, BNE
        6'b000010: Optype=4'd9; // J
        6'b000011: Optype=4'd13; //JAL
        6'b001001, 6'b001111, 6'b001010, 6'b001011: Optype=4'd11; // ADDIU, LUI, SLTI, SLTIU
        default: Optype=4'd0; // undefined
        endcase
    end
    endfunction
    
endmodule

module alu_control(
    input ALUCond,
    input [5:0] Opcode,
    input [5:0] Funct,
    output reg [2:0] ALUop,
    output reg PCWrite
);
    always @(*)
        if(ALUCond)
            case(Opcode)
            6'b000000: // R-format
                case(Funct)
                6'b001000: {ALUop, PCWrite} <= 4'b010_1; // JR
                6'b000000: {ALUop, PCWrite} <= 4'b100_0; // SLL
                6'b101010: {ALUop, PCWrite} <= 4'b111_0; // SLT
                default: {ALUop, PCWrite} <= 4'b010_0; // ADDU and others
                endcase
            6'b000100, 6'b000101: // BEQ, BNE
                {ALUop, PCWrite} <= 4'b110_0;
            6'b001111: // LUI
                {ALUop, PCWrite} <= 4'b011_0;
            6'b001010: // SLT
                {ALUop, PCWrite} <= 4'b111_0;
            6'b001011: // SLTU
                {ALUop, PCWrite} <= 4'b101_0;
            default: // ADDIU and others
                {ALUop, PCWrite} <= 4'b010_0;
            endcase
        else // ALU keeps plus
            {ALUop, PCWrite} <= 4'b010_0;
endmodule

module branch_control(
    input [5:0] Opcode,
    input [31:0] ALUResult,
    input Overflow,
    input CarryOut,
    input Zero,
    output reg BranchResult
);
    always @(*)
    begin
        case(Opcode)
        6'b000100: BranchResult <= Zero; // BEQ
        6'b000101: BranchResult <= ~Zero; // BNE
        default: BranchResult <= 1'b0;
        endcase
    end
    
endmodule
