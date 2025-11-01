module PC (
    input clk,
    input reset
);

wire [31:0] IR_out;
wire [31:0] PcSource_out;
wire [31:0] Data1;
wire [31:0] Data2;
wire [31:0] A_out;
wire [31:0] B_out;
wire [31:0] ALU_result;
wire [31:0] ALUOut_out;
wire [31:0] EPC_out;
wire [31:0] PC_out;
wire [31:0] HI_in;
wire [31:0] HI_out;
wire [31:0] LO_in;
wire [31:0] LO_out;
wire [31:0] Memory_out;
wire [31:0] MemoryData_out;
wire [31:0] RegRs_out;
wire [31:0] ShiftAmt_out;
wire [31:0] ShiftSrc_out;
wire [31:0] Memory_address;
wire [31:0] Exception_out;
wire [31:0] ALUSrcA_out;
wire [31:0] ALUSrcB_out;
wire [4:0] WriteReg_out;
wire [31:0] WriteData_out;
wire [31:0] Shift_out;
wire [31:0] SE_out;
wire [27:0] Shift26_out;
wire [31:0] ShiftedSE_out;
wire [31:0] SignExt_out;
wire [31:0] SignExt1_out;
wire zero_flag;
wire overflow_flag;
wire Igual_flag;
wire Maior_flag;
wire Menor_flag;
wire Negativo_flag;
wire [5:0] OpCode = IR_out[31:26];
wire [5:0] Funct = IR_out[5:0];
wire OpCode404_flag = 1'b0;
wire div_zero = 1'b0;
wire PC_wr;
wire PCWriteCond;
wire and_out;
wire PC_enable;
wire [2:0] IorD;
wire mem_wr;
wire ir_wr;
wire RegRs;
wire [1:0] reg_dst;
wire reg_wr;
wire wr_A;
wire wr_B;
wire [1:0] Alu_Src_A;
wire [1:0] Alu_Src_B;
wire [2:0] Alu_Op;
wire Alu_out_wr;
wire EPC_wr;
wire [2:0] PC_Source;
wire [1:0] cause_control;
wire [2:0] mem_reg;
wire [1:0] load_control;
wire [1:0] store_control;
wire MemDataWrite;
wire ShiftSrc;
wire ShiftAmt;
wire [2:0] shift_control;
wire hi_wr;
wire Lo_wr;
wire div0;
wire reset_out;
wire mult_start;
wire div_start;
wire Mult_div_lo;
wire Mult_div_hi;
wire [1:0] shift_control_in;
wire [1:0] shift_n;
wire [3:0] DataSrc;
wire [1:0] Exception = 2'b00;


parameter valor = {24'b000000000000000000000000,8'b11100011};

    portaAND AND_(
        .in0(zero_flag),
        .in1(PCWriteCond),
        .out(and_out)
    );

    portaOR OR_(
        .in0(PC_wr),
        .in1(and_out),
        .out(PC_enable)
    );

    Registrador PC_(
        .Clk(clk),
        .Reset(reset),
        .Load(PC_enable),
        .Entrada(PcSource_out),
        .Saida(PC_out)
    );

    Registrador A(
        .Clk(clk),
        .Reset(reset),
        .Load(WrA),
        .Entrada(Data1),
        .Saida(A_out)
    );

    Registrador B(
        .Clk(clk),
        .Reset(reset),
        .Load(WrB),
        .Entrada(Data2),
        .Saida(B_out)
    );

    Registrador ALUout(
        .Clk(clk),
        .Reset(reset),
        .Load(ALUOutCtrl),
        .Entrada(ALU_result),
        .Saida(ALUOut_out)
    );

    Registrador EPC(
        .Clk(clk),
        .Reset(reset),
        .Load(EPCCtrl),
        .Entrada(PC_out),
        .Saida(EPC_out)
    );

    Registrador HI(
        .Clk(clk),
        .Reset(reset),
        .Load(WriteHI),
        .Entrada(HI_in),
        .Saida(HI_out)
    );

    Registrador LO(
        .Clk(clk),
        .Reset(reset),
        .Load(WriteLO),
        .Entrada(LO_in),
        .Saida(LO_out)
    );

    Registrador MemoryDataRegister(
        .Clk(clk),
        .Reset(reset),
        .Load(MemDataWrite),
        .Entrada(Memory_out),
        .Saida(MemoryData_out)
    );

    mux2x1 muxRegRs(
        .sel(RegRs),
        .in0(IR_out[25:21]),
        .in1(5'b11101),
        .out(RegRs_out[4:0])
    );

    mux2x1 muxShiftAmt(
        .sel(ShiftAmt),
        .in0(B_out[10:6]),
        .in1(IR_out[10:6]),
        .out(ShiftAmt_out[4:0])
    );

    mux2x1_32 muxShiftSrc(
        .sel(ShiftSrc),
        .in0(A_out),
        .in1(B_out),
        .out(ShiftSrc_out)
    );

    mux3x1 muxIorD(
        .sel(IorD[0]),
        .in0(PC_out),
        .in1(ALUOut_out),
        .in2(ALU_result),
        .out(Memory_address)
    );

    mux3x1 muxException(
        .sel(Exception),
        .in0(32'b00000000000000000000000011111101),
        .in1(32'b00000000000000000000000011111110),
        .in2(32'b00000000000000000000000011111111),
        .out(Exception_out)
    );

    mux3x1 muzALUSrcA(
        .sel(Alu_Src_A[0]),
        .in0(PC_out),
        .in1(A_out),
        .in2(Data1),
        .out(ALUSrcA_out)
        );

    mux4x1 muxALUSrcB(
        .sel(Alu_Src_B), 
        .in0(B_out),
        .in1(32'd4),
        .in2(SignExt_out),
        .in3(ShiftedSE_out),
        .out(ALUSrcB_out)
    );

    mux4x1 muxRegDst(
        .sel(reg_dst),
        .in0(IR_out[20:16]),
        .in1(IR_out[15:11]),
        .in2(5'b11101),
        .in3(5'b11111),
        .out(WriteReg_out)
    );

    mux6x1 muxPcSource(
        .sel(PC_Source),
        .in0(ALU_result),
        .in1(ALUOut_out),
        .in2(Shift26_out),
        .in3(EPC_out),
        .in4(Exception_out),
        .in5(32'b0),
        .out(PcSource_out)
    );

    mux9x1 muxDataSrc(
        .sel(DataSrc),
        .in0(ALU_result),
        .in1(MemoryData_out),
        .in2(HI_out),
        .in3(LO_out),
        .in4(SignExt1_out),
        .in5(ALUOut_out),
        .in6(EPC_out),
        .in7(Shift_out),
        .in8(valor),
        .out(WriteData_out)
    );

    Shift32x32 ShiftLeft2(
        .in(ALU_result),
        .out(PcSource_out)
    );

    sEwithzero16x32 SE(
        .in(IR_out[15:0]),
        .out(SE_out)
    );

    shift26x28 Shift26x28(
        .in(IR_out[25:0]),
        .out(Shift26_out)
    );

    Shift32x32 ShiftPC(
        .in(SE_out),
        .out(ShiftedSE_out)
    );

    signExtend16x32 SignExt(
        .in(IR_out[15:0]),
        .out(SignExt_out)
    );

    signExtend1x32 SignExt1(
        .in(Menor_flag),
        .out(SignExt1_out)
    );

    Instr_Reg InstrReg(
        .Clk(clk),
        .Reset(reset),
        .Load_ir(IRWrite),
        .Entrada(Memory_out),
        .Instr31_26(IR_out[31:26]),
        .Instr25_21(IR_out[25:21]),
        .Instr20_16(IR_out[20:16]),
        .Instr15_0(IR_out[15:0])
    );

    Banco_Reg bankReg(
        .Clk(clk),
        .Reset(reset),
        .RegWrite(RegWrite),
        .ReadReg1(RegRs_out[4:0]),
        .ReadReg2(IR_out[20:16]),
        .WriteReg(WriteReg_out),
        .WriteData(WriteData_out),
        .ReadData1(Data1),
        .ReadData2(Data2)
    );

    Memoria Memory(
        .Clock(clk),
        .Wr({4{MemWR}}),
        .Address(Memory_address),
        .Datain(B_out),
        .Dataout(Memory_out)
    );

    RegDesloc Shift(
        .Clk(clk),
        .Reset(reset),
        .Entrada(ShiftSrc_out),
        .N(ShiftAmt_out[4:0]),
        .Shift(shift_control),
        .Saida(Shift_out)
    );

    Ula32 ALU(
        .A(ALUSrcA_out),
        .B(ALUSrcB_out),
        .Seletor(Alu_Op),
        .S(ALU_result),
        .z(zero_flag),
        .Overflow(overflow_flag),
        .Negativo(Negativo_flag),
        .Igual(Igual_flag),
        .Maior(Maior_flag),
        .Menor(Menor_flag)
    );

    controlUnit CPU(
        .clk(clk),
        .reset(reset),
        .O(overflow_flag),
        .OpCode404_flag(OpCode404_flag),
        .div_zero(div_zero),
        .OpCode(OpCode),
        .Funct(Funct),
        .zero(zero_flag),
        .neg(Negativo_flag),
        .lt(Menor_flag),
        .gt(Maior_flag),
        .et(Igual_flag),
        .PCWriteCond(PCWriteCond),
        .IorD(IorD),
        .mem_wr(mem_wr),
        .ir_wr(ir_wr),
        .reg_wr(reg_wr),
        .wr_A(wr_A),
        .wr_B(wr_B),
        .reg_dst(reg_dst),
        .Alu_Src_A(Alu_Src_A),
        .Alu_Src_B(Alu_Src_B),
        .Alu_Op(Alu_Op),
        .Alu_out_wr(Alu_out_wr),
        .PC_Source(PC_Source),
        .PC_wr(PC_wr),
        .EPC_wr(EPC_wr),
        .load_control(load_control),
        .store_control(store_control),
        .MemDataWrite(MemDataWrite),
        .ShiftSrc(ShiftSrc),
        .ShiftAmt(ShiftAmt),
        .shift_control(shift_control),
        .hi_wr(hi_wr),
        .Lo_wr(Lo_wr),
        .reset_out(reset_out),
        .overflow_flag(overflow_flag),
        .igual_flag(Igual_flag),
        .maior_flag(Maior_flag),
        .menor_flag(Menor_flag),
        .zero_flag(zero_flag)
    );

endmodule