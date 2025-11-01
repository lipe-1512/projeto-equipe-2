module Datapath (
    input clk,
    input reset,
    output [31:0] PC_out,
    output [31:0] ALU_result_out,
    output zero_flag,
    output overflow_flag,
    output Negativo_flag,
    output Igual_flag,
    output Maior_flag,
    output Menor_flag
);

wire and_out;
wire PCWrite;
wire PCWriteCond;
wire [1:0] IorD;
wire MemWR;
wire IRWrite;
wire RegRs;
wire [1:0] RegDst;
wire RegWrite;
wire WrA;
wire WrB;
wire [1:0] ALUSrcA;
wire [2:0] ALUSrcB;
wire [2:0] ALUControl;
wire ALUOutCtrl;
wire EPCCtrl;
wire [2:0] PcSource;
wire [1:0] Exception;
wire [2:0] DataSrc;
wire LSControl;
wire SSControl;
wire MemDataWrite;
wire SEControl;
wire ShiftSrc;
wire ShiftAmt;
wire [2:0] ShiftControl;
wire MDControl;
wire WriteHI;
wire WriteLO;
wire div0;
wire PC_enable;
wire [31:0] IR_out;
wire [31:0] PcSource_out;
wire [31:0] Data1;
wire [31:0] Data2;
wire [31:0] A_out;
wire [31:0] B_out;
wire [31:0] ALU_result;
wire [31:0] ALUOut_out;
wire [31:0] EPC_out;
wire [31:0] HI_out;
wire [31:0] LO_out;
wire [31:0] Memory_out;
wire [31:0] MemoryData_out;
wire [31:0] SE_out;
wire [31:0] Shift26_out;
wire [31:0] ShiftedSE_out;
wire [31:0] SignExt_out;
wire [31:0] SignExt1_out;
wire [31:0] RegRs_out;
wire [4:0] ShiftAmt_out;
wire [31:0] ShiftSrc_out;
wire [31:0] Memory_address;
wire [31:0] Exception_out;
wire [31:0] ALUSrcA_out;
wire [31:0] ALUSrcB_out;
wire [4:0] WriteReg_out;
wire [31:0] WriteData_out;
wire [31:0] Shift_out;
wire op404_flag;
wire [5:0] OpCode;
wire [5:0] Funct;
wire zero, neg, lt, gt, et;
wire reset_out;
wire mult_ready, div_ready, div_zero_flag;
wire [31:0] mult_hi, mult_lo, div_hi, div_lo;
wire mult_start, div_start;

    portaAND AND_(
        .in0(PcSource_out),
        .in1(PCWriteCond),
        .out(and_out)
    );

    portaOR OR_(
        .in0(PCWrite),
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
        .out(RegRs_out)
    );

    mux2x1 muxShiftAmt(
        .sel(ShiftAmt),
        .in0(B_out[10:6]),
        .in1(IR_out[10:6]),
        .out(ShiftAmt_out)
    );

    mux2x1 muxShiftSrc(
        .sel(ShiftSrc),
        .in0(A_out),
        .in1(B_out),
        .out(ShiftSrc_out)
    );

    mux3x1 muxIorD(
        .sel(IorD),
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
        .sel(ALUSrcA),
        .in0(PC_out),
        .in1(A_out),
        .in2(Data1),
        .out(ALUSrcA_out)
        );

    mux4x1 muxALUSrcB(
        .sel(ALUSrcB), 
        .in0(B_out),
        .in1(32'd4),
        .in2(SignExt_out),
        .in3(ShiftedSE_out),
        .out(ALUSrcB_out)
    );

    mux4x1 muxRegDst(
        .sel(RegDst),
        .in0(IR_out[20:16]),
        .in1(IR_out[15:11]),
        .in2(5'b11101),
        .in3(5'b11111),
        .out(WriteReg_out)
    );

    mux6x1 muxPcSource(
        .sel(PcSource),
        .in0(ALU_result),
        .in1(ALUOut_out),
        .in2(Shift26_out),
        .in3(EPC_out),
        .in4(Exception_out),
        .out(PcSource_out)
    );

    mux9x1 muxDataSrc(
        .sel(DataSrc),
        .in0(ALU_result),
        .in1(MemoryData_out),
        .in2(HI_out),
        .in3(LO_out),
        .in4(SignExt1_out),
        .in5(32'b0),
        .in6(32'b0),
        .in7(32'b0),
        .in8(32'b0),
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

    shift26x28 Shift26x32(
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

    Banco_Regs bankReg(
        .Clk(clk),
        .Reset(reset),
        .RegWrite(RegWrite),
        .ReadReg1(RegRs_out),
        .ReadReg2(IR_out[20:16]),
        .WriteReg(WriteReg_out),
        .WriteData(WriteData_out),
        .ReadData1(Data1),
        .ReadData2(Data2)
    );

    Memoria Memory(
        .clk(clk),
        .MemWR(MemWR),
        .address(ALUOut_out),
        .writeData(B_out),
        .readData(Memory_out)
    );

    RegDesloc Shift(
        .in(ShiftSrc_out),
        .shiftAmt(ShiftAmt_out),
        .shiftControl(ShiftControl),
        .out(Shift_out)
    );

    Ula32 ALU(
        .A(ALUSrcA_out),
        .B(ALUSrcB_out),
        .Seletor(ALUControl),
        .S(ALU_result),
        .z(zero_flag),
        .Overflow(overflow_flag),
        .Negativo(Negativo_flag),
        .Igual(Igual_flag),
        .Maior(Maior_flag),
        .Menor(Menor_flag)
    );

    multiplier mult(
        .clk(clk),
        .start(mult_start),
        .a(A_out),
        .b(B_out),
        .hi(mult_hi),
        .lo(mult_lo),
        .ready(mult_ready)
    );

    divider div(
        .clk(clk),
        .start(div_start),
        .a(A_out),
        .b(B_out),
        .hi(div_hi),
        .lo(div_lo),
        .ready(div_ready),
        .div_zero(div_zero_flag)
    );

    control_unit CPU(
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PCWriteCond(PCWriteCond),
        .IorD(IorD),
        .MemWR(MemWR),
        .IRWrite(IRWrite),
        .RegRs(RegRs),
        .RegDst(RegDst),
        .RegWrite(RegWrite),
        .WrA(WrA),
        .WrB(WrB),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ALUControl(ALUControl),
        .ALUOutCtrl(ALUOutCtrl),
        .EPCCtrl(EPCCtrl),
        .PcSource(PcSource),
        .Exception(Exception),
        .DataSrc(DataSrc),
        .LSControl(LSControl),
        .SSControl(SSControl),
        .MemDataWrite(MemDataWrite),
        .SEControl(SEControl),
        .ShiftSrc(ShiftSrc),
        .ShiftAmt(ShiftAmt),
        .ShiftControl(ShiftControl),
        .MDControl(MDControl),
        .WriteHI(WriteHI),
        .WriteLO(WriteLO),
        .div0(div0),
        .reset_out(reset_out),
        .overflow_flag(overflow_flag),
        .igual_flag(igual_flag),
        .maior_flag(maior_flag),
        .menor_flag(menor_flag),
        .zero_flag(zero_flag)
    );

endmodule