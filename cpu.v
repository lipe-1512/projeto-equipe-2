module cpu(input clk, input reset);

// Wires for data paths
wire [31:0] PC_out, PC_in;
wire [31:0] IR_out;
wire [31:0] MDR_out;
wire [31:0] A_out, B_out;
wire [31:0] ALUOut_out;
wire [31:0] HI_out, LO_out;
wire [31:0] EPC_out;
wire [31:0] ALU_result;
wire [31:0] Memory_out;
wire [31:0] Data1, Data2;
wire [31:0] WriteData;
wire [31:0] SE_out, ShiftedSE_out;
wire [27:0] Shift26_out;
wire [31:0] Shift_out;
wire [31:0] SignExt_out, SignExt1_out;
wire [31:0] Exception_out;
wire [31:0] ALUSrcA_out, ALUSrcB_out;
wire [31:0] Memory_address;
wire [31:0] PcSource_out;
wire [4:0] WriteReg_out;
wire [4:0] RegRs_out, ShiftAmt_out;
wire [31:0] ShiftSrc_out;
wire [31:0] HI_in, LO_in;

// Control signals
wire PC_wr, PCWriteCond, and_out, PC_enable;
wire [2:0] IorD;
wire mem_wr;
wire [3:0] mem_wr_byte;
wire ir_wr;
wire RegRs, ShiftAmt, ShiftSrc;
wire [1:0] reg_dst;
wire reg_wr;
wire wr_A, wr_B;
wire [1:0] Alu_Src_A;
wire [1:0] Alu_Src_B;
wire [2:0] Alu_Op;
wire Alu_out_wr;
wire EPC_wr;
wire [2:0] PC_Source;
wire [1:0] cause_control;
wire [2:0] mem_reg;
wire [1:0] load_control, store_control;
wire MemDataWrite;
wire [2:0] shift_control;
wire hi_wr, lo_wr;
wire reset_out;
wire mult_start, div_start;
wire Mult_div_lo, Mult_div_hi;
wire [1:0] shift_control_in;
wire [1:0] shift_n;
wire [3:0] DataSrc;
wire [1:0] Exception;

// ALU flags
wire zero_flag, overflow_flag, neg_flag, et_flag, gt_flag, lt_flag;

// Instruction fields
wire [5:0] OpCode = IR_out[31:26];
wire [5:0] Funct = IR_out[5:0];
wire OpCode404_flag = 1'b0;
wire div_zero;

// Multiplier/Divider
wire mult_ready, div_ready;

// Store/Load logic
wire [31:0] store_data;
wire [31:0] load_data;

// Exception cause register
wire [31:0] cause_out;

// PC
Registrador PC_(
    .Clk(clk),
    .Reset(reset),
    .Load(PC_enable),
    .Entrada(PcSource_out),
    .Saida(PC_out)
);

// Instruction Register
Instr_Reg InstrReg(
    .Clk(clk),
    .Reset(reset),
    .Load_ir(ir_wr),
    .Entrada(Memory_out),
    .Instr31_26(IR_out[31:26]),
    .Instr25_21(IR_out[25:21]),
    .Instr20_16(IR_out[20:16]),
    .Instr15_0(IR_out[15:0])
);

// Memory Data Register
Registrador MemoryDataRegister(
    .Clk(clk),
    .Reset(reset),
    .Load(MemDataWrite),
    .Entrada(load_data),
    .Saida(MDR_out)
);

// Registers A and B
Registrador A(
    .Clk(clk),
    .Reset(reset),
    .Load(wr_A),
    .Entrada(Data1),
    .Saida(A_out)
);

Registrador B(
    .Clk(clk),
    .Reset(reset),
    .Load(wr_B),
    .Entrada(Data2),
    .Saida(B_out)
);

// ALUOut
Registrador ALUout(
    .Clk(clk),
    .Reset(reset),
    .Load(Alu_out_wr),
    .Entrada(ALU_result),
    .Saida(ALUOut_out)
);

// HI and LO
Registrador HI(
    .Clk(clk),
    .Reset(reset),
    .Load(hi_wr),
    .Entrada(HI_in),
    .Saida(HI_out)
);

Registrador LO(
    .Clk(clk),
    .Reset(reset),
    .Load(lo_wr),
    .Entrada(LO_in),
    .Saida(LO_out)
);

// EPC
Registrador EPC(
    .Clk(clk),
    .Reset(reset),
    .Load(EPC_wr),
    .Entrada(PC_out),
    .Saida(EPC_out)
);

// Cause register (for exceptions)
Registrador Cause(
    .Clk(clk),
    .Reset(reset),
    .Load(1'b1), // Always load when exception occurs
    .Entrada({30'b0, cause_control}),
    .Saida(cause_out)
);

// MUXes

// RegRs mux
mux2x1 #(.WIDTH(5)) muxRegRs(
    .sel(RegRs),
    .in0(IR_out[25:21]),
    .in1(5'b11101),
    .out(RegRs_out)
);

// ShiftAmt mux
mux2x1 #(.WIDTH(5)) muxShiftAmt(
    .sel(ShiftAmt),
    .in0(B_out[10:6]),
    .in1(IR_out[10:6]),
    .out(ShiftAmt_out)
);

// ShiftSrc mux
mux2x1 #(.WIDTH(32)) muxShiftSrc(
    .sel(ShiftSrc),
    .in0(A_out),
    .in1(B_out),
    .out(ShiftSrc_out)
);

// IorD mux
mux3x1 #(.WIDTH(32)) muxIorD(
    .sel(IorD[1:0]),
    .in0(PC_out),
    .in1(ALUOut_out),
    .in2(ALU_result),
    .out(Memory_address)
);

// Memory address mux to avoid 'X' during reset
wire [31:0] memory_address_mux = reset ? 32'h00000000 : Memory_address;

// Exception mux
mux3x1 #(.WIDTH(32)) muxException(
    .sel(Exception),
    .in0(32'h000000FD),
    .in1(32'h000000FE),
    .in2(32'h000000FF),
    .out(Exception_out)
);

// ALUSrcA mux
mux3x1 #(.WIDTH(32)) muxALUSrcA(
    .sel(Alu_Src_A),
    .in0(PC_out),
    .in1(A_out),
    .in2(Data1),
    .out(ALUSrcA_out)
);

// ALUSrcB mux
mux4x1 #(.WIDTH(32)) muxALUSrcB(
    .sel(Alu_Src_B),
    .in0(B_out),
    .in1(32'd4),
    .in2(SignExt_out),
    .in3(ShiftedSE_out),
    .out(ALUSrcB_out)
);

// RegDst mux
mux4x1 #(.WIDTH(5)) muxRegDst(
    .sel(reg_dst),
    .in0(IR_out[20:16]),
    .in1(IR_out[15:11]),
    .in2(5'b11101),
    .in3(5'b11111),
    .out(WriteReg_out)
);

// PC Source mux
mux6x1 #(.WIDTH(32)) muxPcSource(
    .sel(PC_Source),
    .in0(ALU_result),
    .in1(ALUOut_out),
    .in2({PC_out[31:28], Shift26_out}),
    .in3(EPC_out),
    .in4(Exception_out),
    .in5(32'b0),
    .out(PcSource_out)
);

// Data Source mux
mux9x1 #(.WIDTH(32)) muxDataSrc(
    .sel(DataSrc),
    .in0(ALU_result),
    .in1(MDR_out),
    .in2(HI_out),
    .in3(LO_out),
    .in4(SignExt1_out),
    .in5(ALUOut_out),
    .in6(EPC_out),
    .in7(Shift_out),
    .in8(32'hE3),
    .out(WriteData)
);

// Sign extenders
signExtend16x32 SignExt(
    .in(IR_out[15:0]),
    .out(SignExt_out)
);

signExtend1x32 SignExt1(
    .in(lt_flag),
    .out(SignExt1_out)
);

// Shift modules
shift26x28 Shift26x28(
    .in(IR_out[25:0]),
    .out(Shift26_out)
);

Shift32x32 ShiftPC(
    .in(SignExt_out),
    .out(ShiftedSE_out)
);

// Shifter
RegDesloc Shift(
    .Clk(clk),
    .Reset(reset),
    .Entrada(ShiftSrc_out),
    .N(ShiftAmt_out),
    .Shift(shift_control),
    .Saida(Shift_out)
);

// ALU
Ula32 ALU(
    .A(ALUSrcA_out),
    .B(ALUSrcB_out),
    .Seletor(Alu_Op),
    .S(ALU_result),
    .z(zero_flag),
    .Overflow(overflow_flag),
    .Negativo(neg_flag),
    .Igual(et_flag),
    .Maior(gt_flag),
    .Menor(lt_flag)
);

// Register Bank
Banco_Reg bankReg(
    .Clk(clk),
    .Reset(reset),
    .RegWrite(reg_wr),
    .ReadReg1(RegRs_out),
    .ReadReg2(IR_out[20:16]),
    .WriteReg(WriteReg_out),
    .WriteData(WriteData),
    .ReadData1(Data1),
    .ReadData2(Data2)
);

// Memory
Memoria Memory(
    .Clock(clk),
    .Wr(mem_wr),
    .Address(memory_address_mux),
    .Datain(store_data),
    .Dataout(Memory_out)
);

// Multiplier
multiplier mult(
    .clk(clk),
    .start(mult_start),
    .a(A_out),
    .b(B_out),
    .hi(HI_in),
    .lo(LO_in),
    .ready(mult_ready)
);

// Divider
divider div(
    .clk(clk),
    .start(div_start),
    .a(A_out),
    .b(B_out),
    .hi(HI_in),
    .lo(LO_in),
    .ready(div_ready),
    .div_zero(div_zero)
);

// Control Unit
controlUnit CPU(
    .clk(clk),
    .reset(reset),
    .O(overflow_flag),
    .OpCode404_flag(OpCode404_flag),
    .div_zero(div_zero),
    .OpCode(OpCode),
    .Funct(Funct),
    .zero(zero_flag),
    .neg(neg_flag),
    .lt(lt_flag),
    .gt(gt_flag),
    .et(et_flag),
    .mult_ready(mult_ready),
    .div_ready(div_ready),
    .IorD(IorD),
    .mem_wr(mem_wr),
    .cause_control(cause_control),
    .ir_wr(ir_wr),
    .reg_wr(reg_wr),
    .wr_A(wr_A),
    .wr_B(wr_B),
    .mem_reg(mem_reg),
    .reg_dst(reg_dst),
    .Alu_Src_A(Alu_Src_A),
    .Alu_Src_B(Alu_Src_B),
    .Alu_Op(Alu_Op),
    .PCWriteCond(PCWriteCond),
    .Alu_out_wr(Alu_out_wr),
    .PC_Source(PC_Source),
    .PC_wr(PC_wr),
    .EPC_wr(EPC_wr),
    .load_control(load_control),
    .store_control(store_control),
    .mult_start(mult_start),
    .div_start(div_start),
    .Mult_div_lo(Mult_div_lo),
    .Mult_div_hi(Mult_div_hi),
    .Lo_wr(lo_wr),
    .hi_wr(hi_wr),
    .reset_out(reset_out),
    .shift_control(shift_control),
    .DataSrc(DataSrc),
    .RegRs(RegRs)
);

// PC enable logic
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

// Store size logic
assign store_data = (store_control == 2'b00) ? B_out : // sw
                   (store_control == 2'b01) ? {24'b0, B_out[7:0]} : // sb
                   B_out;

assign mem_wr_byte = (store_control == 2'b00) ? {4{mem_wr}} : // sw
                    (store_control == 2'b01) ? (4'b0001 << Memory_address[1:0]) : // sb
                    4'b0000;

// Load size logic
assign load_data = (load_control == 2'b00) ? Memory_out : // lw
                  (load_control == 2'b01) ? (
                      (ALUOut_out[1:0] == 2'b00) ? {{24{Memory_out[7]}}, Memory_out[7:0]} :
                      (ALUOut_out[1:0] == 2'b01) ? {{24{Memory_out[15]}}, Memory_out[15:8]} :
                      (ALUOut_out[1:0] == 2'b10) ? {{24{Memory_out[23]}}, Memory_out[23:16]} :
                      {{24{Memory_out[31]}}, Memory_out[31:24]} // 2'b11
                  ) :
                  Memory_out;

endmodule
