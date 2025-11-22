module cpu (
    input clk,
    input reset // Reset externo (opcional)
);

// =================================================================
// 1. Inicialização Automática (Power-On Reset)
// =================================================================
// Gera um pulso rápido apenas para garantir que a FSM vá para o estado 0.
reg internal_por = 1'b1;
initial #10 internal_por = 1'b0;

// O reset que vai para a ControlUnit é a soma do externo com o interno
wire fsm_reset = (reset === 1'b1) | internal_por;

// =================================================================
// Sinais de Interconexão
// =================================================================
wire PC_wr, PCWriteCond, mem_wr, ir_wr, reg_wr, wr_A, wr_B, Alu_out_wr, EPC_wr, mult_start, div_start, hi_wr, lo_wr, RegRs;
wire [2:0] IorD;
wire [2:0] PC_Source;
wire [1:0] reg_dst;
wire [1:0] Alu_Src_A;
wire [2:0] Alu_Src_B;
wire [2:0] Alu_Op;
wire [3:0] DataSrc;
wire [1:0] load_control, store_control;
wire [2:0] shift_control;
wire [1:0] cause_control;
wire [2:0] mem_reg;
wire [3:0] mem_wr_byte_enable;

// Sinal CRÍTICO que vem da ControlUnit
// Indica se a CPU está no estado de Reset (1) ou Operacional (0)
wire reset_out; 

wire [31:0] PC_out, PC_in;
wire [31:0] IR_full;
wire [31:0] MDR_out;
wire [31:0] A_out, B_out;
wire [31:0] ALUOut_out;
wire [31:0] HI_out, LO_out;
wire [31:0] EPC_out;
wire [31:0] ALU_result;
wire [31:0] Memory_read_data;
wire [31:0] Regs_read_data1, Regs_read_data2;
wire [31:0] Write_data_to_regs;
wire [31:0] SignExt_out;
wire [27:0] Shift26_out;
wire [31:0] ALUSrcA_mux_out, ALUSrcB_mux_out;
wire [31:0] Memory_address;
wire [4:0]  WriteReg_mux_out;
wire [4:0]  ReadReg1_final;
wire [31:0] store_data_to_mem;
wire [31:0] loaded_data_final;
wire [31:0] Shift_out;
wire [31:0] HI_in, LO_in;
wire mult_ready, div_ready;

wire [5:0] OpCode = IR_full[31:26];
wire [4:0] rs = IR_full[25:21];
wire [4:0] rt = IR_full[20:16];
wire [4:0] rd = IR_full[15:11];
wire [5:0] Funct = IR_full[5:0];
wire [4:0] shamt = IR_full[10:6];
wire zero_flag, overflow_flag, neg_flag, et_flag, gt_flag, lt_flag;
wire div_zero_flag;
wire OpCode404_flag; 

wire [5:0] ir_31_26;
wire [4:0] ir_25_21;
wire [4:0] ir_20_16;
wire [15:0] ir_15_0;


wire [31:0] Safe_Memory_Address;
assign Safe_Memory_Address = (reset_out === 1'b0) ? Memory_address : 32'h00000000;


controlUnit u_control (
    .clk(clk), 
    .reset(fsm_reset), // Usa o reset com pulso inicial
    .O(overflow_flag), .OpCode404_flag(OpCode404_flag), .div_zero(div_zero_flag),
    .OpCode(OpCode), .Funct(Funct),
    .zero(zero_flag), .neg(neg_flag), .lt(lt_flag), .gt(gt_flag), .et(et_flag),
    .mult_ready(mult_ready), .div_ready(div_ready),

    // Saídas (Conectado apenas UMA vez agora)
    .reset_out(reset_out), // Este sinal controlará a proteção da memória e registradores
    .PCWriteCond(PCWriteCond), .IorD(IorD), .mem_wr(mem_wr), .ir_wr(ir_wr),
    .reg_wr(reg_wr), .wr_A(wr_A), .wr_B(wr_B), .reg_dst(reg_dst),
    .Alu_Src_A(Alu_Src_A), .Alu_Src_B(Alu_Src_B), .Alu_Op(Alu_Op),
    .Alu_out_wr(Alu_out_wr), .PC_Source(PC_Source), .PC_wr(PC_wr),
    .EPC_wr(EPC_wr), .cause_control(cause_control), .mem_reg(mem_reg),
    .load_control(load_control), .store_control(store_control),
    .mult_start(mult_start), .div_start(div_start), .hi_wr(hi_wr),
    .Lo_wr(lo_wr), .shift_control(shift_control),
    .DataSrc(DataSrc), .RegRs(RegRs), .mem_wr_byte_enable(mem_wr_byte_enable)
);

// =================================================================
// Datapath (Usando reset_out para limpar registradores)
// =================================================================
    
    wire PC_enable = PC_wr | (PCWriteCond & zero_flag);
    
    mux6x1 #(.WIDTH(32)) mux_pc_source ( .sel(PC_Source), .in0(ALUOut_out), .in1({PC_out[31:28], Shift26_out}), .in2(EPC_out), .in3(32'h000000FC), .in4(A_out), .in5(32'b0), .out(PC_in) );
    
    // Registradores usam 'reset_out' (Síncrono com o estado da FSM)
    Registrador PC_reg (.Clk(clk), .Reset(reset_out), .Load(PC_enable), .Entrada(PC_in), .Saida(PC_out));

    mux2x1_32 mux_mem_addr (.sel(IorD[0]), .in0(PC_out), .in1(ALUOut_out), .out(Memory_address));
    
    // Memória usa o endereço protegido
    Memoria main_memory (.Clock(clk), .Wr(mem_wr), .Address(Safe_Memory_Address), .Datain(store_data_to_mem), .Dataout(Memory_read_data));
    
    Instr_Reg ir_reg (.Clk(clk), .Reset(reset_out), .Load_ir(ir_wr), .Entrada(Memory_read_data), 
                      .Instr31_26(ir_31_26), .Instr25_21(ir_25_21), .Instr20_16(ir_20_16), .Instr15_0(ir_15_0));

    assign IR_full = {ir_31_26, ir_25_21, ir_20_16, ir_15_0};
    
    mux2x1 #(.WIDTH(5)) mux_read_reg1 (.sel(RegRs), .in0(rs), .in1(5'd29), .out(ReadReg1_final));
    
    Banco_reg reg_file (.Clk(clk), .Reset(reset_out), .RegWrite(reg_wr), .ReadReg1(ReadReg1_final), .ReadReg2(rt), .WriteReg(WriteReg_mux_out), .WriteData(Write_data_to_regs), .ReadData1(Regs_read_data1), .ReadData2(Regs_read_data2));
    
    Registrador A_reg (.Clk(clk), .Reset(reset_out), .Load(wr_A), .Entrada(Regs_read_data1), .Saida(A_out));
    Registrador B_reg (.Clk(clk), .Reset(reset_out), .Load(wr_B), .Entrada(Regs_read_data2), .Saida(B_out));
    
    mux4x1 #(.WIDTH(5)) mux_write_reg (.sel(reg_dst), .in0(rt), .in1(rd), .in2(5'd29), .in3(5'd31), .out(WriteReg_mux_out));
    
    signExtend16x32 sign_ext (.in(IR_full[15:0]), .out(SignExt_out));
    
    wire [25:0] shift_in = IR_full[25:0];
    assign Shift26_out = {shift_in, 2'b00};
    
    mux3x1 #(.WIDTH(32)) mux_alu_src_a (.sel(Alu_Src_A), .in0(PC_out), .in1(A_out), .in2(Regs_read_data1), .out(ALUSrcA_mux_out));
    
    mux4x1 #(.WIDTH(32)) mux_alu_src_b (.sel(Alu_Src_B[1:0]), .in0(B_out), .in1(32'd4), .in2(SignExt_out), .in3({SignExt_out[29:0], 2'b00}), .out(ALUSrcB_mux_out));
    
    Ula32 alu (.A(ALUSrcA_mux_out), .B(ALUSrcB_mux_out), .Seletor(Alu_Op), .S(ALU_result), .Overflow(overflow_flag), .Negativo(neg_flag), .z(et_flag), .Igual(zero_flag), .Maior(gt_flag), .Menor(lt_flag));
    
    Registrador ALUOut_reg (.Clk(clk), .Reset(reset_out), .Load(Alu_out_wr), .Entrada(ALU_result), .Saida(ALUOut_out));
    
    mux2x1_32 mux_loaded_data (.sel(load_control[0]), .in0(Memory_read_data), .in1(loaded_data_final), .out(MDR_out)); 
    
    mux9x1 #(.WIDTH(32)) mux_write_data (.sel(DataSrc), .in0(ALUOut_out), .in1(MDR_out), .in2(HI_out), .in3(LO_out), .in4(PC_out + 32'd4), .in5(Shift_out), .in6(32'b0), .in7(32'b0), .in8(32'b0), .out(Write_data_to_regs));
    
    mux2x1_32 mux_store_data (.sel(store_control[0]), .in0(B_out), .in1(Regs_read_data1), .out(store_data_to_mem)); 
    
    Registrador HI_reg (.Clk(clk), .Reset(reset_out), .Load(hi_wr), .Entrada(HI_in), .Saida(HI_out));
    Registrador LO_reg (.Clk(clk), .Reset(reset_out), .Load(lo_wr), .Entrada(LO_in), .Saida(LO_out));
    
    multiplier mult (.clk(clk), .start(mult_start), .a(A_out), .b(B_out), .hi(HI_in), .lo(LO_in), .ready(mult_ready));
    divider div (.clk(clk), .start(div_start), .a(A_out), .b(B_out), .hi(HI_in), .lo(LO_in), .ready(div_ready), .div_zero(div_zero_flag));
    
    RegDesloc shifter (.Clk(clk), .Reset(reset_out), .Entrada(B_out), .N(shamt), .Shift(shift_control), .Saida(Shift_out));
    
    Registrador EPC_reg (.Clk(clk), .Reset(reset_out), .Load(EPC_wr), .Entrada(PC_out), .Saida(EPC_out));
    
    // Lógica OpCode Inválido
    assign OpCode404_flag = (OpCode == 6'b000000) ? ~(Funct == 6'b100000 || Funct == 6'b100010 || Funct == 6'b100100 || Funct == 6'b100101 || Funct == 6'b101010 || Funct == 6'b011000 || Funct == 6'b011010 || Funct == 6'b010000 || Funct == 6'b010010 || Funct == 6'b001000 || Funct == 6'b000000 || Funct == 6'b000011 || Funct == 6'b000101 || Funct == 6'b000110) : ~(OpCode == 6'b100011 || OpCode == 6'b100000 || OpCode == 6'b101011 || OpCode == 6'b101000 || OpCode == 6'b001000 || OpCode == 6'b001100 || OpCode == 6'b001101 || OpCode == 6'b001010 || OpCode == 6'b000100 || OpCode == 6'b000101 || OpCode == 6'b000010 || OpCode == 6'b000011);
    
endmodule