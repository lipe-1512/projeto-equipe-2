module controlUnit (
    input wire clk, reset,
    input wire O, // Flag de Overflow da ALU
    input wire OpCode404_flag, // Flag indicando opcode inválido
    input wire div_zero, // Flag de divisão por zero
    input wire [5:0] OpCode, Funct,
    input wire zero, neg, lt, gt, et, // Flags de condição da ALU
    input wire mult_ready, div_ready, // Sinais de pronto do Multiplicador/Divisor

    // Sinais de Controle de Saída
    output reg[2:0] IorD,
    output reg mem_wr,
    output reg[1:0] cause_control,
    output reg ir_wr,
    output reg reg_wr,
    output reg wr_A,
    output reg wr_B,
    output reg[2:0] mem_reg,
    output reg[1:0] reg_dst,
    output reg[1:0] Alu_Src_A,
    output reg[2:0] Alu_Src_B,
    output reg[2:0] Alu_Op,
    output reg PCWriteCond,
    output reg Alu_out_wr,
    output reg[2:0] PC_Source,
    output reg PC_wr,
    output reg EPC_wr,
    output reg [1:0] load_control,
    output reg [1:0] store_control,
    output reg mult_start,
    output reg div_start,
    output reg Lo_wr,
    output reg hi_wr,
    output reg reset_out,
    output reg [2:0] shift_control,
    output reg [3:0] DataSrc,
    output reg RegRs
);

// Parâmetros para estados
parameter reset_start = 6'b111111;
parameter fetch = 6'b000001;
parameter decode = 6'b000010;
parameter op404_state = 6'b000011;
parameter overflow_state = 6'b000100;
parameter zero_div_state = 6'b000101;

// Estados de instrução R-type
parameter R_EXEC_state = 6'b000110;
parameter R_WB_state = 6'b000111;
parameter JR_state = 6'b001000;
parameter MULT_START_state = 6'b001001;
parameter MULT_WAIT_state = 6'b001010;
parameter DIV_START_state = 6'b001011;
parameter DIV_WAIT_state = 6'b001100;
parameter MFHI_state = 6'b001101;
parameter MFLO_state = 6'b001110;
parameter SLL_SRA_EXEC_state = 6'b001111;

// Estados de instrução I-type
parameter I_EXEC_state = 6'b010000;
parameter I_WB_state = 6'b010001;
parameter BEQ_BNE_state = 6'b010010;

// Estados de Load/Store
parameter LW_SW_ADDR_state = 6'b010011;
parameter LW_MEM_state = 6'b010100;
parameter LW_WB_state = 6'b010101;
parameter SW_MEM_state = 6'b010110;

// Estados de Jump
parameter J_state = 6'b010111;
parameter JAL_state = 6'b011000;

// Estados de Pilha
parameter PUSH_ADDR_state = 6'b011001;
parameter PUSH_MEM_state = 6'b011010;
parameter PUSH_SP_state = 6'b011011;
parameter POP_ADDR_state = 6'b011100;
parameter POP_MEM_state = 6'b011101;
parameter POP_WB_state = 6'b011110;
parameter POP_SP_state = 6'b011111;

// Opcodes
parameter R_TYPE = 6'b000000;
parameter LW_OP = 6'b100011;
parameter LB_OP = 6'b100000;
parameter SW_OP = 6'b101011;
parameter SB_OP = 6'b101000;
parameter ADDI_OP = 6'b001000;
parameter ANDI_OP = 6'b001100;
parameter ORI_OP = 6'b001101;
parameter SLTI_OP = 6'b001010;
parameter BEQ_OP = 6'b000100;
parameter BNE_OP = 6'b000101;
parameter J_OP = 6'b000010;
parameter JAL_OP = 6'b000011;

// Funct para R-type
parameter ADD_F = 6'b100000;
parameter SUB_F = 6'b100010;
parameter AND_F = 6'b100100;
parameter OR_F = 6'b100101;
parameter SLT_F = 6'b101010;
parameter MULT_F = 6'b011000;
parameter DIV_F = 6'b011010;
parameter MFHI_F = 6'b010000;
parameter MFLO_F = 6'b010010;
parameter JR_F = 6'b001000;
parameter SLL_F = 6'b000000;
parameter SRA_F = 6'b000011;
parameter PUSH_F = 6'b000101;
parameter POP_F = 6'b000110;

// Registradores de estado
reg [5:0] state;

// Lógica da FSM
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= reset_start;
    end else begin
        // Lógica de transição de estado
        case (state)
            reset_start: state <= fetch;
            
            fetch: state <= decode;
            
            decode: begin
                if (OpCode404_flag) begin
                    state <= op404_state;
                end else begin
                    case (OpCode)
                        R_TYPE: begin
                            case (Funct)
                                ADD_F, SUB_F, AND_F, OR_F, SLT_F: state <= R_EXEC_state;
                                MULT_F: state <= MULT_START_state;
                                DIV_F: state <= DIV_START_state;
                                MFHI_F: state <= MFHI_state;
                                MFLO_F: state <= MFLO_state;
                                JR_F: state <= JR_state;
                                SLL_F, SRA_F: state <= SLL_SRA_EXEC_state;
                                PUSH_F: state <= PUSH_ADDR_state; // Reintegrado
                                POP_F: state <= POP_ADDR_state;   // Reintegrado
                                default: state <= op404_state;
                            endcase
                        end
                        LW_OP, LB_OP: state <= LW_SW_ADDR_state;
                        SW_OP, SB_OP: state <= LW_SW_ADDR_state;
                        ADDI_OP, ANDI_OP, ORI_OP, SLTI_OP: state <= I_EXEC_state;
                        BEQ_OP, BNE_OP: state <= BEQ_BNE_state;
                        J_OP: state <= J_state;
                        JAL_OP: state <= JAL_state;
                        default: state <= op404_state;
                    endcase
                end
            end
            
            // R-Type
            R_EXEC_state: begin
                if (O && (Funct == ADD_F || Funct == SUB_F)) state <= overflow_state;
                else state <= R_WB_state;
            end
            R_WB_state: state <= fetch;
            SLL_SRA_EXEC_state: state <= R_WB_state;
            JR_state: state <= fetch;
            
            // Multiplicação
            MULT_START_state: state <= MULT_WAIT_state;
            MULT_WAIT_state: begin
                if (mult_ready) state <= fetch;
                else state <= MULT_WAIT_state;
            end
            
            // Divisão
            DIV_START_state: begin
                if (div_zero) state <= zero_div_state;
                else state <= DIV_WAIT_state;
            end
            DIV_WAIT_state: begin
                if (div_ready) state <= fetch;
                else state <= DIV_WAIT_state;
            end
            
            // MFHI/MFLO
            MFHI_state: state <= R_WB_state;
            MFLO_state: state <= R_WB_state;
            
            // I-Type
            I_EXEC_state: begin
                if (O && OpCode == ADDI_OP) state <= overflow_state;
                else state <= I_WB_state;
            end
            I_WB_state: state <= fetch;
            BEQ_BNE_state: state <= fetch;
            
            // Load/Store
            LW_SW_ADDR_state: begin
                if (OpCode == LW_OP || OpCode == LB_OP) state <= LW_MEM_state;
                else state <= SW_MEM_state;
            end
            LW_MEM_state: state <= LW_WB_state;
            LW_WB_state: state <= fetch;
            SW_MEM_state: state <= fetch;
            
            // Jump
            J_state: state <= fetch;
            JAL_state: state <= fetch;
            
            // Pilha (PUSH) - Reintegrado
            PUSH_ADDR_state: state <= PUSH_MEM_state;
            PUSH_MEM_state: state <= PUSH_SP_state;
            PUSH_SP_state: state <= fetch;
            
            // Pilha (POP) - Reintegrado
            POP_ADDR_state: state <= POP_MEM_state;
            POP_MEM_state: state <= POP_WB_state;
            POP_WB_state: state <= POP_SP_state;
            POP_SP_state: state <= fetch;
            
            // Exceções
            op404_state: state <= fetch;
            overflow_state: state <= fetch;
            zero_div_state: state <= fetch;
            
            default: state <= fetch;
        endcase
    end
end

// Lógica de controle de saída
always @(*) begin
    // Valores padrão (todos os sinais de controle em 0 ou valor neutro)
    IorD = 3'b000; mem_wr = 1'b0; cause_control = 2'b00; ir_wr = 1'b0;
    reg_wr = 1'b0; wr_A = 1'b0; wr_B = 1'b0; mem_reg = 3'b000;
    reg_dst = 2'b00; Alu_Src_A = 2'b00; Alu_Src_B = 3'b000; Alu_Op = 3'b000;
    PCWriteCond = 1'b0; Alu_out_wr = 1'b0; PC_Source = 3'b000; PC_wr = 1'b0;
    EPC_wr = 1'b0; load_control = 2'b00; store_control = 2'b00; mult_start = 1'b0;
    div_start = 1'b0; Lo_wr = 1'b0; hi_wr = 1'b0;
    reset_out = 1'b0; shift_control = 3'b000; DataSrc = 4'b0000; RegRs = 1'b0;

    case (state)
        reset_start: begin
            reset_out = 1'b1; // Sinal para resetar registradores auxiliares (HI/LO, etc.)
        end
        
        fetch: begin
            // Ciclo 1: ALUOut = PC + 4
            Alu_Src_A = 2'b00; // PC
            Alu_Src_B = 3'b001; // 4
            Alu_Op = 3'b001; // ADD
            Alu_out_wr = 1'b1;
            
            // Ciclo 2: IR = Mem[PC], PC = ALUOut
            IorD = 3'b000; // Endereço de instrução (PC)
            ir_wr = 1'b1;
            PC_Source = 3'b000; // ALUOut
            PC_wr = 1'b1;
        end
        
        decode: begin
            // A = Reg[rs], B = Reg[rt], ALUOut = PC + (signext(imm) << 2)
            wr_A = 1'b1;
            wr_B = 1'b1;
            Alu_Src_A = 2'b00; // PC
            Alu_Src_B = 3'b011; // SignExt << 2
            Alu_Op = 3'b001; // ADD
            Alu_out_wr = 1'b1;
        end
        
        // R-Type (ADD, SUB, AND, OR, SLT)
        R_EXEC_state: begin
            // ALUOut = A op B
            Alu_Src_A = 2'b01; // A
            Alu_Src_B = 3'b000; // B
            
            case (Funct)
                ADD_F: Alu_Op = 3'b001; // ADD
                SUB_F: Alu_Op = 3'b010; // SUB
                AND_F: Alu_Op = 3'b011; // AND
                OR_F: Alu_Op = 3'b100; // OR
                SLT_F: Alu_Op = 3'b101; // SLT
            endcase
            
            Alu_out_wr = 1'b1;
        end
        R_WB_state: begin
            // Reg[rd] = ALUOut
            DataSrc = 4'b0000; // ALUOut
            reg_dst = 2'b01; // rd
            reg_wr = 1'b1;
        end
        
        // Shift (SLL, SRA)
        SLL_SRA_EXEC_state: begin
            // Reg[rd] = Shift(B)
            
            case (Funct)
                SLL_F: shift_control = 3'b001; // SLL
                SRA_F: shift_control = 3'b010; // SRA
            endcase
            
            DataSrc = 4'b0101; // Shift_out
            reg_dst = 2'b01; // rd
            reg_wr = 1'b1;
        end
        
        // Jump Register (JR)
        JR_state: begin
            PC_Source = 3'b010; // A (endereço de salto)
            PC_wr = 1'b1;
        end
        
        // Multiplicação
        MULT_START_state: begin
            mult_start = 1'b1;
        end
        MULT_WAIT_state: begin
            // Espera por mult_ready
            if (mult_ready) begin
                Lo_wr = 1'b1;
                hi_wr = 1'b1;
            end
        end
        
        // Divisão
        DIV_START_state: begin
            div_start = 1'b1;
        end
        DIV_WAIT_state: begin
            // Espera por div_ready
            if (div_ready) begin
                Lo_wr = 1'b1;
                hi_wr = 1'b1;
            end
        end
        
        // MFHI/MFLO
        MFHI_state: begin
            // Reg[rd] = HI
            DataSrc = 4'b0010; // HI
            reg_dst = 2'b01; // rd
            reg_wr = 1'b1;
        end
        MFLO_state: begin
            // Reg[rd] = LO
            DataSrc = 4'b0011; // LO
            reg_dst = 2'b01; // rd
            reg_wr = 1'b1;
        end
        
        // I-Type (ADDI, ANDI, ORI, SLTI)
        I_EXEC_state: begin
            // ALUOut = A op SignExt(imm)
            Alu_Src_A = 2'b01; // A
            Alu_Src_B = 3'b010; // SignExt(imm)
            
            case (OpCode)
                ADDI_OP: Alu_Op = 3'b001; // ADD
                ANDI_OP: Alu_Op = 3'b011; // AND
                ORI_OP: Alu_Op = 3'b100; // OR
                SLTI_OP: Alu_Op = 3'b101; // SLT
            endcase
            
            Alu_out_wr = 1'b1;
        end
        I_WB_state: begin
            // Reg[rt] = ALUOut
            DataSrc = 4'b0000; // ALUOut
            reg_dst = 2'b00; // rt
            reg_wr = 1'b1;
        end
        
        // Branch (BEQ, BNE)
        BEQ_BNE_state: begin
            // ALUOut = A - B
            Alu_Src_A = 2'b01; // A
            Alu_Src_B = 3'b000; // B
            Alu_Op = 3'b010; // SUB
            Alu_out_wr = 1'b1;
            
            PCWriteCond = 1'b1; // Habilita escrita condicional
            PC_Source = 3'b001; // ALUOut (PC + offset)
            
            if (OpCode == BEQ_OP) begin
                // BEQ: PC_wr = zero
                if (zero) PC_wr = 1'b1;
            end else begin
                // BNE: PC_wr = !zero
                if (!zero) PC_wr = 1'b1;
            end
        end
        
        // Load/Store Address Calculation
        LW_SW_ADDR_state: begin
            // ALUOut = A + SignExt(imm)
            Alu_Src_A = 2'b01; // A
            Alu_Src_B = 3'b010; // SignExt(imm)
            Alu_Op = 3'b001; // ADD
            Alu_out_wr = 1'b1;
        end
        
        // Load Word/Byte (LW/LB)
        LW_MEM_state: begin
            // MDR = Mem[ALUOut]
            IorD = 3'b001; // Endereço de dado (ALUOut)
            mem_wr = 1'b0; // Leitura
            
            if (OpCode == LW_OP) load_control = 2'b10; // LW
            else load_control = 2'b01; // LB
        end
        LW_WB_state: begin
            // Reg[rt] = MDR
            DataSrc = 4'b0001; // MDR
            reg_dst = 2'b00; // rt
            reg_wr = 1'b1;
        end
        
        // Store Word/Byte (SW/SB)
        SW_MEM_state: begin
            // Mem[ALUOut] = B
            IorD = 3'b001; // Endereço de dado (ALUOut)
            mem_wr = 1'b1; // Escrita
            
            if (OpCode == SW_OP) store_control = 2'b10; // SW
            else store_control = 2'b01; // SB
        end
        
        // Jump (J)
        J_state: begin
            PC_Source = 3'b001; // PC + offset (ALUOut)
            PC_wr = 1'b1;
        end
        
        // Jump and Link (JAL)
        JAL_state: begin
            // Reg[31] = PC + 4
            DataSrc = 4'b0100; // PC + 4
            reg_dst = 2'b11; // R31
            reg_wr = 1'b1;
            
            // PC = PC + offset
            PC_Source = 3'b001; // PC + offset (ALUOut)
            PC_wr = 1'b1;
        end
        
        // Pilha (PUSH) - Reintegrado e Corrigido
        PUSH_ADDR_state: begin
            // ALUOut = SP - 4
            Alu_Src_A = 2'b01; // A (SP)
            Alu_Src_B = 3'b001; // 4
            Alu_Op = 3'b010; // SUB
            Alu_out_wr = 1'b1;
            RegRs = 1'b1; // Seleciona R29 para A
        end
        PUSH_MEM_state: begin
            // Mem[ALUOut] = B (dado a ser salvo)
            IorD = 3'b001; // Endereço de dado (ALUOut)
            mem_wr = 1'b1; // Escrita
            store_control = 2'b10; // SW
            RegRs = 1'b0; // B é o dado a ser salvo (Reg[rt])
        end
        PUSH_SP_state: begin
            // SP = ALUOut
            DataSrc = 4'b0000; // ALUOut
            reg_dst = 2'b10; // R29
            reg_wr = 1'b1;
            RegRs = 1'b1; // Garante que o registrador de leitura 1 seja o SP para o próximo ciclo
        end
        
        // Pilha (POP) - Reintegrado e Corrigido
        POP_ADDR_state: begin
            // ALUOut = SP + 4
            Alu_Src_A = 2'b01; // A (SP)
            Alu_Src_B = 3'b001; // 4
            Alu_Op = 3'b001; // ADD
            Alu_out_wr = 1'b1;
            RegRs = 1'b1; // Seleciona R29 para A
        end
        POP_MEM_state: begin
            // MDR = Mem[SP]
            IorD = 3'b001; // Endereço de dado (ALUOut)
            mem_wr = 1'b0; // Leitura
            load_control = 2'b10; // LW
            RegRs = 1'b1; // Garante que o registrador de leitura 1 seja o SP
        end
        POP_WB_state: begin
            // Reg[rd] = MDR
            DataSrc = 4'b0001; // MDR
            reg_dst = 2'b01; // rd
            reg_wr = 1'b1;
            RegRs = 1'b1; // Garante que o registrador de leitura 1 seja o SP
        end
        POP_SP_state: begin
            // SP = ALUOut
            DataSrc = 4'b0000; // ALUOut
            reg_dst = 2'b10; // R29
            reg_wr = 1'b1;
            RegRs = 1'b1; // Garante que o registrador de leitura 1 seja o SP
        end
        
        // Exceções
        op404_state: begin
            EPC_wr = 1'b1;
            cause_control = 2'b10; // OpCode Inválido
            PC_Source = 3'b011; // Endereço de exceção (0x000000FC)
            PC_wr = 1'b1;
        end
        overflow_state: begin
            EPC_wr = 1'b1;
            cause_control = 2'b00; // Overflow
            PC_Source = 3'b011; // Endereço de exceção (0x000000FC)
            PC_wr = 1'b1;
        end
        zero_div_state: begin
            EPC_wr = 1'b1;
            cause_control = 2'b01; // Divisão por Zero
            PC_Source = 3'b011; // Endereço de exceção (0x000000FC)
            PC_wr = 1'b1;
        end
    endcase
end

endmodule