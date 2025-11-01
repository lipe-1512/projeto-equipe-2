module controlUnit (
    input wire clk, reset,
    input wire O, // Flag de Overflow da ALU
    input wire OpCode404_flag, // Flag indicando opcode inválido
    input wire div_zero, // Flag de divisão por zero
    input wire [5:0] OpCode, Funct,
    input wire zero, neg, lt, gt, et, // Flags de condição da ALU

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
    output reg Mult_div_lo,
    output reg Mult_div_hi,
    output reg Lo_wr,
    output reg hi_wr,
    output reg reset_out,
    output reg [2:0] shift_control,
    output reg [3:0] DataSrc,
    output reg RegRs
    // CORREÇÃO: Removidas as portas ShiftAmt e ShiftSrc, pois são controladas pelo datapath
    // CORREÇÃO: A porta MemDataWrite também foi removida, pois a CU não a controla diretamente.
);

// Parâmetros para estados
parameter reset_start = 6'b111111;
parameter fetch = 6'b000001;
parameter decode = 6'b000010;
parameter op404_state = 6'b000011;
parameter overflow_state = 6'b000100;
parameter zero_div_state = 6'b000101;

// Estados de instrução
parameter ADD_state = 6'b000110;
parameter SUB_state = 6'b000111;
parameter AND_state = 6'b001000;
parameter OR_state = 6'b001001;
parameter SLT_state = 6'b001010;
parameter MULT_state = 6'b001011;
parameter DIV_state = 6'b001100;
parameter MFHI_state = 6'b001101;
parameter MFLO_state = 6'b001110;
parameter JR_state = 6'b001111;
parameter SLL_state = 6'b011010;
parameter SRA_state = 6'b011011;
parameter PUSH_state = 6'b011100;
parameter POP_state = 6'b011101;
parameter LW_state = 6'b010000;
parameter SW_state = 6'b010001;
parameter ADDI_state = 6'b010010;
parameter ANDI_state = 6'b010011;
parameter ORI_state = 6'b010100;
parameter SLTI_state = 6'b010101;
parameter BEQ_state = 6'b010110;
parameter BNE_state = 6'b010111;
parameter J_state = 6'b011000;
parameter JAL_state = 6'b011001;

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
reg [5:0] counter;
reg use_sp;

// Lógica da FSM
always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= reset_start;
        counter <= 6'b000000;
    end else begin
        // Lógica de transição de estado e incremento de counter
        case (state)
            reset_start: begin
                state <= fetch;
                counter <= 6'b000000;
            end
            fetch: begin
                if (counter == 6'b000001) begin
                    state <= decode;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            decode: begin
                // Transição baseada em OpCode e Funct
                if (OpCode404_flag) begin
                    state <= op404_state;
                end else begin
                    case (OpCode)
                        R_TYPE: begin
                            case (Funct)
                                ADD_F: state <= ADD_state;
                                SUB_F: state <= SUB_state;
                                AND_F: state <= AND_state;
                                OR_F: state <= OR_state;
                                SLT_F: state <= SLT_state;
                                MULT_F: state <= MULT_state;
                                DIV_F: state <= DIV_state;
                                MFHI_F: state <= MFHI_state;
                                MFLO_F: state <= MFLO_state;
                                JR_F: state <= JR_state;
                                SLL_F: state <= SLL_state;
                                SRA_F: state <= SRA_state;
                                PUSH_F: state <= PUSH_state;
                                POP_F: state <= POP_state;
                                default: state <= op404_state;
                            endcase
                        end
                        LW_OP: state <= LW_state;
                        LB_OP: state <= LW_state;
                        SW_OP: state <= SW_state;
                        ADDI_OP: state <= ADDI_state;
                        ANDI_OP: state <= ANDI_state;
                        ORI_OP: state <= ORI_state;
                        SLTI_OP: state <= SLTI_state;
                        BEQ_OP: state <= BEQ_state;
                        BNE_OP: state <= BNE_state;
                        J_OP: state <= J_state;
                        JAL_OP: state <= JAL_state;
                        default: state <= op404_state;
                    endcase
                end
                counter <= 6'b000000;
            end
            // Estados de instrução
            ADD_state: begin
                if (counter == 6'b000001) begin
                    if (O) begin
                        state <= overflow_state;
                        counter <= 6'b000000;
                    end else begin
                        counter <= counter + 1;
                    end
                end else if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SUB_state: begin
                if (counter == 6'b000001) begin
                    if (O) begin
                        state <= overflow_state;
                        counter <= 6'b000000;
                    end else begin
                        counter <= counter + 1;
                    end
                end else if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            AND_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            OR_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SLT_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            MULT_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            DIV_state: begin
                if (counter == 6'b000001) begin
                    if (div_zero) begin
                        state <= zero_div_state;
                        counter <= 6'b000000;
                    end else begin
                        counter <= counter + 1;
                    end
                end else if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            MFHI_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            MFLO_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            JR_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SLL_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SRA_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            PUSH_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            POP_state: begin
                if (counter == 6'b000100) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            LW_state: begin
                if (counter == 6'b000100) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SW_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            ADDI_state: begin
                if (counter == 6'b000001) begin
                    if (O) begin
                        state <= overflow_state;
                        counter <= 6'b000000;
                    end else begin
                        counter <= counter + 1;
                    end
                end else if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            ANDI_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            ORI_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            SLTI_state: begin
                if (counter == 6'b000010) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            BEQ_state: begin
                if (counter == 6'b000001) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            BNE_state: begin
                if (counter == 6'b000001) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            J_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            JAL_state: begin
                if (counter == 6'b000001) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            // Estados de exceção
            op404_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            overflow_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
            zero_div_state: begin
                if (counter == 6'b000000) begin
                    state <= fetch;
                    counter <= 6'b000000;
                end else begin
                    counter <= counter + 1;
                end
            end
        endcase
    end
end

// Lógica de controle de saída
always @(*) begin
    // Valores padrão
    IorD = 3'b000;
    mem_wr = 1'b0;
    cause_control = 2'b00;
    ir_wr = 1'b0;
    reg_wr = 1'b0;
    wr_A = 1'b0;
    wr_B = 1'b0;
    mem_reg = 3'b000;
    reg_dst = 2'b00;
    Alu_Src_A = 2'b00;
    Alu_Src_B = 3'b000;
    Alu_Op = 3'b000;
    Alu_out_wr = 1'b0;
    PC_Source = 3'b000;
    PC_wr = 1'b0;
    EPC_wr = 1'b0;
    load_control = 2'b00;
    store_control = 2'b00;
    mult_start = 1'b0;
    div_start = 1'b0;
    Mult_div_lo = 1'b0;
    Mult_div_hi = 1'b0;
    Lo_wr = 1'b0;
    hi_wr = 1'b0;
    reset_out = 1'b0;
    shift_control = 3'b000;

    case (state)
        fetch: begin
            if (counter == 6'b000000) begin
                // ALUOut = PC + 4
                Alu_Src_A = 2'b00;
                Alu_Src_B = 3'b001;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                // IR = Mem[PC], PC = ALUOut
                IorD = 3'b000;
                ir_wr = 1'b1;
                PC_Source = 3'b000;
                PC_wr = 1'b1;
            end
        end
        decode: begin
            // A = Reg[rs], B = Reg[rt], ALUOut = PC + (signext(imm) << 2)
            wr_A = 1'b1;
            wr_B = 1'b1;
            Alu_Src_A = 2'b00;
            Alu_Src_B = 3'b011;
            Alu_Op = 3'b001;
            Alu_out_wr = 1'b1;
        end
        ADD_state: begin
            if (counter == 6'b000000) begin
                // Execução: ALUOut = A + B
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                if (O) begin
                    // Overflow detectado
                end else begin
                    // Write-back
                    mem_reg = 3'b011;
                    reg_dst = 2'b01;
                    reg_wr = 1'b1;
                end
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        SUB_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b010;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                if (O) begin
                end else begin
                    mem_reg = 3'b011;
                    reg_dst = 2'b01;
                    reg_wr = 1'b1;
                end
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        AND_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b011;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        OR_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b100;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        SLT_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b101;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        MULT_state: begin
            if (counter == 6'b000000) begin
                mult_start = 1'b1;
            end else if (counter == 6'b000001) begin
                Lo_wr = 1'b1;
                hi_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                // Final
            end
        end
        DIV_state: begin
            if (counter == 6'b000000) begin
                div_start = 1'b1;
            end else if (counter == 6'b000001) begin
                if (div_zero) begin
                    // Transição para zero_div_state
                end else begin
                    Lo_wr = 1'b1;
                    hi_wr = 1'b1;
                end
            end else if (counter == 6'b000010) begin
                // Final
            end
        end
        MFHI_state: begin
            if (counter == 6'b000000) begin
                mem_reg = 3'b100;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end
        end
        MFLO_state: begin
            if (counter == 6'b000000) begin
                mem_reg = 3'b101;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end
        end
        JR_state: begin
            if (counter == 6'b000000) begin
                PC_Source = 3'b010;
                PC_wr = 1'b1;
            end
        end
        SLL_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b110;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        SRA_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b111;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        PUSH_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b01;
                Alu_Src_B = 3'b001;
                Alu_Op = 3'b010;
                Alu_out_wr = 1'b1;
                use_sp = 1'b1;
            end else if (counter == 6'b000001) begin
                IorD = 3'b011;
                mem_wr = 1'b1;
                use_sp = 1'b1;
            end else if (counter == 6'b000010) begin
                mem_reg = 3'b011;
                reg_dst = 2'b11;
                reg_wr = 1'b1;
                use_sp = 1'b1;
            end
        end
        POP_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b01;
                Alu_Src_B = 3'b001;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
                use_sp = 1'b1;
            end else if (counter == 6'b000001) begin
                IorD = 3'b011;
                mem_wr = 1'b0;
                use_sp = 1'b1;
            end else if (counter == 6'b000011) begin
                mem_reg = 3'b010;
                reg_dst = 2'b01;
                reg_wr = 1'b1;
                use_sp = 1'b1;
            end else if (counter == 6'b000100) begin
                mem_reg = 3'b011;
                reg_dst = 2'b11;
                reg_wr = 1'b1;
                use_sp = 1'b1;
            end
        end
        LW_state: begin
            if (counter == 6'b000000) begin
                // ALUOut = A + signext(imm)
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                // MDR = Mem[ALUOut]
                IorD = 3'b100;
                mem_wr = 1'b0;
            end else if (counter == 6'b000010) begin
                // CORREÇÃO: Seta o load_control para LW (word) ou LB (byte)
                if (OpCode == 6'b100011) // LW
                    load_control = 2'b10;
                else if (OpCode == 6'b100000) // LB
                    load_control = 2'b01;
            end else if (counter == 6'b000011) begin
                // Reg[rt] = MDR
                mem_reg = 3'b010;
                reg_dst = 2'b00;
                reg_wr = 1'b1;
            end else if (counter == 6'b000100) begin
                reg_wr = 1'b1;
            end
        end
        SW_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                IorD = 3'b100;
                mem_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                // Final
            end
        end
        ADDI_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b001;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                if (O) begin
                end else begin
                    mem_reg = 3'b011;
                    reg_dst = 2'b00;
                    reg_wr = 1'b1;
                end
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        ANDI_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b011;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b00;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        ORI_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b100;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b00;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        SLTI_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b010;
                Alu_Op = 3'b101;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                mem_reg = 3'b011;
                reg_dst = 2'b00;
                reg_wr = 1'b1;
            end else if (counter == 6'b000010) begin
                reg_wr = 1'b1;
            end
        end
        BEQ_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b010;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                if (zero) begin
                    PC_Source = 3'b001;
                    PC_wr = 1'b1;
                end
            end
        end
        BNE_state: begin
            if (counter == 6'b000000) begin
                Alu_Src_A = 2'b10;
                Alu_Src_B = 3'b000;
                Alu_Op = 3'b010;
                Alu_out_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                if (!zero) begin
                    PC_Source = 3'b001;
                    PC_wr = 1'b1;
                end
            end
        end
        J_state: begin
            if (counter == 6'b000000) begin
                PC_Source = 3'b011;
                PC_wr = 1'b1;
            end
        end
        JAL_state: begin
            if (counter == 6'b000000) begin
                // Reg[31] = PC + 4
                mem_reg = 3'b110;
                reg_dst = 2'b10;
                reg_wr = 1'b1;
            end else if (counter == 6'b000001) begin
                PC_Source = 3'b011;
                PC_wr = 1'b1;
                reg_wr = 1'b1;
            end
        end
        op404_state: begin
            if (counter == 6'b000000) begin
                EPC_wr = 1'b1;
                cause_control = 2'b10;
                PC_Source = 3'b100;
                PC_wr = 1'b1;
            end
        end
        overflow_state: begin
            if (counter == 6'b000000) begin
                EPC_wr = 1'b1;
                cause_control = 2'b00;
                PC_Source = 3'b100;
                PC_wr = 1'b1;
            end
        end
        zero_div_state: begin
            if (counter == 6'b000000) begin
                EPC_wr = 1'b1;
                cause_control = 2'b01;
                PC_Source = 3'b100;
                PC_wr = 1'b1;
            end
        end
    endcase
end

endmodule
