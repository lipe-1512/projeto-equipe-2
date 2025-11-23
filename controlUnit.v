module controlUnit (
    input wire clk, reset,
    input wire O, OpCode404_flag, div_zero,
    input wire [5:0] OpCode, Funct,
    input wire zero, neg, lt, gt, et,
    input wire mult_ready, div_ready,

    output reg[2:0] IorD,
    output reg mem_wr,
    output reg[1:0] cause_control,
    output reg ir_wr, reg_wr, wr_A, wr_B,
    output reg[2:0] mem_reg,
    output reg[1:0] reg_dst, Alu_Src_A,
    output reg[2:0] Alu_Src_B, Alu_Op,
    output reg PCWriteCond, Alu_out_wr,
    output reg[2:0] PC_Source,
    output reg PC_wr, EPC_wr,
    output reg [1:0] load_control, store_control,
    output reg mult_start, div_start, Lo_wr, hi_wr, reset_out,
    output reg [2:0] shift_control,
    output reg [3:0] DataSrc,
    output reg RegRs,
    output reg [3:0] mem_wr_byte_enable
);

// Mapeamento de Estados
parameter reset_start = 6'd0;
parameter fetch = 6'd1;
parameter decode = 6'd2;
parameter op404_state = 6'd3;
parameter overflow_state = 6'd4;
parameter zero_div_state = 6'd5;
parameter R_EXEC_state = 6'd6;
parameter R_WB_state = 6'd7;
parameter JR_state = 6'd8;
parameter MULT_START_state = 6'd9;
parameter MULT_WAIT_state = 6'd10;
parameter DIV_START_state = 6'd11;
parameter DIV_WAIT_state = 6'd12;
parameter MFHI_state = 6'd13;
parameter MFLO_state = 6'd14;
parameter SLL_SRA_EXEC_state = 6'd15;
parameter I_EXEC_state = 6'd16;
parameter I_WB_state = 6'd17;
parameter BEQ_BNE_state = 6'd18;
parameter LW_SW_ADDR_state = 6'd19;
parameter LW_MEM_state = 6'd20;
parameter LW_WB_state = 6'd21;
parameter SW_MEM_state = 6'd22;
parameter J_state = 6'd23;
parameter JAL_state = 6'd24;
parameter PUSH_ADDR_state = 6'd25;
parameter PUSH_MEM_state = 6'd26;
parameter PUSH_SP_state = 6'd27;
parameter POP_ADDR_state = 6'd28;
parameter POP_MEM_state = 6'd29;
parameter POP_WB_state = 6'd30;
parameter POP_SP_state = 6'd31;
parameter LUI_WB_state = 6'd32; 

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
parameter LUI_OP = 6'b001111;

// Funct R-type
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

reg [5:0] state;

initial state = reset_start;

// Lógica Sequencial (FSM)
always @(posedge clk or posedge reset) begin
    if (reset) state <= reset_start;
    else begin
        case (state)
            reset_start: state <= fetch;
            fetch: state <= decode;
            decode: begin
                if (OpCode404_flag) state <= op404_state;
                else case (OpCode)
                    R_TYPE: begin
                        case(Funct)
                            MULT_F: state <= MULT_START_state;
                            DIV_F: state <= DIV_START_state;
                            MFHI_F: state <= MFHI_state;
                            MFLO_F: state <= MFLO_state;
                            JR_F: state <= JR_state;
                            SLL_F, SRA_F: state <= SLL_SRA_EXEC_state;
                            PUSH_F: state <= PUSH_ADDR_state;
                            POP_F: state <= POP_ADDR_state;
                            default: state <= R_EXEC_state; // ADD, SUB, AND, OR, SLT
                        endcase
                    end
                    LW_OP, LB_OP, SW_OP, SB_OP: state <= LW_SW_ADDR_state;
                    ADDI_OP, ANDI_OP, ORI_OP, SLTI_OP: state <= I_EXEC_state;
                    BEQ_OP, BNE_OP: state <= BEQ_BNE_state;
                    J_OP: state <= J_state;
                    JAL_OP: state <= JAL_state;
                    LUI_OP: state <= LUI_WB_state;
                    default: state <= op404_state;
                endcase
            end
            
            R_EXEC_state: state <= (O && (Funct==ADD_F || Funct==SUB_F)) ? overflow_state : R_WB_state;
            R_WB_state: state <= fetch;
            SLL_SRA_EXEC_state: state <= fetch;
            JR_state: state <= fetch;
            MULT_START_state: state <= MULT_WAIT_state;
            MULT_WAIT_state: state <= (mult_ready) ? fetch : MULT_WAIT_state;
            DIV_START_state: state <= (div_zero) ? zero_div_state : DIV_WAIT_state;
            DIV_WAIT_state: state <= (div_ready) ? fetch : DIV_WAIT_state;
            MFHI_state: state <= R_WB_state;
            MFLO_state: state <= R_WB_state;
            
            I_EXEC_state: state <= (O && OpCode == ADDI_OP) ? overflow_state : I_WB_state;
            I_WB_state: state <= fetch;
            BEQ_BNE_state: state <= fetch;
            
            LW_SW_ADDR_state: state <= (OpCode == LW_OP || OpCode == LB_OP) ? LW_MEM_state : SW_MEM_state;
            LW_MEM_state: state <= LW_WB_state;
            LW_WB_state: state <= fetch;
            SW_MEM_state: state <= fetch;
            
            J_state: state <= fetch;
            JAL_state: state <= fetch;
            LUI_WB_state: state <= fetch;
            
            PUSH_ADDR_state: state <= PUSH_MEM_state;
            PUSH_MEM_state: state <= PUSH_SP_state;
            PUSH_SP_state: state <= fetch;
            POP_ADDR_state: state <= POP_MEM_state;
            POP_MEM_state: state <= POP_WB_state;
            POP_WB_state: state <= POP_SP_state;
            POP_SP_state: state <= fetch;
            
            op404_state, overflow_state, zero_div_state: state <= fetch;
            default: state <= fetch;
        endcase
    end
end

// Lógica Combinacional (Saídas)
always @(*) begin
    // Defaults
    IorD = 0; mem_wr = 0; cause_control = 0; ir_wr = 0; reg_wr = 0; wr_A = 0; wr_B = 0; mem_reg = 0;
    reg_dst = 0; Alu_Src_A = 0; Alu_Src_B = 0; Alu_Op = 0; PCWriteCond = 0; Alu_out_wr = 0;
    PC_Source = 0; PC_wr = 0; EPC_wr = 0; load_control = 0; store_control = 0; mult_start = 0;
    div_start = 0; Lo_wr = 0; hi_wr = 0; reset_out = 0; shift_control = 0; DataSrc = 0;
    RegRs = 0; mem_wr_byte_enable = 0;

    case (state)
        reset_start: reset_out = 1;
        fetch: begin
            IorD = 0; Alu_Src_A = 0; Alu_Src_B = 1; Alu_Op = 1; PC_Source = 0; ir_wr = 1; PC_wr = 1;
        end
        decode: begin
            wr_A = 1; wr_B = 1; Alu_Src_A = 0; Alu_Src_B = 3; Alu_Op = 1; Alu_out_wr = 1;
        end
        R_EXEC_state: begin
            Alu_Src_A = 1; Alu_Src_B = 0; Alu_out_wr = 1;
            case(Funct)
                ADD_F: Alu_Op=1; SUB_F: Alu_Op=2; AND_F: Alu_Op=3; OR_F: Alu_Op=4; SLT_F: Alu_Op=5;
                default: Alu_Op=0;
            endcase
        end
        R_WB_state: begin reg_dst = 1; reg_wr = 1; DataSrc = 0; end
        SLL_SRA_EXEC_state: begin
            reg_dst = 1; reg_wr = 1; DataSrc = 5;
            shift_control = (Funct == SLL_F) ? 1 : 2;
        end
        JR_state: begin PC_Source = 2; PC_wr = 1; end
        MULT_START_state: mult_start = 1;
        MULT_WAIT_state: if(mult_ready) begin Lo_wr=1; hi_wr=1; end
        DIV_START_state: div_start = 1;
        DIV_WAIT_state: if(div_ready) begin Lo_wr=1; hi_wr=1; end
        MFHI_state: begin reg_dst=1; reg_wr=1; DataSrc=2; end
        MFLO_state: begin reg_dst=1; reg_wr=1; DataSrc=3; end
        I_EXEC_state: begin
            Alu_Src_A = 1; Alu_Src_B = 2; Alu_out_wr = 1;
            case(OpCode)
                ADDI_OP: Alu_Op=1; ANDI_OP: Alu_Op=3; ORI_OP: Alu_Op=4; SLTI_OP: Alu_Op=5;
            endcase
        end
        I_WB_state: begin reg_dst = 0; reg_wr = 1; DataSrc = 0; end
        BEQ_BNE_state: begin
            Alu_Src_A = 1; Alu_Src_B = 0; Alu_Op = 2; PCWriteCond = 1; PC_Source = 1;
            if((OpCode==BEQ_OP && zero) || (OpCode==BNE_OP && !zero)) PC_wr = 1;
        end
        LW_SW_ADDR_state: begin Alu_Src_A = 1; Alu_Src_B = 2; Alu_Op = 1; Alu_out_wr = 1; end
        LW_MEM_state: begin IorD = 1; mem_wr = 0; load_control = (OpCode == LW_OP) ? 2 : 1; end
        LW_WB_state: begin reg_dst = 0; reg_wr = 1; DataSrc = 1; end
        SW_MEM_state: begin IorD = 1; mem_wr = 1; mem_wr_byte_enable = 15; store_control = (OpCode == SW_OP) ? 2 : 1; end
        J_state: begin PC_Source = 1; PC_wr = 1; end
        JAL_state: begin PC_Source = 1; PC_wr = 1; reg_dst = 3; reg_wr = 1; DataSrc = 4; end
        LUI_WB_state: begin reg_dst = 0; reg_wr = 1; DataSrc = 6; end
        PUSH_ADDR_state: begin Alu_Src_A=1; Alu_Src_B=1; Alu_Op=2; Alu_out_wr=1; RegRs=1; end
        PUSH_MEM_state: begin IorD=1; mem_wr=1; mem_wr_byte_enable=15; store_control=2; RegRs=0; end
        PUSH_SP_state: begin reg_dst=2; reg_wr=1; DataSrc=0; RegRs=1; end
        POP_ADDR_state: begin Alu_Src_A=1; Alu_Src_B=1; Alu_Op=1; Alu_out_wr=1; RegRs=1; end
        POP_MEM_state: begin IorD=1; mem_wr=0; load_control=2; RegRs=1; end
        POP_WB_state: begin reg_dst=1; reg_wr=1; DataSrc=1; RegRs=1; end
        POP_SP_state: begin reg_dst=2; reg_wr=1; DataSrc=0; RegRs=1; end
        op404_state: begin EPC_wr=1; cause_control=2; PC_Source=3; PC_wr=1; end
        overflow_state: begin EPC_wr=1; cause_control=0; PC_Source=3; PC_wr=1; end
        zero_div_state: begin EPC_wr=1; cause_control=1; PC_Source=3; PC_wr=1; end
    endcase
end
endmodule