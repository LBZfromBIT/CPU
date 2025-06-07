module control_unit (
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    
    // 输出控制信号
    output reg reg_write,      // 寄存器写使能
    output reg mem_read,       // 内存读使能
    output reg mem_write,      // 内存写使能
    output reg branch,         // 分支指令
    output reg jump,           // 跳转指令
    output reg alu_src,        // ALU第二个操作数选择（0:寄存器, 1:立即数）
    output reg mem_to_reg,     // 写回寄存器数据选择（0:ALU结果, 1:内存数据）
    output reg [1:0] alu_op,   // ALU操作类型
    output reg pc_src          // PC来源选择（0：顺序执行 1：PC+立即数）

// ALU操作类型定义
parameter ALU_ADD  = 2'b00;  // Load/Store指令使用加法计算地址
parameter ALU_SUB  = 2'b01;  // Branch指令使用减法比较
parameter ALU_FUNC = 2'b10;  // R-type指令，根据funct字段确定
parameter ALU_IMM  = 2'b11;  // I-type算术指令

always @(*) begin
    // 默认值
    reg_write   = 1'b0;
    mem_read    = 1'b0;
    mem_write   = 1'b0;
    branch      = 1'b0;
    jump        = 1'b0;
    alu_src     = 1'b0;
    mem_to_reg  = 1'b0;
    alu_op      = ALU_ADD;
    pc_src      = 1'b0;
    
    case (opcode)
        7'b0110011: begin // R-type instructions (ADD, SUB, AND, OR, XOR, SLT, SLTU, SLL, SRL, SRA)
            reg_write = 1'b1;
            alu_src   = 1'b0;  // 使用寄存器数据
            alu_op    = ALU_FUNC;
        end
        
        7'b0010011: begin // I-type arithmetic (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
            reg_write = 1'b1;
            alu_src   = 1'b1;  // 使用立即数
            alu_op    = ALU_IMM;
        end
        
        7'b0000011: begin // Load instructions (LW, LH, LB, LHU, LBU)
            reg_write  = 1'b1;
            mem_read   = 1'b1;
            alu_src    = 1'b1;  // 地址计算使用立即数
            mem_to_reg = 1'b1;  // 写回内存数据
            alu_op     = ALU_ADD;
        end
        
        7'b0100011: begin // Store instructions (SW, SH, SB)
            mem_write = 1'b1;
            alu_src   = 1'b1;   // 地址计算使用立即数
            alu_op    = ALU_ADD;
        end
        
        7'b1100011: begin // Branch instructions (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            branch = 1'b1;
            alu_op = ALU_SUB;    // 比较使用减法
            //先默认分支不跳转
        end
        
        7'b1101111: begin // JAL
            reg_write = 1'b1;
            jump      = 1'b1;
            pc_src    = 1'b1;
        end
        
        default: begin
            // 保持默认值
        end
    endcase
end

endmodule

module alu_control (
    input [1:0] alu_op,        // 来自主控制单元
    input [2:0] funct3,        // 指令的funct3字段
    input [6:0] funct7,        // 指令的funct7字段
    output reg [3:0] alu_control_out
);

// ALU控制信号定义
parameter ALU_ADD  = 4'b0000;
parameter ALU_SUB  = 4'b0001;
parameter ALU_AND  = 4'b0010;
parameter ALU_OR   = 4'b0011;
parameter ALU_XOR  = 4'b0100;
parameter ALU_SLT  = 4'b0101;
parameter ALU_SLTU = 4'b0110;
parameter ALU_SLL  = 4'b0111;
parameter ALU_SRL  = 4'b1000;
parameter ALU_SRA  = 4'b1001;

always @(*) begin
    case (alu_op)
        2'b00: // 加法（Load/Store）
            alu_control_out = ALU_ADD;
            
        2'b01: // 减法（Branch比较）
            alu_control_out = ALU_SUB;
            
        2'b10: begin // R-type指令
            case (funct3)
                3'b000: // ADD (funct7[5] = 0) / SUB (funct7[5] = 1)
                    alu_control_out = (funct7[5]) ? ALU_SUB : ALU_ADD;
                3'b001: // SLL
                    alu_control_out = ALU_SLL;
                3'b010: // SLT
                    alu_control_out = ALU_SLT;
                3'b011: // SLTU
                    alu_control_out = ALU_SLTU;
                3'b100: // XOR
                    alu_control_out = ALU_XOR;
                3'b101: // SRL (funct7[5] = 0) / SRA (funct7[5] = 1)
                    alu_control_out = (funct7[5]) ? ALU_SRA : ALU_SRL;
                3'b110: // OR
                    alu_control_out = ALU_OR;
                3'b111: // AND
                    alu_control_out = ALU_AND;
                default:
                    alu_control_out = ALU_ADD;
            endcase
        end
        
        2'b11: begin // I-type算术指令
            case (funct3)
                3'b000: // ADDI
                    alu_control_out = ALU_ADD;
                3'b001: // SLLI
                    alu_control_out = ALU_SLL;
                3'b010: // SLTI
                    alu_control_out = ALU_SLT;
                3'b011: // SLTIU
                    alu_control_out = ALU_SLTU;
                3'b100: // XORI
                    alu_control_out = ALU_XOR;
                3'b101: // SRLI(funct7[5] = 0)/SRAI(funct7[5] = 1)
                    alu_control_out = (funct7[5]) ? ALU_SRA : ALU_SRL;
                3'b110: // ORI
                    alu_control_out = ALU_OR;
                3'b111: // ANDI
                    alu_control_out = ALU_AND;
                default:
                    alu_control_out = ALU_ADD;
            endcase
        end
        
        default:
            alu_control_out = ALU_ADD;
    endcase
end

endmodule

module branch_control (
    input branch,              // 来自主控制单元的分支信号
    input [2:0] funct3,        // 分支指令类型
    input [31:0] rs1_data,     // 第一个源寄存器数据
    input [31:0] rs2_data,     // 第二个源寄存器数据
    output reg branch_taken    // 分支是否跳转(0不跳转 1跳转)
);

always @(*) begin
    branch_taken = 1'b0;
    
    if (branch) begin
        case (funct3)
            3'b000: // BEQ
                branch_taken = (rs1_data == rs2_data);
            3'b001: // BNE
                branch_taken = (rs1_data != rs2_data);
            3'b100: // BLT
                branch_taken = ($signed(rs1_data) < $signed(rs2_data));
            3'b101: // BGE
                branch_taken = ($signed(rs1_data) >= $signed(rs2_data));
            3'b110: // BLTU
                branch_taken = (rs1_data < rs2_data);
            3'b111: // BGEU
                branch_taken = (rs1_data >= rs2_data);
            default:
                branch_taken = 1'b0;
        endcase
    end
end

endmodule