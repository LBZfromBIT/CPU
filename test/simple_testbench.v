`timescale 1ns/1ps

module simple_cpu_testbench();

// 时钟和复位信号
reg clk;
reg rst;

// 指令存储器接口
wire [31:0] imem_addr;
reg [31:0] imem_data;

// 数据存储器接口
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
reg [31:0] dmem_rdata;

// 调试接口
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// 简单的指令存储器 - 使用数组实现
reg [31:0] instruction_memory [0:31];

// 简单的数据存储器 - 使用数组实现
reg [31:0] data_memory [0:63];

// CPU实例化
risc_v_cpu cpu_inst (
    .clk(clk),
    .rst(rst),
    .imem_addr(imem_addr),
    .imem_data(imem_data),
    .dmem_addr(dmem_addr),
    .dmem_wdata(dmem_wdata),
    .dmem_we(dmem_we),
    .dmem_rdata(dmem_rdata),
    .debug_pc(debug_pc),
    .debug_instruction(debug_instruction)
);

// 时钟生成 - 10ns周期
initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

// 指令存储器读取逻辑
always @(*) begin
    imem_data = instruction_memory[imem_addr[7:2]]; // 字地址
end

// 数据存储器写入逻辑
always @(posedge clk) begin
    if (dmem_we) begin
        data_memory[dmem_addr[7:2]] <= dmem_wdata;
        $display("[Memory Write] Time: %0t, Addr: 0x%h, Data: %d", 
                 $time, dmem_addr, dmem_wdata);
    end
end

// 数据存储器读取逻辑
always @(*) begin
    dmem_rdata = data_memory[dmem_addr[7:2]];
end

// 测试程序
initial begin
    $display("=== Simple CPU Test - No Control Hazards ===");
    
    // 初始化复位
    rst = 1;
    
    // 初始化指令存储器
    // 测试1: 基本立即数指令
    instruction_memory[0]  = 32'h00500093;  // addi x1, x0, 5      ; x1 = 5
    instruction_memory[1]  = 32'h00300113;  // addi x2, x0, 3      ; x2 = 3
    instruction_memory[2]  = 32'h00a00193;  // addi x3, x0, 10     ; x3 = 10
    instruction_memory[3]  = 32'hfff00213;  // addi x4, x0, -1     ; x4 = -1 (0xffffffff)
    
    // 测试2: R-type算术指令
    instruction_memory[4]  = 32'h002082b3;  // add  x5, x1, x2     ; x5 = x1 + x2 = 8
    instruction_memory[5]  = 32'h40208333;  // sub  x6, x1, x2     ; x6 = x1 - x2 = 2
    instruction_memory[6]  = 32'h0020f3b3;  // and  x7, x1, x2     ; x7 = x1 & x2 = 1
    instruction_memory[7]  = 32'h0020e433;  // or   x8, x1, x2     ; x8 = x1 | x2 = 7
    instruction_memory[8]  = 32'h0020c4b3;  // xor  x9, x1, x2     ; x9 = x1 ^ x2 = 6
    
    // 测试3: I-type算术指令
    instruction_memory[9]  = 32'h00508513;  // addi x10, x1, 5     ; x10 = x1 + 5 = 10
    instruction_memory[10] = 32'h00f0f593;  // andi x11, x1, 15    ; x11 = x1 & 15 = 5
    instruction_memory[11] = 32'h00a0e613;  // ori  x12, x1, 10    ; x12 = x1 | 10 = 15
    instruction_memory[12] = 32'h00a0c693;  // xori x13, x1, 10    ; x13 = x1 ^ 10 = 15
    
    // 测试4: 移位指令
    instruction_memory[13] = 32'h00109713;  // slli x14, x1, 1     ; x14 = x1 << 1 = 10
    instruction_memory[14] = 32'h0010d793;  // srli x15, x1, 1     ; x15 = x1 >> 1 = 2
    
    // 测试5: 比较指令
    instruction_memory[15] = 32'h0020a833;  // slt  x16, x1, x2    ; x16 = (x1 < x2) = 0
    instruction_memory[16] = 32'h0010b8b3;  // sltu x17, x1, x2    ; x17 = (x1 < x2) unsigned = 0
    instruction_memory[17] = 32'h00312913;  // slti x18, x2, 3     ; x18 = (x2 < 3) = 0
    instruction_memory[18] = 32'h00213993;  // sltiu x19, x2, 2    ; x19 = (x2 < 2) unsigned = 0
    
    // 测试6: Store指令 - 将数据存储到内存
    instruction_memory[19] = 32'h00102023;  // sw   x1, 0(x0)      ; mem[0] = x1 = 5
    instruction_memory[20] = 32'h00202223;  // sw   x2, 4(x0)      ; mem[4] = x2 = 3
    instruction_memory[21] = 32'h00502423;  // sw   x5, 8(x0)      ; mem[8] = x5 = 8
    instruction_memory[22] = 32'h00602623;  // sw   x6, 12(x0)     ; mem[12] = x6 = 2
    
    // 测试7: Load指令 - 从内存加载数据
    instruction_memory[23] = 32'h00002a03;  // lw   x20, 0(x0)     ; x20 = mem[0] = 5
    instruction_memory[24] = 32'h00402a83;  // lw   x21, 4(x0)     ; x21 = mem[4] = 3
    instruction_memory[25] = 32'h00802b03;  // lw   x22, 8(x0)     ; x22 = mem[8] = 8
    instruction_memory[26] = 32'h00c02b83;  // lw   x23, 12(x0)    ; x23 = mem[12] = 2

      // 测试8: Load-Use冒险测试
    instruction_memory[27] = 32'h01002c03;  // lw   x24, 16(x0)    ; x24 = mem[16] (应该暂停)
    instruction_memory[28] = 32'h018c0cb3;  // add  x25, x24, x24  ; x25 = x24 + x24 (需要等待x24)
    
    // 测试9: 数据前馈测试
    instruction_memory[29] = 32'h00100d13;  // addi x26, x0, 1     ; x26 = 1
    instruction_memory[30] = 32'h01ad0db3;  // add  x27, x26, x26  ; x27 = x26 + x26 = 2 (EX->EX转发)
    instruction_memory[31] = 32'h00000013;  // nop                  ; 程序结束
    
    // 初始化数据存储器
    data_memory[0] = 32'h12345678;   // mem[0]
    data_memory[1] = 32'h87654321;   // mem[4]
    data_memory[2] = 32'h11111111;   // mem[8]
    data_memory[3] = 32'h22222222;   // mem[12]
    data_memory[4] = 32'h33333333;   // mem[16]
    data_memory[5] = 32'h44444444;   // mem[20]
    
    // 其余内存位置初始化为0
    for (integer i = 6; i < 64; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // 开始测试
    #20 rst = 0;
    $display("Starting CPU test at time %0t", $time);
    
    // 运行足够的周期让所有指令执行完
    #800;
    
    // 检查结果
    $display("\n=== Test Results ===");
    $display("Expected register values:");
    $display("x1  = 5,   x2  = 3,   x3  = 10,  x4  = -1");
    $display("x5  = 8,   x6  = 2,   x7  = 1,   x8  = 7,   x9  = 6");
    $display("x10 = 10,  x11 = 5,   x12 = 15,  x13 = 15");
    $display("x14 = 10,  x15 = 2");
    $display("x16 = 0,   x17 = 0,   x18 = 0,   x19 = 0");
    $display("x20 = 5,   x21 = 3,   x22 = 8,   x23 = 2");
    $display("x26 = 1,   x27 = 2");
    
    $display("\nActual register values:");
    $display("x1  = %d,   x2  = %d,   x3  = %d,   x4  = %d", 
             cpu_inst.reg_file.registers[1], cpu_inst.reg_file.registers[2], 
             cpu_inst.reg_file.registers[3], cpu_inst.reg_file.registers[4]);
    $display("x5  = %d,   x6  = %d,   x7  = %d,   x8  = %d,   x9  = %d", 
             cpu_inst.reg_file.registers[5], cpu_inst.reg_file.registers[6], 
             cpu_inst.reg_file.registers[7], cpu_inst.reg_file.registers[8], 
             cpu_inst.reg_file.registers[9]);
    $display("x10 = %d,  x11 = %d,   x12 = %d,  x13 = %d", 
             cpu_inst.reg_file.registers[10], cpu_inst.reg_file.registers[11], 
             cpu_inst.reg_file.registers[12], cpu_inst.reg_file.registers[13]);
    $display("x14 = %d,  x15 = %d", 
             cpu_inst.reg_file.registers[14], cpu_inst.reg_file.registers[15]);
    $display("x16 = %d,   x17 = %d,   x18 = %d,   x19 = %d", 
             cpu_inst.reg_file.registers[16], cpu_inst.reg_file.registers[17], 
             cpu_inst.reg_file.registers[18], cpu_inst.reg_file.registers[19]);
    $display("x20 = %d,   x21 = %d,   x22 = %d,   x23 = %d", 
             cpu_inst.reg_file.registers[20], cpu_inst.reg_file.registers[21], 
             cpu_inst.reg_file.registers[22], cpu_inst.reg_file.registers[23]);
    $display("x26 = %d,   x27 = %d", 
             cpu_inst.reg_file.registers[26], cpu_inst.reg_file.registers[27]);
    
    $display("\nMemory contents:");
    $display("mem[0]  = 0x%h (%d)", data_memory[0], data_memory[0]);
    $display("mem[4]  = 0x%h (%d)", data_memory[1], data_memory[1]);
    $display("mem[8]  = 0x%h (%d)", data_memory[2], data_memory[2]);
    $display("mem[12] = 0x%h (%d)", data_memory[3], data_memory[3]);
    
    $finish;
end

// 监控PC变化，检测异常跳转
reg [31:0] prev_pc = 32'h00000000;
always @(posedge clk) begin
    if (!rst) begin
        if (debug_pc != prev_pc + 4 && prev_pc != 32'h00000000) begin
            $display("WARNING: Unexpected PC jump from 0x%h to 0x%h at time %0t", 
                     prev_pc, debug_pc, $time);
        end
        prev_pc <= debug_pc;
    end
end

// 监控寄存器写回
always @(posedge clk) begin
    if (!rst && cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 5'b00000) begin
        $display("  -> WB Debug: rd=%d, write_data=%d, reg_write=%b", 
                 cpu_inst.wb_rd_addr, 
                 cpu_inst.wb_write_data,
                 cpu_inst.wb_reg_write);
    end
end

// 监控指令执行
always @(posedge clk) begin
    if (!rst) begin
        $display("Time: %0t, PC: 0x%h, Inst: 0x%h, Description: %s", 
                 $time, debug_pc, debug_instruction, decode_instruction(debug_instruction));
          // 添加Store指令的调试信息
        if (debug_instruction[6:0] == 7'b0100011) begin // SW指令
            $display("  -> Store Debug: rs2_addr=%d, rs2_original=%d, rs2_forwarded=%d, forward_b=%d", 
                     debug_instruction[24:20], 
                     cpu_inst.ex_rs2_data,           // ID阶段从寄存器读取的原始数据
                     cpu_inst.ex_alu_input_b_temp,   // EX阶段经过转发的数据
                     cpu_inst.forward_b);
            $display("  -> Store Debug2: reg_x5=%d, mem_write_data=%d",
                     cpu_inst.reg_file.registers[5], // 寄存器文件中x5的当前值
                     cpu_inst.mem_rs2_data);         // 传递到MEM阶段的实际数据
        end          // 添加ADD指令的调试信息
        if (debug_instruction[6:0] == 7'b0110011 && debug_instruction[14:12] == 3'b000 && debug_instruction[31:25] == 7'b0000000) begin // ADD指令
            $display("  -> ADD Debug: rs1_addr=%d, rs2_addr=%d, rs1_data=%d, rs2_data=%d", 
                     debug_instruction[19:15],        // rs1地址
                     debug_instruction[24:20],        // rs2地址
                     cpu_inst.ex_rs1_data,           // 从寄存器读取的原始rs1数据
                     cpu_inst.ex_rs2_data);          // 从寄存器读取的原始rs2数据
            $display("  -> ADD Debug2: alu_input_a=%d, alu_input_b=%d, result=%d, rd_addr=%d", 
                     cpu_inst.ex_alu_input_a,        // ALU输入A (转发后)
                     cpu_inst.ex_alu_input_b,        // ALU输入B (转发后)
                     cpu_inst.ex_alu_result,         // ALU结果
                     debug_instruction[11:7]);        // 目标寄存器地址
            $display("  -> ADD Debug3: ex_alu_src=%b, ex_immediate=%d, alu_input_b_temp=%d", 
                     cpu_inst.ex_alu_src,            // ALU输入B选择信号
                     cpu_inst.ex_immediate,          // 立即数值
                     cpu_inst.ex_alu_input_b_temp);  // 转发后的寄存器数据
        end
          // 添加Load-Use冒险检测调试
        if (cpu_inst.hazard_detect.ex_mem_read && (cpu_inst.hazard_detect.ex_rd != 5'b00000)) begin
            $display("  -> Load-Use Check: ex_rd=%d, id_rs1=%d, id_rs2=%d, hazard=%b", 
                     cpu_inst.hazard_detect.ex_rd,
                     cpu_inst.hazard_detect.id_rs1, 
                     cpu_inst.hazard_detect.id_rs2,
                     !cpu_inst.pc_write);  // pc_write=0表示检测到冒险
            $display("  -> Instruction Decode: rs1_addr=%d, rs2_addr=%d, rd_addr=%d",
                     debug_instruction[19:15],     // 当前指令的rs1
                     debug_instruction[24:20],     // 当前指令的rs2  
                     debug_instruction[11:7]);     // 当前指令的rd
        end
    end
end

// 简单的指令解码函数
function [127:0] decode_instruction;
    input [31:0] inst;
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;
    begin
        opcode = inst[6:0];
        funct3 = inst[14:12];
        funct7 = inst[31:25];
        
        case (opcode)
            7'b0010011: begin // I-type算术
                case (funct3)
                    3'b000: decode_instruction = "ADDI";
                    3'b001: decode_instruction = "SLLI";
                    3'b010: decode_instruction = "SLTI";
                    3'b011: decode_instruction = "SLTIU";
                    3'b100: decode_instruction = "XORI";
                    3'b101: decode_instruction = (funct7[5]) ? "SRAI" : "SRLI";
                    3'b110: decode_instruction = "ORI";
                    3'b111: decode_instruction = "ANDI";
                endcase
            end
            7'b0110011: begin // R-type
                case (funct3)
                    3'b000: decode_instruction = (funct7[5]) ? "SUB" : "ADD";
                    3'b001: decode_instruction = "SLL";
                    3'b010: decode_instruction = "SLT";
                    3'b011: decode_instruction = "SLTU";
                    3'b100: decode_instruction = "XOR";
                    3'b101: decode_instruction = (funct7[5]) ? "SRA" : "SRL";
                    3'b110: decode_instruction = "OR";
                    3'b111: decode_instruction = "AND";
                endcase
            end
            7'b0000011: decode_instruction = "LW";
            7'b0100011: decode_instruction = "SW";
            default: decode_instruction = "UNKNOWN";
        endcase
    end
endfunction

endmodule
