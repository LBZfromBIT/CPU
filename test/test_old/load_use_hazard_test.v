`timescale 1ns/1ps

module load_use_hazard_testbench();

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
reg [31:0] instruction_memory [0:15];

// 简单的数据存储器 - 使用数组实现
reg [31:0] data_memory [0:15];

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

// 数据存储器访问逻辑
always @(posedge clk) begin
    if (dmem_we) begin
        data_memory[dmem_addr[7:2]] <= dmem_wdata;
        $display("[Memory Write] Time: %0t, Addr: 0x%h, Data: %d", 
                 $time, dmem_addr, dmem_wdata);
    end
end

always @(*) begin
    dmem_rdata = data_memory[dmem_addr[7:2]];
end

// 测试程序 - 专门测试Load-Use冒险
initial begin
    $display("=== Load-Use Hazard Detection Test ===");
    
    // 初始化复位
    rst = 1;
    
    // 初始化指令存储器 - 设计Load-Use冒险场景
    instruction_memory[0]  = 32'h00500093;  // addi x1, x0, 5      ; x1 = 5 (为了设置基地址)
    instruction_memory[1]  = 32'h00002103;  // lw   x2, 0(x0)      ; x2 = mem[0] (load指令)
    instruction_memory[2]  = 32'h00210233;  // add  x4, x2, x2     ; x4 = x2 + x2 (使用x2, 应该冒险暂停)
    instruction_memory[3]  = 32'h00100293;  // addi x5, x0, 1      ; x5 = 1 (这条指令应该延迟执行)
    instruction_memory[4]  = 32'h00000013;  // nop                  ; 空操作
    instruction_memory[5]  = 32'h00000013;  // nop                  ; 空操作
    
    // 其余指令初始化为NOP
    for (integer i = 6; i < 16; i = i + 1) begin
        instruction_memory[i] = 32'h00000013; // nop
    end
    
    // 初始化数据存储器
    data_memory[0] = 32'h0000000A;   // mem[0] = 10
    data_memory[1] = 32'h00000014;   // mem[4] = 20
    for (integer i = 2; i < 16; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // 开始测试
    #20 rst = 0;
    $display("Starting Load-Use hazard test at time %0t", $time);
    
    // 运行足够的周期观察冒险检测
    #300;
    
    // 检查结果
    $display("\n=== Test Results ===");
    $display("Expected: x1=5, x2=10, x4=20, x5=1");
    $display("Actual:   x1=%d, x2=%d, x4=%d, x5=%d", 
             cpu_inst.reg_file.registers[1], 
             cpu_inst.reg_file.registers[2], 
             cpu_inst.reg_file.registers[4],
             cpu_inst.reg_file.registers[5]);
    
    if (cpu_inst.reg_file.registers[1] == 5 && 
        cpu_inst.reg_file.registers[2] == 10 && 
        cpu_inst.reg_file.registers[4] == 20 &&
        cpu_inst.reg_file.registers[5] == 1) begin
        $display("TEST PASSED!");
    end else begin
        $display("TEST FAILED!");
    end
    
    $finish;
end

// 详细监控冒险检测逻辑
always @(posedge clk) begin
    if (!rst) begin
        $display("Time: %0t, PC: 0x%h, Inst: 0x%h, Description: %s", 
                 $time, debug_pc, debug_instruction, decode_instruction(debug_instruction));
        
        // 监控所有阶段的寄存器地址信号
        $display("  -> Pipeline Register Addresses:");
        $display("     ID: rs1=%d, rs2=%d, rd=%d", 
                 cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, cpu_inst.id_rd_addr);
        $display("     EX: rs1=%d, rs2=%d, rd=%d", 
                 cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr, cpu_inst.ex_rd_addr);
        $display("     MEM: rd=%d", cpu_inst.mem_rd_addr);
        $display("     WB: rd=%d", cpu_inst.wb_rd_addr);
        
        // 监控冒险检测单元的输入和输出
        $display("  -> Hazard Detection Unit:");
        $display("     Inputs: id_rs1=%d, id_rs2=%d, ex_rd=%d, ex_mem_read=%b", 
                 cpu_inst.hazard_detect.id_rs1, 
                 cpu_inst.hazard_detect.id_rs2,
                 cpu_inst.hazard_detect.ex_rd,
                 cpu_inst.hazard_detect.ex_mem_read);
        $display("     Outputs: pc_write=%b, if_id_write=%b, control_mux_sel=%b", 
                 cpu_inst.pc_write, 
                 cpu_inst.if_id_write,
                 cpu_inst.control_mux_sel);
        $display("     Load-Use Check: ex_rd!=0=%b, rs1_match=%b, rs2_match=%b", 
                 (cpu_inst.hazard_detect.ex_rd != 5'b00000),
                 (cpu_inst.hazard_detect.ex_rd == cpu_inst.hazard_detect.id_rs1),
                 (cpu_inst.hazard_detect.ex_rd == cpu_inst.hazard_detect.id_rs2));
        
        // 监控控制信号的传播
        $display("  -> Control Signal Pipeline:");
        $display("     ID stage: opcode=0x%h, mem_read=%b, reg_write=%b", 
                 cpu_inst.id_opcode, cpu_inst.id_mem_read, cpu_inst.id_reg_write);
        $display("     After mux: mem_read=%b, reg_write=%b (mux_sel=%b)", 
                 cpu_inst.ctrl_mem_read, cpu_inst.ctrl_reg_write, cpu_inst.control_mux_sel);
        $display("     EX stage: mem_read=%b, reg_write=%b", 
                 cpu_inst.ex_mem_read, cpu_inst.ex_reg_write);
        
        // 监控PC控制
        $display("  -> PC Control:");
        $display("     Current PC: 0x%h, Next PC: 0x%h, PC+4: 0x%h", 
                 cpu_inst.pc_current, cpu_inst.pc_next, cpu_inst.pc_plus4);
        $display("     pc_write=%b (1=update, 0=stall)", cpu_inst.pc_write);
        
        // 监控寄存器文件读写
        $display("  -> Register File:");
        $display("     Read: rs1_data=%d, rs2_data=%d", 
                 cpu_inst.id_rs1_data, cpu_inst.id_rs2_data);
        if (cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 0)
            $display("     Write: rd=%d, data=%d", cpu_inst.wb_rd_addr, cpu_inst.wb_write_data);
        
        // 特别监控Load指令
        if (cpu_inst.ex_mem_read) begin
            $display("  -> Load Instruction in EX stage:");
            $display("     EX rd=%d, will write to register x%d", 
                     cpu_inst.ex_rd_addr, cpu_inst.ex_rd_addr);
            $display("     Memory address: 0x%h", cpu_inst.ex_alu_result);
        end
        
        // 特别监控使用Load结果的指令
        if (cpu_inst.control_mux_sel) begin
            $display("  -> STALL DETECTED! Load-Use hazard causing pipeline stall");
            $display("     Stalled instruction opcode: 0x%h", cpu_inst.id_opcode);
            $display("     Reason: ID stage instruction uses register that EX stage Load will write");
        end
        
        $display(""); // 空行分隔
    end
end

// 检测PC是否按预期更新或暂停
reg [31:0] prev_pc = 32'h00000000;
always @(posedge clk) begin
    if (!rst) begin
        if (debug_pc == prev_pc && cpu_inst.pc_write == 0) begin
            $display("  -> PC STALL: PC remains at 0x%h due to hazard detection", debug_pc);
        end else if (debug_pc != prev_pc + 4 && prev_pc != 32'h00000000 && cpu_inst.pc_write == 1) begin
            $display("  -> WARNING: Unexpected PC jump from 0x%h to 0x%h", prev_pc, debug_pc);
        end
        prev_pc <= debug_pc;
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
            default: decode_instruction = "NOP";
        endcase
    end
endfunction

endmodule
