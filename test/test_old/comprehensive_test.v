`timescale 1ns/1ps

module comprehensive_testbench();

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
reg [31:0] data_memory [0:31];

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
    if (imem_addr[7:2] < 32)
        imem_data = instruction_memory[imem_addr[7:2]]; // 字地址
    else
        imem_data = 32'h00000013; // NOP for out of range
end

// 数据存储器读写逻辑
always @(posedge clk) begin
    if (dmem_we && dmem_addr[7:2] < 32) begin
        data_memory[dmem_addr[7:2]] <= dmem_wdata;
    end
end

always @(*) begin
    if (dmem_addr[7:2] < 32)
        dmem_rdata = data_memory[dmem_addr[7:2]];
    else
        dmem_rdata = 32'h00000000;
end

// 调试输出 - 每个时钟周期显示关键信息
always @(posedge clk) begin
    if (!rst && $time >= 20) begin
        $display("Time: %0t, PC: 0x%08h, Inst: 0x%08h, Description: %s", 
                 $time, debug_pc, debug_instruction, get_instruction_name(debug_instruction));
        
        // 显示管线寄存器地址
        $display("  -> Pipeline Register Addresses:");
        $display("     ID: rs1=%2d, rs2=%2d, rd=%2d", 
                 cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, cpu_inst.id_rd_addr);
        $display("     EX: rs1=%2d, rs2=%2d, rd=%2d", 
                 cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr, cpu_inst.ex_rd_addr);
        $display("     MEM: rd=%2d", cpu_inst.mem_rd_addr);
        $display("     WB: rd=%2d", cpu_inst.wb_rd_addr);
          // 显示冒险检测信息
        $display("  -> Hazard Detection Unit:");
        $display("     Inputs: id_rs1=%2d, id_rs2=%2d, ex_rd=%2d, ex_mem_read=%b",
                 cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, 
                 cpu_inst.ex_rd_addr, cpu_inst.ex_mem_read);
        $display("     Outputs: pc_write=%b, if_id_write=%b, control_mux_sel=%b",
                 cpu_inst.pc_write, cpu_inst.if_id_write, 
                 cpu_inst.control_mux_sel);
        $display("     Load-Use Check: ex_rd!=0=%b, rs1_match=%b, rs2_match=%b",
                 (cpu_inst.ex_rd_addr != 0),
                 (cpu_inst.id_rs1_addr == cpu_inst.ex_rd_addr),
                 (cpu_inst.id_rs2_addr == cpu_inst.ex_rd_addr));
          // 显示控制信号管线
        $display("  -> Control Signal Pipeline:");
        $display("     ID stage: opcode=0x%02h, mem_read=%b, reg_write=%b",
                 cpu_inst.id_instruction[6:0], cpu_inst.id_mem_read, cpu_inst.id_reg_write);
        $display("     After mux: mem_read=%b, reg_write=%b (mux_sel=%b)",
                 cpu_inst.ex_mem_read, cpu_inst.ex_reg_write, cpu_inst.control_mux_sel);
        $display("     EX stage: mem_read=%b, reg_write=%b",
                 cpu_inst.ex_mem_read, cpu_inst.ex_reg_write);
          // 显示PC控制
        $display("  -> PC Control:");
        $display("     Current PC: 0x%08h, Next PC: 0x%08h, PC+4: 0x%08h",
                 cpu_inst.pc_current, cpu_inst.pc_next, cpu_inst.pc_plus4);
        $display("     pc_write=%b (1=update, 0=stall)", cpu_inst.pc_write);
          // 显示寄存器文件读取
        $display("  -> Register File:");
        $display("     Read: rs1_data=%9d, rs2_data=%9d", 
                 cpu_inst.id_rs1_data, cpu_inst.id_rs2_data);
        
        // 显示寄存器写入（如果发生）
        if (cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 0) begin
            $display("     Write: rd=%2d, data=%9d", 
                     cpu_inst.wb_rd_addr, cpu_inst.wb_write_data);
        end
          // 检测Load指令在EX阶段
        if (cpu_inst.ex_mem_read) begin
            $display("  -> Load Instruction in EX stage:");
            $display("     EX rd=%2d, will write to register x%2d", 
                     cpu_inst.ex_rd_addr, cpu_inst.ex_rd_addr);
            $display("     Memory address: 0x%08h", cpu_inst.ex_alu_result);
        end
        
        // 检测暂停
        if (!cpu_inst.pc_write) begin
            $display("  -> STALL DETECTED! Load-Use hazard causing pipeline stall");
            $display("     Stalled instruction opcode: 0x%02h", cpu_inst.id_instruction[6:0]);
            $display("     Reason: ID stage instruction uses register that EX stage Load will write");
        end
        
        // 检测数据前递
        if (cpu_inst.forward_a != 2'b00 || cpu_inst.forward_b != 2'b00) begin
            $display("  -> DATA FORWARDING:");
            $display("     Forward A: %b, Forward B: %b", cpu_inst.forward_a, cpu_inst.forward_b);
        end
          // 检测Store指令
        if (cpu_inst.ex_mem_write) begin
            $display("  -> Store Instruction in EX stage:");
            $display("     Address: 0x%08h, Data: %d", cpu_inst.ex_alu_result, cpu_inst.ex_rs2_data);
        end
        
        $display("");
    end
end

// 指令名称解码函数
function [127:0] get_instruction_name;
    input [31:0] instruction;
    case (instruction[6:0])
        7'b0010011: get_instruction_name = "             ADDI";  // I-type
        7'b0110011: get_instruction_name = "              ADD";  // R-type 
        7'b0000011: get_instruction_name = "               LW";  // Load
        7'b0100011: get_instruction_name = "               SW";  // Store
        7'b1100011: get_instruction_name = "              BEQ";  // Branch
        7'b1101111: get_instruction_name = "              JAL";  // Jump
        default:    get_instruction_name = "              NOP";  // Unknown/NOP
    endcase
endfunction

// 综合测试程序
initial begin
    $display("=== Comprehensive CPU Test ===");
    
    // 初始化复位
    rst = 1;
    
    // 初始化指令存储器 - 更复杂的测试程序
    instruction_memory[0]  = 32'h00A00093;  // addi x1, x0, 10     ; x1 = 10
    instruction_memory[1]  = 32'h00500113;  // addi x2, x0, 5      ; x2 = 5  
    instruction_memory[2]  = 32'h00000193;  // addi x3, x0, 0      ; x3 = 0 (base address)
    instruction_memory[3]  = 32'h0001A023;  // sw   x0, 0(x3)      ; mem[0] = 0
    instruction_memory[4]  = 32'h0011A023;  // sw   x1, 0(x3)      ; mem[0] = x1 = 10
    instruction_memory[5]  = 32'h0001A203;  // lw   x4, 0(x3)      ; x4 = mem[0] = 10 (Load)
    instruction_memory[6]  = 32'h00420233;  // add  x4, x4, x4     ; x4 = x4 + x4 = 20 (Load-Use hazard!)
    instruction_memory[7]  = 32'h001102B3;  // add  x5, x2, x1     ; x5 = x2 + x1 = 15
    instruction_memory[8]  = 32'h40520333;  // sub  x6, x4, x5     ; x6 = x4 - x5 = 5
    instruction_memory[9]  = 32'h0061A423;  // sw   x6, 8(x3)      ; mem[8] = x6 = 5
    instruction_memory[10] = 32'h0081A383;  // lw   x7, 8(x3)      ; x7 = mem[8] = 5
    instruction_memory[11] = 32'h007384B3;  // add  x9, x7, x7     ; x9 = x7 + x7 = 10 (Load-Use hazard!)
    instruction_memory[12] = 32'h00148493;  // addi x9, x9, 1      ; x9 = x9 + 1 = 11
    instruction_memory[13] = 32'h00000013;  // nop                  ; 结束程序
    instruction_memory[14] = 32'h00000013;  // nop
    instruction_memory[15] = 32'h00000013;  // nop
    
    // 其余指令初始化为NOP
    for (integer i = 16; i < 32; i = i + 1) begin
        instruction_memory[i] = 32'h00000013; // nop
    end
    
    // 初始化数据存储器
    for (integer i = 0; i < 32; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // 开始测试
    #20 rst = 0;
    $display("Starting comprehensive test at time %0t", $time);
    
    // 运行足够的周期让所有指令执行完成
    #500;
    
    // 检查结果
    $display("\n=== Test Results ===");
    $display("Register values:");
    $display("x1 = %d (expected: 10)", cpu_inst.reg_file.registers[1]);
    $display("x2 = %d (expected: 5)",  cpu_inst.reg_file.registers[2]); 
    $display("x3 = %d (expected: 0)",  cpu_inst.reg_file.registers[3]);
    $display("x4 = %d (expected: 20)", cpu_inst.reg_file.registers[4]);
    $display("x5 = %d (expected: 15)", cpu_inst.reg_file.registers[5]);
    $display("x6 = %d (expected: 5)",  cpu_inst.reg_file.registers[6]);
    $display("x7 = %d (expected: 5)",  cpu_inst.reg_file.registers[7]);
    $display("x9 = %d (expected: 11)", cpu_inst.reg_file.registers[9]);
    
    $display("\nMemory values:");
    $display("mem[0] = %d (expected: 10)", data_memory[0]);
    $display("mem[8] = %d (expected: 5)",  data_memory[2]); // word address 2 = byte address 8
    
    // 验证结果
    if (cpu_inst.reg_file.registers[1] == 10 && 
        cpu_inst.reg_file.registers[2] == 5 && 
        cpu_inst.reg_file.registers[3] == 0 &&
        cpu_inst.reg_file.registers[4] == 20 &&
        cpu_inst.reg_file.registers[5] == 15 &&
        cpu_inst.reg_file.registers[6] == 5 &&
        cpu_inst.reg_file.registers[7] == 5 &&
        cpu_inst.reg_file.registers[9] == 11 &&
        data_memory[0] == 10 &&
        data_memory[2] == 5) begin
        $display("\nCOMPREHENSIVE TEST PASSED!");
    end else begin
        $display("\nCOMPREHENSIVE TEST FAILED!");
    end
    
    $finish;
end

endmodule
