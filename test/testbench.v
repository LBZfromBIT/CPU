`timescale 1ns/1ps

module cpu_testbench();

// 时钟和复位信号
reg clk;
reg rst;

// 指令存储器接口
wire [31:0] imem_addr;
wire [31:0] imem_data;

// 数据存储器接口
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
wire [31:0] dmem_rdata;

// 调试接口
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// 实例化CPU
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

// 实例化指令存储器
instruction_memory imem_inst (
    .pc(imem_addr),
    .instruction(imem_data)
);

// 实例化数据存储器
data_memory dmem_inst (
    .clk(clk),
    .mem_read(1'b1),  // 总是允许读
    .mem_write(dmem_we),
    .address(dmem_addr),
    .write_data(dmem_wdata),
    .read_data(dmem_rdata)
);

// 时钟生成
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns周期，100MHz
end

// 复位和测试控制
initial begin
    // 初始化
    rst = 1;
    #20;
    rst = 0;
    
    // 等待更长时间确保程序执行完成
    #5000;
    
    // 检查排序结果
    $display("=== 排序结果检查 ===");
    $display("原始数据: 4, 5, 3, 1, 2");
    $display("排序后数据:");
    $display("array[0] = %d (地址0x100)", dmem_inst.dmem[64]);
    $display("array[1] = %d (地址0x104)", dmem_inst.dmem[65]);
    $display("array[2] = %d (地址0x108)", dmem_inst.dmem[66]);
    $display("array[3] = %d (地址0x10C)", dmem_inst.dmem[67]);
    $display("array[4] = %d (地址0x110)", dmem_inst.dmem[68]);
    
    $finish;
end

// 在testbench中添加更多监控
initial begin
    // 监控存储器访问
    $monitor("Time: %0t, PC: 0x%h, Inst: 0x%h, dmem_addr: 0x%h, dmem_we: %b, dmem_wdata: %d", 
             $time, debug_pc, debug_instruction, dmem_addr, dmem_we, dmem_wdata);
end

// 监控寄存器变化
always @(posedge clk) begin
    if (!rst && debug_pc != 0) begin
        $display("[Cycle] PC=0x%h, x8=0x%h, x5=%d, x6=%d, x7=%d", 
                 debug_pc, 
                 cpu_inst.reg_file.registers[8],   // 基地址
                 cpu_inst.reg_file.registers[5],   // 外层循环
                 cpu_inst.reg_file.registers[6],   // 循环上限
                 cpu_inst.reg_file.registers[7]);  // 内层循环
    end
end

// 监控数据存储器变化
always @(posedge clk) begin
    if (dmem_we) begin
        $display("[Memory] Write to addr=0x%h, data=%d at time %0t", 
                 dmem_addr, dmem_wdata, $time);
    end
end
                                            
endmodule