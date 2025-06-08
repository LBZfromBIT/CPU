`timescale 1ns/1ps

module cpu_testbench();

// Clock and reset signals
reg clk;
reg rst;

// Instruction memory interface
wire [31:0] imem_addr;
wire [31:0] imem_data;

// Data memory interface
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
wire [31:0] dmem_rdata;

// Debug interface
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// Instantiate CPU
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

// Instantiate instruction memory
instruction_memory imem_inst (
    .pc(imem_addr),
    .instruction(imem_data)
);

// Instantiate data memory
data_memory dmem_inst (
    .clk(clk),
    .mem_read(1'b1),  // Always allow read
    .mem_write(dmem_we),
    .address(dmem_addr),
    .write_data(dmem_wdata),
    .read_data(dmem_rdata)
);

// Clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns period, 100MHz
end

// Reset and test control
initial begin
    // Initialize
    rst = 1;
    #20;
    rst = 0;
    
    // Wait longer to ensure program execution completion
    #5000;
    
    // Check sorting results
    $display("=== Sorting Results Check ===");
    $display("Original data: 4, 5, 3, 1, 2");
    $display("Sorted data:");
    $display("array[0] = %d (address 0x100)", dmem_inst.dmem[64]);
    $display("array[1] = %d (address 0x104)", dmem_inst.dmem[65]);
    $display("array[2] = %d (address 0x108)", dmem_inst.dmem[66]);
    $display("array[3] = %d (address 0x10C)", dmem_inst.dmem[67]);
    $display("array[4] = %d (address 0x110)", dmem_inst.dmem[68]);
    
    $finish;
end

// Add more monitoring in testbench
initial begin
    // Monitor memory access
    $monitor("Time: %0t, PC: 0x%h, Inst: 0x%h, dmem_addr: 0x%h, dmem_we: %b, dmem_wdata: %d", 
             $time, debug_pc, debug_instruction, dmem_addr, dmem_we, dmem_wdata);
end

// Monitor register changes
always @(posedge clk) begin
    if (!rst && debug_pc != 0) begin
        $display("[Cycle] PC=0x%h, x5=%d, x7=%d", 
                 debug_pc, 
                 cpu_inst.reg_file.registers[5],   // Outer loop
                 cpu_inst.reg_file.registers[7]);  // Inner loop
    end
end

// Monitor data memory changes
always @(posedge clk) begin
    if (dmem_we) begin
        $display("[Memory] Write to addr=0x%h, data=%d at time %0t", 
                 dmem_addr, dmem_wdata, $time);
    end
end
                                            
endmodule