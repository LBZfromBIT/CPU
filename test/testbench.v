`timescale 1ns/1ps

module cpu_testbench();

// ʱ�Ӻ͸�λ�ź�
reg clk;
reg rst;

// ָ��洢���ӿ�
wire [31:0] imem_addr;
wire [31:0] imem_data;

// ���ݴ洢���ӿ�
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
wire [31:0] dmem_rdata;

// ���Խӿ�
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// ʵ����CPU
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

// ʵ����ָ��洢��
instruction_memory imem_inst (
    .pc(imem_addr),
    .instruction(imem_data)
);

// ʵ�������ݴ洢��
data_memory dmem_inst (
    .clk(clk),
    .mem_read(1'b1),  // ���������
    .mem_write(dmem_we),
    .address(dmem_addr),
    .write_data(dmem_wdata),
    .read_data(dmem_rdata)
);

// ʱ������
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns���ڣ�100MHz
end

// ��λ�Ͳ��Կ���
initial begin
    // ��ʼ��
    rst = 1;
    #20;
    rst = 0;
    
    // �ȴ�����ʱ��ȷ������ִ�����
    #5000;
    
    // ���������
    $display("=== ��������� ===");
    $display("ԭʼ����: 4, 5, 3, 1, 2");
    $display("���������:");
    $display("array[0] = %d (��ַ0x100)", dmem_inst.dmem[64]);
    $display("array[1] = %d (��ַ0x104)", dmem_inst.dmem[65]);
    $display("array[2] = %d (��ַ0x108)", dmem_inst.dmem[66]);
    $display("array[3] = %d (��ַ0x10C)", dmem_inst.dmem[67]);
    $display("array[4] = %d (��ַ0x110)", dmem_inst.dmem[68]);
    
    $finish;
end

// ��testbench����Ӹ�����
initial begin
    // ��ش洢������
    $monitor("Time: %0t, PC: 0x%h, Inst: 0x%h, dmem_addr: 0x%h, dmem_we: %b, dmem_wdata: %d", 
             $time, debug_pc, debug_instruction, dmem_addr, dmem_we, dmem_wdata);
end

// ��ؼĴ����仯
always @(posedge clk) begin
    if (!rst && debug_pc != 0) begin
        $display("[Cycle] PC=0x%h, x8=0x%h, x5=%d, x6=%d, x7=%d", 
                 debug_pc, 
                 cpu_inst.reg_file.registers[8],   // ����ַ
                 cpu_inst.reg_file.registers[5],   // ���ѭ��
                 cpu_inst.reg_file.registers[6],   // ѭ������
                 cpu_inst.reg_file.registers[7]);  // �ڲ�ѭ��
    end
end

// ������ݴ洢���仯
always @(posedge clk) begin
    if (dmem_we) begin
        $display("[Memory] Write to addr=0x%h, data=%d at time %0t", 
                 dmem_addr, dmem_wdata, $time);
    end
end
                                            
endmodule