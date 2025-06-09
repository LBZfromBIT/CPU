`timescale 1ns/1ps

module add_debug_testbench();

// ʱ�Ӻ͸�λ�ź�
reg clk;
reg rst;

// ָ��洢���ӿ�
wire [31:0] imem_addr;
reg [31:0] imem_data;

// ���ݴ洢���ӿ�
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
reg [31:0] dmem_rdata;

// ���Խӿ�
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// �򵥵�ָ��洢�� - ʹ������ʵ��
reg [31:0] instruction_memory [0:15];

// �򵥵����ݴ洢�� - ʹ������ʵ��
reg [31:0] data_memory [0:15];

// CPUʵ����
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

// ʱ������ - 10ns����
initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

// ָ��洢����ȡ�߼�
always @(*) begin
    imem_data = instruction_memory[imem_addr[7:2]]; // �ֵ�ַ
end

// ���ݴ洢�������߼�
always @(posedge clk) begin
    if (dmem_we) begin
        data_memory[dmem_addr[7:2]] <= dmem_wdata;
    end
end

always @(*) begin
    dmem_rdata = data_memory[dmem_addr[7:2]];
end

// ���Գ��� - ר�Ų���ADDָ��
initial begin
    $display("=== ADD Instruction Debug Test ===");
    
    // ��ʼ����λ
    rst = 1;
    
    // ��ʼ��ָ��洢�� - ֻ����ADD��ص���Сָ������
    instruction_memory[0]  = 32'h00500093;  // addi x1, x0, 5      ; x1 = 5
    instruction_memory[1]  = 32'h00300113;  // addi x2, x0, 3      ; x2 = 3
    instruction_memory[2]  = 32'h002081b3;  // add  x3, x1, x2     ; x3 = x1 + x2 = 8
    instruction_memory[3]  = 32'h00000013;  // nop                  ; �ղ���
    instruction_memory[4]  = 32'h00000013;  // nop                  ; �ղ���
    instruction_memory[5]  = 32'h00000013;  // nop                  ; �ղ���
    
    // ����ָ���ʼ��ΪNOP
    for (integer i = 6; i < 16; i = i + 1) begin
        instruction_memory[i] = 32'h00000013; // nop
    end
    
    // ��ʼ�����ݴ洢��
    for (integer i = 0; i < 16; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // ��ʼ����
    #20 rst = 0;
    $display("Starting ADD debug test at time %0t", $time);
    
    // �����㹻������
    #200;
    
    // �����
    $display("\n=== Test Results ===");
    $display("Expected: x1=5, x2=3, x3=8");
    $display("Actual:   x1=%d, x2=%d, x3=%d", 
             cpu_inst.reg_file.registers[1], 
             cpu_inst.reg_file.registers[2], 
             cpu_inst.reg_file.registers[3]);
    
    if (cpu_inst.reg_file.registers[1] == 5 && 
        cpu_inst.reg_file.registers[2] == 3 && 
        cpu_inst.reg_file.registers[3] == 8) begin
        $display("TEST PASSED!");
    end else begin
        $display("TEST FAILED!");
    end
    
    $finish;
end

// ��ϸ���ָ��ִ�кͿ����ź�
always @(posedge clk) begin
    if (!rst) begin
        $display("Time: %0t, PC: 0x%h, Inst: 0x%h, Description: %s", 
                 $time, debug_pc, debug_instruction, decode_instruction(debug_instruction));
        
        // ������н׶εļĴ�����ַ�ź�
        $display("  -> Pipeline Register Addresses:");
        $display("     ID: rs1=%d, rs2=%d, rd=%d", 
                 cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, cpu_inst.id_rd_addr);
        $display("     EX: rs1=%d, rs2=%d, rd=%d", 
                 cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr, cpu_inst.ex_rd_addr);
        $display("     MEM: rd=%d", cpu_inst.mem_rd_addr);
        $display("     WB: rd=%d", cpu_inst.wb_rd_addr);
        
        // ��ؼĴ����ļ���д
        $display("  -> Register File:");
        $display("     Read: rs1_data=%d, rs2_data=%d", 
                 cpu_inst.id_rs1_data, cpu_inst.id_rs2_data);
        if (cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 0)
            $display("     Write: rd=%d, data=%d", cpu_inst.wb_rd_addr, cpu_inst.wb_write_data);
        
        // ��ؿ����ź����ɺʹ���
        $display("  -> Control Signals:");
        $display("     ID stage: opcode=0x%h, alu_src=%b, reg_write=%b", 
                 cpu_inst.id_opcode, cpu_inst.id_alu_src, cpu_inst.id_reg_write);
        $display("     EX stage: alu_src=%b, reg_write=%b, alu_op=%b", 
                 cpu_inst.ex_alu_src, cpu_inst.ex_reg_write, cpu_inst.ex_alu_op);
        
        // ���ALU��������
        if (cpu_inst.ex_opcode == 7'b0110011) begin // R-typeָ��
            $display("  -> ALU Debug (R-type):");
            $display("     Input A: %d (from rs1_data=%d, forward_a=%d)", 
                     cpu_inst.ex_alu_input_a, cpu_inst.ex_rs1_data, cpu_inst.forward_a);
            $display("     Input B: %d (alu_src=%b, rs2_data=%d, immediate=%d, forward_b=%d)", 
                     cpu_inst.ex_alu_input_b, cpu_inst.ex_alu_src, 
                     cpu_inst.ex_rs2_data, cpu_inst.ex_immediate, cpu_inst.forward_b);
            $display("     ALU temp B: %d", cpu_inst.ex_alu_input_b_temp);
            $display("     Result: %d", cpu_inst.ex_alu_result);
        end
        
        // �������ת��
        $display("  -> Forwarding:");
        $display("     forward_a=%d, forward_b=%d", cpu_inst.forward_a, cpu_inst.forward_b);
        $display("     MEM forward data=%d, WB forward data=%d", 
                 cpu_inst.mem_alu_result, cpu_inst.wb_write_data);
        
        $display(""); // ���зָ�
    end
end

// �򵥵�ָ����뺯��
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
            7'b0010011: begin // I-type����
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
