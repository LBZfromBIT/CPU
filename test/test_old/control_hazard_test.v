`timescale 1ns/1ps

module control_hazard_testbench();

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
reg [31:0] instruction_memory [0:63];

// �򵥵����ݴ洢�� - ʹ������ʵ��
reg [31:0] data_memory [0:31];

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
    if (imem_addr[7:2] < 64)
        imem_data = instruction_memory[imem_addr[7:2]]; // �ֵ�ַ
    else
        imem_data = 32'h00000013; // NOP for out of range
end

// ���ݴ洢����д�߼�
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

// ������� - ÿ��ʱ��������ʾ�ؼ���Ϣ
always @(posedge clk) begin
    if (!rst && $time >= 20) begin
        $display("Time: %0t, PC: 0x%08h, Inst: 0x%08h, Description: %s", 
                 $time, debug_pc, debug_instruction, get_instruction_name(debug_instruction));
        
        // ��ʾ���߼Ĵ�����ַ
        $display("  -> Pipeline Register Addresses:");
        $display("     ID: rs1=%2d, rs2=%2d, rd=%2d", 
                 cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, cpu_inst.id_rd_addr);
        $display("     EX: rs1=%2d, rs2=%2d, rd=%2d", 
                 cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr, cpu_inst.ex_rd_addr);
        
        // ��ʾ��֧/��ת�����Ϣ
        if (cpu_inst.ex_branch || cpu_inst.ex_jump) begin
            $display("  -> Control Transfer:");
            $display("     EX Branch: %b, Jump: %b", cpu_inst.ex_branch, cpu_inst.ex_jump);
            if (cpu_inst.ex_branch) begin
                $display("     Branch Type (funct3): %b", cpu_inst.ex_funct3);
                $display("     Compare: rs1=%d vs rs2=%d", 
                         cpu_inst.ex_alu_input_a, cpu_inst.ex_alu_input_b_temp);
                $display("     Branch Taken: %b", cpu_inst.branch_taken);
                $display("     Branch Target: 0x%08h", cpu_inst.branch_target);
            end
            if (cpu_inst.ex_jump) begin
                $display("     Jump Target: 0x%08h", cpu_inst.jump_target);
            end
        end
        
        // ��ʾPC����
        $display("  -> PC Control:");
        $display("     Current PC: 0x%08h, Next PC: 0x%08h, PC+4: 0x%08h",
                 cpu_inst.pc_current, cpu_inst.pc_next, cpu_inst.pc_plus4);
        $display("     pc_write=%b (1=update, 0=stall)", cpu_inst.pc_write);
        
        // �����߳�ˢ
        if ((cpu_inst.branch_taken && cpu_inst.ex_branch) || cpu_inst.ex_jump) begin
            $display("  -> PIPELINE FLUSH!");
            $display("     Reason: %s", cpu_inst.ex_branch ? "Branch taken" : "Jump instruction");
            $display("     Flushing IF/ID and ID/EX stages");
        end
        
        // ��ʾ�Ĵ����ļ���ȡ
        $display("  -> Register File:");
        $display("     Read: rs1_data=%9d, rs2_data=%9d", 
                 cpu_inst.id_rs1_data, cpu_inst.id_rs2_data);
        
        // ��ʾ�Ĵ���д�루���������
        if (cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 0) begin
            $display("     Write: rd=%2d, data=%9d", 
                     cpu_inst.wb_rd_addr, cpu_inst.wb_write_data);
        end
        
        // �����ͣ
        if (!cpu_inst.pc_write) begin
            $display("  -> STALL DETECTED! Data hazard causing pipeline stall");
        end
        
        // �������ǰ��
        if (cpu_inst.forward_a != 2'b00 || cpu_inst.forward_b != 2'b00) begin
            $display("  -> DATA FORWARDING:");
            $display("     Forward A: %b, Forward B: %b", cpu_inst.forward_a, cpu_inst.forward_b);
        end
        
        $display("");
    end
end

// ָ�����ƽ��뺯��
function [127:0] get_instruction_name;
    input [31:0] instruction;
    case (instruction[6:0])
        7'b0010011: get_instruction_name = "             ADDI";  // I-type
        7'b0110011: begin // R-type
            case (instruction[14:12])
                3'b000: get_instruction_name = instruction[30] ? "              SUB" : "              ADD";
                default: get_instruction_name = "            R-TYPE";
            endcase
        end
        7'b0000011: get_instruction_name = "               LW";  // Load
        7'b0100011: get_instruction_name = "               SW";  // Store
        7'b1100011: begin // Branch
            case (instruction[14:12])
                3'b000: get_instruction_name = "              BEQ";
                3'b001: get_instruction_name = "              BNE";
                3'b100: get_instruction_name = "              BLT";
                3'b101: get_instruction_name = "              BGE";
                3'b110: get_instruction_name = "             BLTU";
                3'b111: get_instruction_name = "             BGEU";
                default: get_instruction_name = "           BRANCH";
            endcase
        end
        7'b1101111: get_instruction_name = "              JAL";  // Jump
        default:    get_instruction_name = "              NOP";  // Unknown/NOP
    endcase
endfunction

// ����ð�ղ��Գ���
initial begin
    $display("=== Control Hazard Test ===");
    
    // ��ʼ����λ
    rst = 1;
    
    // ��ʼ��ָ��洢�� - ����ð�ղ��Գ���
    // ����1: �򵥷�֧����������
    instruction_memory[0]  = 32'h00500093;  // addi x1, x0, 5      ; x1 = 5
    instruction_memory[1]  = 32'h00300113;  // addi x2, x0, 3      ; x2 = 3
    instruction_memory[2]  = 32'h00208463;  // beq  x1, x2, 8     ; ��֧��PC+8 (��Ӧ����ת��5!=3)
    instruction_memory[3]  = 32'h00100193;  // addi x3, x0, 1      ; x3 = 1 (Ӧ��ִ��)
    instruction_memory[4]  = 32'h00200213;  // addi x4, x0, 2      ; x4 = 2 (Ӧ��ִ��)
    instruction_memory[5]  = 32'h00c0006f;  // jal  x0, 12         ; ��ת��ָ��8 (PC+12)
    instruction_memory[6]  = 32'h00300293;  // addi x5, x0, 3      ; x5 = 3 (��Ӧ��ִ��)
    instruction_memory[7]  = 32'h00400313;  // addi x6, x0, 4      ; x6 = 4 (��Ӧ��ִ��)
      // ����2: ��֧��תĿ�� (ָ��8-11)
    instruction_memory[8]  = 32'h00500393;  // addi x7, x0, 5      ; x7 = 5 (JAL��ת������)
    instruction_memory[9]  = 32'h00108663;  // beq  x1, x1, 12    ; ��֧��PC+12 (Ӧ����ת��x1==x1=5)
    instruction_memory[10] = 32'h00600413;  // addi x8, x0, 6      ; x8 = 6 (��Ӧ��ִ��)
    instruction_memory[11] = 32'h00700493;  // addi x9, x0, 7      ; x9 = 7 (��Ӧ��ִ��)
    
    // ����3: ��֧Ŀ�� (ָ��12-15)
    instruction_memory[12] = 32'h00800513;  // addi x10, x0, 8     ; x10 = 8 (BEQ��ת������)
    instruction_memory[13] = 32'h00111663;  // bne  x2, x1, 12     ; ��֧��PC+12 (Ӧ����ת��3!=5)
    instruction_memory[14] = 32'h00900593;  // addi x11, x0, 9     ; x11 = 9 (��Ӧ��ִ��)
    instruction_memory[15] = 32'h00a00613;  // addi x12, x0, 10    ; x12 = 10 (��Ӧ��ִ��)
    
    // ����4: ��֧Ŀ�� (ָ��16-19)
    instruction_memory[16] = 32'h00b00693;  // addi x13, x0, 11    ; x13 = 11 (BNE��ת������)
    instruction_memory[17] = 32'h00000013;  // nop                  ; 
    instruction_memory[18] = 32'h00000013;  // nop                  ;
    instruction_memory[19] = 32'h00000013;  // nop                  ;
    
    // ����5: ������صķ�֧ð��
    instruction_memory[20] = 32'h00c00713;  // addi x14, x0, 12    ; x14 = 12
    instruction_memory[21] = 32'h00170713;  // addi x14, x14, 1    ; x14 = 13 (��������)
    instruction_memory[22] = 32'h00d70463;  // beq  x14, x13, 8    ; ��֧�ж� (Ӧ����ת��13==11+2=13����Ҫ���x13��ֵ)
    instruction_memory[23] = 32'h00f00793;  // addi x15, x0, 15    ; x15 = 15 (���ܲ�ִ��)
    instruction_memory[24] = 32'h01000813;  // addi x16, x0, 16    ; x16 = 16 (���ܲ�ִ��)
    instruction_memory[25] = 32'h00000013;  // nop                  ;
    instruction_memory[26] = 32'h01100893;  // addi x17, x0, 17    ; x17 = 17 (��֧Ŀ��)
    
    // ����ָ���ʼ��ΪNOP
    for (integer i = 27; i < 64; i = i + 1) begin
        instruction_memory[i] = 32'h00000013; // nop
    end
    
    // ��ʼ�����ݴ洢��
    for (integer i = 0; i < 32; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // ��ʼ����
    #20 rst = 0;
    $display("Starting control hazard test at time %0t", $time);
    
    // �����㹻������������ָ��ִ�����
    #800;
    
    // �����
    $display("\n=== Test Results ===");
    $display("Register values:");
    $display("x1 = %d (expected: 5)",   cpu_inst.reg_file.registers[1]);
    $display("x2 = %d (expected: 3)",   cpu_inst.reg_file.registers[2]);
    $display("x3 = %d (expected: 1)",   cpu_inst.reg_file.registers[3]);  
    $display("x4 = %d (expected: 2)",   cpu_inst.reg_file.registers[4]);
    $display("x5 = %d (expected: 0)",   cpu_inst.reg_file.registers[5]);  // ��Ӧ��ִ��
    $display("x6 = %d (expected: 0)",   cpu_inst.reg_file.registers[6]);  // ��Ӧ��ִ��
    $display("x7 = %d (expected: 5)",   cpu_inst.reg_file.registers[7]);
    $display("x8 = %d (expected: 0)",   cpu_inst.reg_file.registers[8]);  // ��Ӧ��ִ��
    $display("x9 = %d (expected: 0)",   cpu_inst.reg_file.registers[9]);  // ��Ӧ��ִ��
    $display("x10 = %d (expected: 8)",  cpu_inst.reg_file.registers[10]);
    $display("x11 = %d (expected: 0)",  cpu_inst.reg_file.registers[11]); // ��Ӧ��ִ��
    $display("x12 = %d (expected: 0)",  cpu_inst.reg_file.registers[12]); // ��Ӧ��ִ��
    $display("x13 = %d (expected: 11)", cpu_inst.reg_file.registers[13]);
    $display("x14 = %d (expected: 13)", cpu_inst.reg_file.registers[14]);
    $display("x15 = %d (expected: ?)",  cpu_inst.reg_file.registers[15]); // ȡ���ڷ�֧���
    $display("x17 = %d (expected: ?)",  cpu_inst.reg_file.registers[17]); // ȡ���ڷ�֧���
    
    // ��֤��֧����תָ����ȷִ��
    if (cpu_inst.reg_file.registers[1] == 5 && 
        cpu_inst.reg_file.registers[2] == 3 && 
        cpu_inst.reg_file.registers[3] == 1 &&
        cpu_inst.reg_file.registers[4] == 2 &&
        cpu_inst.reg_file.registers[5] == 0 &&  // JAL������ָ��
        cpu_inst.reg_file.registers[6] == 0 &&  // JAL������ָ��
        cpu_inst.reg_file.registers[7] == 5 &&
        cpu_inst.reg_file.registers[8] == 0 &&  // BEQ������ָ��
        cpu_inst.reg_file.registers[9] == 0 &&  // BEQ������ָ��
        cpu_inst.reg_file.registers[10] == 8 &&
        cpu_inst.reg_file.registers[11] == 0 && // BNE������ָ��
        cpu_inst.reg_file.registers[12] == 0 && // BNE������ָ��
        cpu_inst.reg_file.registers[13] == 11 &&
        cpu_inst.reg_file.registers[14] == 13) begin
        $display("\nCONTROL HAZARD TEST PASSED!");
    end else begin
        $display("\nCONTROL HAZARD TEST FAILED!");
        $display("Some branch/jump instructions did not execute correctly");
    end
    
    $finish;
end

endmodule
