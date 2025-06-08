`timescale 1ns/1ps

module mixed_hazard_testbench();

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
reg [31:0] instruction_memory [0:127];

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
    if (imem_addr[7:2] < 128)
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
            $display("  -> STALL DETECTED! Load-Use hazard causing pipeline stall");
        end
        
        // �������ǰ��
        if (cpu_inst.forward_a != 2'b00 || cpu_inst.forward_b != 2'b00) begin
            $display("  -> DATA FORWARDING:");
            $display("     Forward A: %b, Forward B: %b", cpu_inst.forward_a, cpu_inst.forward_b);
        end
        
        // ��ʾ�ڴ����
        if (cpu_inst.ex_mem_read || cpu_inst.ex_mem_write) begin
            $display("  -> MEMORY OPERATION:");
            if (cpu_inst.ex_mem_read)
                $display("     Load from address: 0x%08h", cpu_inst.ex_alu_result);
            if (cpu_inst.ex_mem_write)
                $display("     Store to address: 0x%08h, data: %d", 
                         cpu_inst.ex_alu_result, cpu_inst.ex_alu_input_b_temp);
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
                3'b001: get_instruction_name = "              SLL";
                3'b010: get_instruction_name = "              SLT";
                3'b011: get_instruction_name = "             SLTU";
                3'b100: get_instruction_name = "              XOR";
                3'b101: get_instruction_name = instruction[30] ? "              SRA" : "              SRL";
                3'b110: get_instruction_name = "               OR";
                3'b111: get_instruction_name = "              AND";
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

// ���Ա�������
integer pass_count;
integer total_tests;
integer i; // forѭ������

// ���ð�ղ��Գ���
initial begin
    
    $display("=== Mixed Hazard Test (Data + Control) ===");
    
    // ��ʼ����λ
    rst = 1;
      // ��ʼ�����ݴ洢��������һЩ��������
    data_memory[0] = 32'h00000010;  // ��ַ0x100: 16
    data_memory[1] = 32'h00000020;  // ��ַ0x104: 32
    data_memory[2] = 32'h00000030;  // ��ַ0x108: 48
    data_memory[3] = 32'h00000040;  // ��ַ0x10C: 64
    for (i = 4; i < 32; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    
    // ��ʼ��ָ��洢�� - ���ð�ղ��Գ���
    
    // === ����1: ������ǰ�� ===
    instruction_memory[0]  = 32'h00500093;  // addi x1, x0, 5      ; x1 = 5
    instruction_memory[1]  = 32'h00108113;  // addi x2, x1, 1      ; x2 = x1 + 1 = 6 (EX-to-EX forwarding)
    instruction_memory[2]  = 32'h002081b3;  // add  x3, x1, x2     ; x3 = x1 + x2 = 11 (MEM-to-EX forwarding)
    
    // === ����2: Load-Useð�գ���Ҫ��ͣ�� ===
    instruction_memory[3]  = 32'h10000213;  // addi x4, x0, 0x100  ; x4 = 0x100 (����ַ)
    instruction_memory[4]  = 32'h00022283;  // lw   x5, 0(x4)      ; x5 = mem[0x100] = 16
    instruction_memory[5]  = 32'h00528313;  // addi x6, x5, 5      ; x6 = x5 + 5 = 21 (Load-Use stall)
    
    // === ����3: ��ָ֧���е�����ð�� ===
    instruction_memory[6]  = 32'h00700393;  // addi x7, x0, 7      ; x7 = 7
    instruction_memory[7]  = 32'h00738413;  // addi x8, x7, 7      ; x8 = x7 + 7 = 14 (ǰ��)
    instruction_memory[8]  = 32'h00740863;  // beq  x8, x7, 16     ; �Ƚ�x8(14) vs x7(7), ����ת (ǰ�ݵ���֧)
    instruction_memory[9]  = 32'h00900493;  // addi x9, x0, 9      ; x9 = 9 (Ӧ��ִ��)
    instruction_memory[10] = 32'h00a00513;  // addi x10, x0, 10    ; x10 = 10 (Ӧ��ִ��)
    
    // === ����4: ��ת�������ð�� ===
    instruction_memory[11] = 32'h0100006f;  // jal  x0, 16         ; ��ת��ָ��15
    instruction_memory[12] = 32'h00b00593;  // addi x11, x0, 11    ; x11 = 11 (��Ӧ��ִ��)
    instruction_memory[13] = 32'h00c00613;  // addi x12, x0, 12    ; x12 = 12 (��Ӧ��ִ��)
    instruction_memory[14] = 32'h00d00693;  // addi x13, x0, 13    ; x13 = 13 (��Ӧ��ִ��)
    
    // ��תĿ�� (ָ��15-18)
    instruction_memory[15] = 32'h00e00713;  // addi x14, x0, 14    ; x14 = 14 (JAL��ת������)
    instruction_memory[16] = 32'h00e70793;  // addi x15, x14, 14   ; x15 = x14 + 14 = 28 (EX-to-EX forwarding)
    instruction_memory[17] = 32'h00f78813;  // addi x16, x15, 15   ; x16 = x15 + 15 = 43 (EX-to-EX forwarding)
    
    // === ����5: Store-Load������֧ ===
    instruction_memory[18] = 32'h10000893;  // addi x17, x0, 0x100 ; x17 = 0x100
    instruction_memory[19] = 32'h00f8a223;  // sw   x15, 4(x17)    ; mem[0x104] = x15 = 28
    instruction_memory[20] = 32'h0048a903;  // lw   x18, 4(x17)    ; x18 = mem[0x104] = 28 (Store-to-Load forwarding)
    instruction_memory[21] = 32'h01c90a63;  // beq  x18, x28, 20   ; �Ƚ�x18 vs x28 (��������ת)
    
    // === ����6: ���ӵ�Load-Use���֧��� ===
    instruction_memory[22] = 32'h0008a983;  // lw   x19, 0(x17)    ; x19 = mem[0x100] = 16
    instruction_memory[23] = 32'h01398a63;  // beq  x19, x19, 20   ; �Լ����Լ��Ƚϣ�Ӧ����ת (Load-Use + Branch)
    instruction_memory[24] = 32'h01400a13;  // addi x20, x0, 20    ; x20 = 20 (��Ӧ��ִ��)
    instruction_memory[25] = 32'h01500a93;  // addi x21, x0, 21    ; x21 = 21 (��Ӧ��ִ��)
    instruction_memory[26] = 32'h01600b13;  // addi x22, x0, 22    ; x22 = 22 (��Ӧ��ִ��)
    
    // ��֧Ŀ�� (ָ��27-30)
    instruction_memory[27] = 32'h01700b93;  // addi x23, x0, 23    ; x23 = 23 (BEQ��ת������)
    instruction_memory[28] = 32'h01330c13;  // addi x24, x6, 19    ; x24 = x6 + 19 = 40 (MEM/WB-to-EX forwarding)
    
    // === ����7: Ƕ�׷�֧���������� ===
    instruction_memory[29] = 32'h01800c93;  // addi x25, x0, 24    ; x25 = 24
    instruction_memory[30] = 32'h018c8d13;  // addi x26, x25, 24   ; x26 = x25 + 24 = 48 (ǰ��)
    instruction_memory[31] = 32'h019d0463;  // beq  x26, x25, 8    ; �Ƚ�x26(48) vs x25(24), ����ת
    instruction_memory[32] = 32'h01ad0e63;  // beq  x26, x26, 28   ; x26���Լ��Ƚϣ�Ӧ����ת
    instruction_memory[33] = 32'h01b00db3;  // add  x27, x0, x27   ; x27 = x27 (��Ӧ��ִ��)
    
    // === ����8: �༶Load-Use��ͣ ===
    instruction_memory[34] = 32'h0088ae03;  // lw   x28, 8(x17)    ; x28 = mem[0x108] = 48
    instruction_memory[35] = 32'h01ce0e33;  // add  x28, x28, x28  ; x28 = x28 + x28 = 96 (Load-Use stall)
    instruction_memory[36] = 32'h01ce0eb3;  // add  x29, x28, x28  ; x29 = x28 + x28 = 192 (��������)
    
    // === ����9: ��֧Ԥ��ʧ�ܺ�Ļָ� ===
    instruction_memory[37] = 32'h01f00f13;  // addi x30, x0, 31    ; x30 = 31
    instruction_memory[38] = 32'h000f0463;  // beq  x30, x0, 8     ; x30(31) vs x0(0), ����ת
    instruction_memory[39] = 32'h01ef0f93;  // addi x31, x30, 30   ; x31 = x30 + 30 = 61 (����ִ��)
    instruction_memory[40] = 32'h01ff0463;  // beq  x30, x31, 8    ; x30(31) vs x31(61), ����ת
    instruction_memory[41] = 32'h00000013;  // nop                  ; 
    
    // === ����10: ������ָ֤�� ===
    instruction_memory[42] = 32'h00100013;  // addi x0, x0, 1      ; nop (x0����д)
    instruction_memory[43] = 32'h00200013;  // addi x0, x0, 2      ; nop 
    instruction_memory[44] = 32'h00300013;  // addi x0, x0, 3      ; nop
    instruction_memory[45] = 32'h00400013;  // addi x0, x0, 4      ; nop    
    // ����ָ���ʼ��ΪNOP
    for (i = 46; i < 128; i = i + 1) begin
        instruction_memory[i] = 32'h00000013; // nop
    end
    
    // ��ʼ����
    #20 rst = 0;
    $display("Starting mixed hazard test at time %0t", $time);
    
    // �����㹻������������ָ��ִ�����
    #1500;
    
    // �����
    $display("\n=== Mixed Hazard Test Results ===");
    $display("Data Forwarding Tests:");
    $display("x1 = %d (expected: 5)",    cpu_inst.reg_file.registers[1]);
    $display("x2 = %d (expected: 6)",    cpu_inst.reg_file.registers[2]);
    $display("x3 = %d (expected: 11)",   cpu_inst.reg_file.registers[3]);
    
    $display("\nLoad-Use Hazard Tests:");
    $display("x4 = %d (expected: 256)",  cpu_inst.reg_file.registers[4]);  // 0x100
    $display("x5 = %d (expected: 16)",   cpu_inst.reg_file.registers[5]);
    $display("x6 = %d (expected: 21)",   cpu_inst.reg_file.registers[6]);
    
    $display("\nBranch with Data Hazard Tests:");
    $display("x7 = %d (expected: 7)",    cpu_inst.reg_file.registers[7]);
    $display("x8 = %d (expected: 14)",   cpu_inst.reg_file.registers[8]);
    $display("x9 = %d (expected: 9)",    cpu_inst.reg_file.registers[9]);   // Ӧ��ִ��
    $display("x10 = %d (expected: 10)",  cpu_inst.reg_file.registers[10]);  // Ӧ��ִ��
    
    $display("\nJump with Control Hazard Tests:");
    $display("x11 = %d (expected: 0)",   cpu_inst.reg_file.registers[11]);  // ��Ӧ��ִ��
    $display("x12 = %d (expected: 0)",   cpu_inst.reg_file.registers[12]);  // ��Ӧ��ִ��
    $display("x13 = %d (expected: 0)",   cpu_inst.reg_file.registers[13]);  // ��Ӧ��ִ��
    $display("x14 = %d (expected: 14)",  cpu_inst.reg_file.registers[14]);
    $display("x15 = %d (expected: 28)",  cpu_inst.reg_file.registers[15]);
    $display("x16 = %d (expected: 43)",  cpu_inst.reg_file.registers[16]);
    
    $display("\nMemory Operation Tests:");
    $display("x17 = %d (expected: 256)", cpu_inst.reg_file.registers[17]); // 0x100
    $display("x18 = %d (expected: 28)",  cpu_inst.reg_file.registers[18]);
    $display("x19 = %d (expected: 16)",  cpu_inst.reg_file.registers[19]);
    
    $display("\nComplex Hazard Tests:");
    $display("x20 = %d (expected: 0)",   cpu_inst.reg_file.registers[20]);  // ��Ӧ��ִ��
    $display("x21 = %d (expected: 0)",   cpu_inst.reg_file.registers[21]);  // ��Ӧ��ִ��
    $display("x22 = %d (expected: 0)",   cpu_inst.reg_file.registers[22]);  // ��Ӧ��ִ��
    $display("x23 = %d (expected: 23)",  cpu_inst.reg_file.registers[23]);
    $display("x24 = %d (expected: 40)",  cpu_inst.reg_file.registers[24]);
    $display("x25 = %d (expected: 24)",  cpu_inst.reg_file.registers[25]);
    $display("x26 = %d (expected: 48)",  cpu_inst.reg_file.registers[26]);
    $display("x28 = %d (expected: 96)",  cpu_inst.reg_file.registers[28]);
    $display("x29 = %d (expected: 192)", cpu_inst.reg_file.registers[29]);
    $display("x30 = %d (expected: 31)",  cpu_inst.reg_file.registers[30]);    $display("x31 = %d (expected: 61)",  cpu_inst.reg_file.registers[31]);
      // ��֤���Խ��
    pass_count = 0;
    total_tests = 27;  // �����ܲ�������
    
    // ��������ǰ�ݲ���
    if (cpu_inst.reg_file.registers[1] == 5) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[2] == 6) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[3] == 11) pass_count = pass_count + 1;
    
    // Load-Useð�ղ���
    if (cpu_inst.reg_file.registers[4] == 256) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[5] == 16) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[6] == 21) pass_count = pass_count + 1;
    
    // ��֧������ð�ղ���
    if (cpu_inst.reg_file.registers[7] == 7) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[8] == 14) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[9] == 9) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[10] == 10) pass_count = pass_count + 1;
    
    // ��ת����ð�ղ���
    if (cpu_inst.reg_file.registers[11] == 0) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[12] == 0) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[13] == 0) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[14] == 14) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[15] == 28) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[16] == 43) pass_count = pass_count + 1;
    
    // �ڴ��������
    if (cpu_inst.reg_file.registers[17] == 256) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[18] == 28) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[19] == 16) pass_count = pass_count + 1;
    
    // ����ð�ղ���
    if (cpu_inst.reg_file.registers[20] == 0) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[21] == 0) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[22] == 0) pass_count = pass_count + 1;  // �ؼ������Ӧ����0
    if (cpu_inst.reg_file.registers[23] == 23) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[24] == 40) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[25] == 24) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[26] == 48) pass_count = pass_count + 1;
    if (cpu_inst.reg_file.registers[28] == 96) pass_count = pass_count + 1;  // �ؼ������Ӧ����96
    if (cpu_inst.reg_file.registers[29] == 192) pass_count = pass_count + 1; // �ؼ������Ӧ����192
    if (cpu_inst.reg_file.registers[30] == 31) pass_count = pass_count + 1;  // �ؼ������Ӧ����31
    if (cpu_inst.reg_file.registers[31] == 61) pass_count = pass_count + 1;  // �ؼ������Ӧ����61
    
    $display("\n=== Test Summary ===");
    $display("Passed: %d/%d tests", pass_count, total_tests);
    
    if (pass_count == total_tests) begin
        $display("MIXED HAZARD TEST PASSED!");
        $display("All data forwarding, stalls, and control hazards handled correctly!");
    end else begin
        $display("MIXED HAZARD TEST FAILED!");
        $display("Some hazards were not handled properly");
    end
    
    $finish;
end

endmodule
