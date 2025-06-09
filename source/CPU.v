`timescale 1ns/1ps

module risc_v_cpu (
    input clk,                    // ϵͳʱ�ӣ���������ʱ���߼�
    input rst,                    // ��λ�źţ�ϵͳ����ʱ��ʼ������״̬
    
    // === ָ��洢���ӿ� ===
    output [31:0] imem_addr,      // ���͸�ָ��洢���ĵ�ַ��PCֵ��
    input [31:0] imem_data,       // ��ָ��洢����ȡ��32λָ��
    
    // === ���ݴ洢���ӿ� ===
    output [31:0] dmem_addr,      // ���͸����ݴ洢���ĵ�ַ��Load/Store��ַ��
    output [31:0] dmem_wdata,     // д�����ݴ洢�������ݣ�Store���ݣ�
    output dmem_we,               // ���ݴ洢��дʹ�ܣ�1=д�룬0=��ȡ��
    input [31:0] dmem_rdata,      // �����ݴ洢����ȡ�����ݣ�Load���ݣ�

    // === ���Խӿ� ===
    output [31:0] debug_pc,       // ��ǰPCֵ�����ڵ��Ժͷ���
    output [31:0] debug_instruction // ��ǰָ����ڵ��Ժͷ���
);

// ============================================================================
// === �ź�����
// ============================================================================

// === PC�����ź� ===
wire [31:0] pc_current;           // ��ǰPCֵ��ָ��ǰ����ȡָ�ĵ�ַ
wire [31:0] pc_next;              // ��һ��PCֵ���¸�����PC�����µ���ֵ
wire [31:0] pc_plus4;             // PC+4��˳��ִ��ʱ����һ��ָ���ַ
wire pc_write;                    // PCдʹ�ܣ�����ð�ռ�⣬����PC�Ƿ����

// === IF�׶��ź� ===
wire [31:0] if_pc;                // IF�׶ε�PCֵ�����ݸ�IF/ID�Ĵ���
wire [31:0] if_instruction;       // IF�׶�ȡ����ָ���ָ��洢����ȡ

// === ID�׶��ź� ===
// ����IF/ID�Ĵ���
wire [31:0] id_pc;                // ID�׶ε�PCֵ�����ڼ����֧/��תĿ��
wire [31:0] id_instruction;       // ID�׶ε�ָ���Ҫ�����32λָ��

// ָ���ֶν���
wire [4:0] id_rs1_addr;           // Դ�Ĵ���1��ַ��instruction[19:15]
wire [4:0] id_rs2_addr;           // Դ�Ĵ���2��ַ��instruction[24:20]
wire [4:0] id_rd_addr;            // Ŀ��Ĵ�����ַ��instruction[11:7]
wire [2:0] id_funct3;             // �����ֶ�3��instruction[14:12]
wire [6:0] id_funct7;             // �����ֶ�7��instruction[31:25]
wire [6:0] id_opcode;             // �����룺instruction[6:0]

// �Ĵ����ļ���������
wire [31:0] id_rs1_data;          // Դ�Ĵ���1�����ݣ�reg_file[rs1]��ֵ
wire [31:0] id_rs2_data;          // Դ�Ĵ���2�����ݣ�reg_file[rs2]��ֵ
wire [31:0] id_immediate;         // ��ָ���н�������������������չ���ֵ

// �����źţ�ԭʼ��
wire id_reg_write;                // �Ĵ���дʹ�ܣ���ָ���Ƿ�д�ؼĴ���
wire id_mem_read;                 // �ڴ��ʹ�ܣ��Ƿ�ΪLoadָ��
wire id_mem_write;                // �ڴ�дʹ�ܣ��Ƿ�ΪStoreָ��
wire id_branch;                   // ��֧��־���Ƿ�Ϊ��ָ֧��
wire id_jump;                     // ��ת��־���Ƿ�Ϊ��תָ��
wire id_alu_src;                  // ALUԴѡ��ALU�ڶ���������ѡ��
wire id_mem_to_reg;               // д��ѡ��д��������Դѡ��
wire id_pc_src;                   // PCԴѡ��PC���·�ʽѡ��
wire [1:0] id_alu_op;             // ALU�������ͣ����ݸ�ALU���Ƶ�Ԫ

// �����źţ�����ð�մ���
wire ctrl_reg_write;              // �����ļĴ���дʹ��
wire ctrl_mem_read;               // �������ڴ��ʹ��
wire ctrl_mem_write;              // �������ڴ�дʹ��
wire ctrl_branch;                 // �����ķ�֧��־
wire ctrl_jump;                   // ��������ת��־
wire ctrl_alu_src;                // ������ALUԴѡ��
wire ctrl_mem_to_reg;             // ������д��ѡ��
wire ctrl_pc_src;                 // ������PCԴѡ��
wire [1:0] ctrl_alu_op;           // ������ALU��������

// ��֧ת���ź�
wire [31:0] forwarded_rs1_data;   // ת�����rs1���ݣ����ڷ�֧�ж�
wire [31:0] forwarded_rs2_data;   // ת�����rs2���ݣ����ڷ�֧�ж�

// === EX�׶��ź� ===
// ����ID/EX�Ĵ���
wire [31:0] ex_pc;                // EX�׶ε�PCֵ�����ڼ����֧/��תĿ��
wire [31:0] ex_rs1_data;          // EX�׶ε�rs1���ݣ����ܱ�ת���޸�
wire [31:0] ex_rs2_data;          // EX�׶ε�rs2���ݣ����ܱ�ת���޸�
wire [31:0] ex_immediate;         // EX�׶ε�������������ALU����

// �Ĵ�����ַ������ת����⣩
wire [4:0] ex_rs1_addr;           // EX�׶ε�rs1��ַ��ת����Ԫ��Ҫ
wire [4:0] ex_rs2_addr;           // EX�׶ε�rs2��ַ��ת����Ԫ��Ҫ
wire [4:0] ex_rd_addr;            // EX�׶ε�rd��ַ��ת����Ԫ��Ҫ

// ָ���ֶΣ�����ALU���ƣ�
wire [2:0] ex_funct3;             // EX�׶ε�funct3��ALU�������
wire [6:0] ex_funct7;             // EX�׶ε�funct7��ALU�������
wire [6:0] ex_opcode;             // EX�׶ε�opcode����ת��ַ����

// �����ź�
wire ex_reg_write, ex_mem_read, ex_mem_write, ex_branch, ex_jump;
wire ex_alu_src, ex_mem_to_reg, ex_pc_src;
wire [1:0] ex_alu_op;

// ALU����ź�
wire [31:0] ex_alu_input_a;       // ALU����A������ת�����������
wire [31:0] ex_alu_input_b;       // ALU����B����������ת������
wire [31:0] ex_alu_input_b_temp;  // ALU����B��ʱ�źţ�ת��ǰ������
wire [31:0] ex_alu_result;        // ALU�����������
wire [3:0] alu_control;           // ALU�����źţ������ALU������

// ��֧��ת�ź�
wire branch_taken;                // ��֧�Ƿ��������֧�����жϽ��
wire [31:0] branch_target;        // ��֧Ŀ���ַ��PC + branch_offset
wire [31:0] jump_target;          // ��תĿ���ַ��PC + jump_offset

// === MEM�׶��ź� ===
// ����EX/MEM�Ĵ���
wire [31:0] mem_pc;               // MEM�׶ε�PCֵ������JALд��
wire [31:0] mem_alu_result;       // MEM�׶ε�ALU������ڴ��ַ��������
wire [31:0] mem_rs2_data;         // MEM�׶ε�rs2���ݣ�Storeָ���д������
wire [31:0] mem_branch_target;    // MEM�׶εķ�֧Ŀ�꣺����PC����
wire [31:0] mem_jump_target;      // MEM�׶ε���תĿ�꣺����PC����

// ���ƺ�״̬�ź�
wire [4:0] mem_rd_addr;           // MEM�׶ε�rd��ַ��д�ص�ַ
wire [2:0] mem_funct3;            // MEM�׶ε�funct3��Load/Store��ȿ���
wire mem_reg_write, mem_mem_read, mem_mem_write, mem_mem_to_reg;
wire mem_branch_taken;            // MEM�׶εķ�֧���������PC����
wire mem_jump;                    // MEM�׶ε���ת�źţ�����PC����
wire mem_jump_for_wb;             // MEM�׶ε���תд�ر�־������PC+4д��

// �ڴ�����ź�
wire [31:0] mem_data_out;         // �����ݴ洢����ȡ������

// === WB�׶��ź� ===
// ����MEM/WB�Ĵ���
wire [31:0] wb_pc;                // WB�׶ε�PCֵ�����ڼ���PC+4
wire [31:0] wb_alu_result;        // WB�׶ε�ALU���������������
wire [31:0] wb_mem_data;          // WB�׶ε��ڴ����ݣ�Loadָ������
wire [31:0] wb_write_data;        // ����д�����ݣ�����ѡ���д��ֵ
wire [31:0] wb_pc_plus4;          // WB�׶ε�PC+4��JALָ��д��ֵ

// д�ؿ���
wire [4:0] wb_rd_addr;            // д�ؼĴ�����ַ��Ŀ��Ĵ���
wire wb_reg_write;                // д��ʹ�ܣ��Ƿ�ִ��д��
wire wb_mem_to_reg;               // ����Դѡ��ALU��� vs �ڴ�����
wire wb_jump_for_wb;              // ��תд�ر�־���Ƿ�д��PC+4

// === ð�ռ���ת���ź� ===
// ����ת������
wire [1:0] forward_a;             // ALU����A��ת��ѡ��00=�Ĵ���,01=MEM,10=WB
wire [1:0] forward_b;             // ALU����B��ת��ѡ��ͬ��

// ��ˮ�߿����ź�
wire if_id_write;                 // IF/ID�Ĵ���дʹ�ܣ�0=��ͣ
wire control_mux_sel;             // �����ź�ѡ��1=����NOP
wire flush_if_id;                 // ���IF/ID�Ĵ�������֧Ԥ�����
wire flush_id_ex;                 // ���ID/EX�Ĵ�������֧Ԥ�����
wire flush_ex_mem;                // ���EX/MEM�Ĵ������쳣����

// ============================================================================
// === IF�׶� (ȡָ�׶�) 
// ============================================================================

// PC�����߼�
assign pc_plus4 = pc_current + 4;

// PCѡ���߼���������һ��PC����Դ
assign pc_next = (branch_taken && ex_branch) ? branch_target :  // ��֧��ת (EX�׶�)
                 (ex_jump) ? jump_target :                       // ��������ת (EX�׶�)
                 pc_plus4;                                       // ˳��ִ��

// PC�Ĵ���
pc pc_reg (
    .clk(clk),
    .rst(rst),
    .pc_write(pc_write),      // ð�ռ�����PC�Ƿ����
    .pc_next(pc_next),
    .pc_out(pc_current)
);

// ָ��洢���ӿ�����
assign imem_addr = pc_current;    // PC��Ϊָ��洢����ַ
assign if_pc = pc_current;        // ���ݸ�IF/ID�Ĵ���
assign if_instruction = imem_data; // ��ָ��洢����ȡָ��

// ============================================================================
// === IF/ID��ˮ�߼Ĵ���
// ============================================================================

if_id_register if_id_reg (
    .clk(clk),
    .rst(rst),
    .stall(~if_id_write),         // ð�ռ������Ƿ���ͣ
    .flush(flush_if_id),          // ��֧/��תʱ���
    .pc_in(if_pc),
    .instruction_in(if_instruction),
    .pc_out(id_pc),
    .instruction_out(id_instruction)
);

// ============================================================================
// === ID�׶� (����׶�) 
// ============================================================================

// ָ���ֶν���
assign id_opcode = id_instruction[6:0];     // �������ֶ�
assign id_funct3 = id_instruction[14:12];   // �����ֶ�3
assign id_funct7 = id_instruction[31:25];   // �����ֶ�7
assign id_rs1_addr = id_instruction[19:15]; // Դ�Ĵ���1��ַ
assign id_rs2_addr = id_instruction[24:20]; // Դ�Ĵ���2��ַ
assign id_rd_addr = id_instruction[11:7];   // Ŀ��Ĵ�����ַ

// �Ĵ����ļ�
register_file reg_file (
    .clk(clk),
    .rst(rst),
    .reg_write(wb_reg_write),     // д��������WB�׶�
    .rs1(id_rs1_addr),            // ���˿�1��ַ
    .rs2(id_rs2_addr),            // ���˿�2��ַ
    .rd(wb_rd_addr),              // д�˿ڵ�ַ����WB�׶�
    .write_data(wb_write_data),   // д��������WB�׶�
    .read_data1(id_rs1_data),     // ���˿�1����
    .read_data2(id_rs2_data)      // ���˿�2����
);

// ������������
immediate_generator imm_gen (
    .instruction(id_instruction),
    .immediate(id_immediate)      // ���ɷ�����չ��������
);

// �����Ƶ�Ԫ
control_unit ctrl_unit (
    .opcode(id_opcode),
    .funct3(id_funct3),
    .funct7(id_funct7),
    // �����ź����
    .reg_write(id_reg_write),
    .mem_read(id_mem_read),
    .mem_write(id_mem_write),
    .branch(id_branch),
    .jump(id_jump),
    .alu_src(id_alu_src),
    .mem_to_reg(id_mem_to_reg),
    .alu_op(id_alu_op),
    .pc_src(id_pc_src)
);

// ����ð�մ�������Load-Useð��ʱ����NOP��
control_hazard_mux ctrl_hazard_mux (
    // ���룺ԭʼ�����ź�
    .reg_write_in(id_reg_write),
    .mem_read_in(id_mem_read),
    .mem_write_in(id_mem_write),
    .branch_in(id_branch),
    .jump_in(id_jump),
    .alu_src_in(id_alu_src),
    .mem_to_reg_in(id_mem_to_reg),
    .alu_op_in(id_alu_op),
    .pc_src_in(id_pc_src),
    // �����ź�
    .control_mux_sel(control_mux_sel), // 1=����NOP��0=��������
    // ����������Ŀ����ź�
    .reg_write_out(ctrl_reg_write),
    .mem_read_out(ctrl_mem_read),
    .mem_write_out(ctrl_mem_write),
    .branch_out(ctrl_branch),
    .jump_out(ctrl_jump),
    .alu_src_out(ctrl_alu_src),
    .mem_to_reg_out(ctrl_mem_to_reg),
    .alu_op_out(ctrl_alu_op),
    .pc_src_out(ctrl_pc_src)
);

// ��֧ת����Ԫ�������ָ֧�������ת����
branch_forwarding_unit branch_forward (
    .id_rs1(id_rs1_addr),
    .id_rs2(id_rs2_addr),
    .ex_rd(ex_rd_addr),
    .ex_reg_write(ex_reg_write),
    .ex_alu_result(ex_alu_result),
    .mem_rd(mem_rd_addr),
    .mem_reg_write(mem_reg_write),
    .mem_data(mem_alu_result),
    .id_rs1_data(id_rs1_data),
    .id_rs2_data(id_rs2_data),
    .forwarded_rs1_data(forwarded_rs1_data),
    .forwarded_rs2_data(forwarded_rs2_data)
);

// ============================================================================
// === ID/EX��ˮ�߼Ĵ���
// ============================================================================

id_ex_register id_ex_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 
    .flush(flush_id_ex),          // ��֧/��תʱ���
    
    // �����ź�����
    .reg_write_in(ctrl_reg_write),
    .mem_read_in(ctrl_mem_read),
    .mem_write_in(ctrl_mem_write),
    .branch_in(ctrl_branch),
    .jump_in(ctrl_jump),
    .alu_src_in(ctrl_alu_src),
    .mem_to_reg_in(ctrl_mem_to_reg),
    .alu_op_in(ctrl_alu_op),
    .pc_src_in(ctrl_pc_src),
    
    // �����ź�����
    .pc_in(id_pc),
    .rs1_data_in(id_rs1_data),
    .rs2_data_in(id_rs2_data),
    .immediate_in(id_immediate),
    .rs1_addr_in(id_rs1_addr),
    .rs2_addr_in(id_rs2_addr),
    .rd_addr_in(id_rd_addr),
    .funct3_in(id_funct3),
    .funct7_in(id_funct7),
    .opcode_in(id_opcode),
    
    // �����ź����
    .reg_write_out(ex_reg_write),
    .mem_read_out(ex_mem_read),
    .mem_write_out(ex_mem_write),
    .branch_out(ex_branch),
    .jump_out(ex_jump),
    .alu_src_out(ex_alu_src),
    .mem_to_reg_out(ex_mem_to_reg),
    .alu_op_out(ex_alu_op),
    .pc_src_out(ex_pc_src),
    
    // �����ź����
    .pc_out(ex_pc),
    .rs1_data_out(ex_rs1_data),
    .rs2_data_out(ex_rs2_data),
    .immediate_out(ex_immediate),
    .rs1_addr_out(ex_rs1_addr),
    .rs2_addr_out(ex_rs2_addr),
    .rd_addr_out(ex_rd_addr),
    .funct3_out(ex_funct3),
    .funct7_out(ex_funct7),
    .opcode_out(ex_opcode)
);

// ============================================================================
// === EX�׶� (ִ�н׶�)
// ============================================================================

// ����ת����Ԫ
forwarding_unit forward_unit (
    .ex_rs1(ex_rs1_addr),         // EX�׶ε�Դ�Ĵ���1
    .ex_rs2(ex_rs2_addr),         // EX�׶ε�Դ�Ĵ���2
    .mem_rd(mem_rd_addr),         // MEM�׶ε�Ŀ��Ĵ���
    .mem_reg_write(mem_reg_write), // MEM�׶��Ƿ�д��
    .wb_rd(wb_rd_addr),           // WB�׶ε�Ŀ��Ĵ���
    .wb_reg_write(wb_reg_write),  // WB�׶��Ƿ�д��
    .forward_a(forward_a),        // ALU����A��ת������
    .forward_b(forward_b)         // ALU����B��ת������
);

// ALU����A��ת����·ѡ����
forwarding_mux alu_input_a_mux (
    .reg_data(ex_rs1_data),       // ���ԼĴ�����ԭʼ����
    .mem_forward_data(mem_alu_result), // ��MEM�׶�ת��������
    .wb_forward_data(wb_write_data),   // ��WB�׶�ת��������
    .forward_sel(forward_a),      // ת��ѡ���ź�
    .mux_out(ex_alu_input_a)      // ���յ�ALU����A
);

// ALU����B��ת����·ѡ����
forwarding_mux alu_input_b_mux (
    .reg_data(ex_rs2_data),       // ���ԼĴ�����ԭʼ����
    .mem_forward_data(mem_alu_result), // ��MEM�׶�ת��������
    .wb_forward_data(wb_write_data),   // ��WB�׶�ת��������
    .forward_sel(forward_b),      // ת��ѡ���ź�
    .mux_out(ex_alu_input_b_temp) // ת�����rs2����
);

// ALUԴ����ѡ�������Ĵ������� vs ��������
assign ex_alu_input_b = ex_alu_src ? ex_immediate : ex_alu_input_b_temp;

// ALU���Ƶ�Ԫ
alu_control alu_ctrl (
    .alu_op(ex_alu_op),           // ���Կ��Ƶ�Ԫ�Ĳ�������
    .funct3(ex_funct3),           // ָ���funct3�ֶ�
    .funct7(ex_funct7),           // ָ���funct7�ֶ�
    .alu_control_out(alu_control) // ���ɾ����ALU�����ź�
);

// ALU (�����߼���Ԫ)
alu main_alu (
    .a(ex_alu_input_a),           // ALU����A
    .b(ex_alu_input_b),           // ALU����B
    .alu_control(alu_control),    // ALU�����ź�
    .result(ex_alu_result)        // ALU������
);

// ��֧���Ƶ�Ԫ
branch_control branch_ctrl (
    .branch(ex_branch),           // �Ƿ�Ϊ��ָ֧��
    .funct3(ex_funct3),           // ��֧���� (BEQ, BNE, BLT��)
    .rs1_data(ex_alu_input_a),    // �Ƚ�����1 (ת����)
    .rs2_data(ex_alu_input_b_temp), // �Ƚ�����2 (ת�����rs2������������)
    .branch_taken(branch_taken)   // ��֧�Ƿ����
);

// ��֧����תĿ���ַ����
assign branch_target = ex_pc + ex_immediate; // ��֧Ŀ���ַ
assign jump_target = ex_pc + ex_immediate;   // ��תĿ���ַ

// ============================================================================
// === EX/MEM��ˮ�߼Ĵ���
// ============================================================================

ex_mem_register ex_mem_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 
    .flush(flush_ex_mem),         // �쳣ʱ���
    
    // �����ź�����
    .reg_write_in(ex_reg_write),
    .mem_read_in(ex_mem_read),
    .mem_write_in(ex_mem_write),
    .mem_to_reg_in(ex_mem_to_reg),
    .branch_taken_in(branch_taken),
    .jump_in(ex_jump),
    .jump_for_wb_in(ex_jump),     // JALָ����Ҫд��PC+4
    
    // �����ź�����
    .pc_in(ex_pc),
    .alu_result_in(ex_alu_result),
    .rs2_data_in(ex_alu_input_b_temp), // Storeָ���д����
    .branch_target_in(branch_target),
    .jump_target_in(jump_target),
    .rd_addr_in(ex_rd_addr),
    .funct3_in(ex_funct3),
    
    // �����ź����
    .reg_write_out(mem_reg_write),
    .mem_read_out(mem_mem_read),
    .mem_write_out(mem_mem_write),
    .mem_to_reg_out(mem_mem_to_reg),
    .branch_taken_out(mem_branch_taken),
    .jump_out(mem_jump),
    .jump_for_wb_out(mem_jump_for_wb),
    
    // �����ź����
    .pc_out(mem_pc),
    .alu_result_out(mem_alu_result),
    .rs2_data_out(mem_rs2_data),
    .branch_target_out(mem_branch_target),
    .jump_target_out(mem_jump_target),
    .rd_addr_out(mem_rd_addr),
    .funct3_out(mem_funct3)
);

// ============================================================================
// === MEM�׶� (�ô�׶�) 
// ============================================================================

// ���ݴ洢���ӿ�����
assign dmem_addr = mem_alu_result;    // �ڴ��ַ����ALU������
assign dmem_wdata = mem_rs2_data;     // д��������rs2�Ĵ���
assign dmem_we = mem_mem_write;       // дʹ�����Կ����ź�
assign mem_data_out = dmem_rdata;     // ���������Դ洢��

// ============================================================================
// === MEM/WB��ˮ�߼Ĵ���
// ============================================================================

mem_wb_register mem_wb_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 // WB�׶β���Ҫ��ͣ
    .flush(1'b0),                 // WB�׶β���Ҫ���
    
    // �����ź�����
    .reg_write_in(mem_reg_write),
    .mem_to_reg_in(mem_mem_to_reg),
    .jump_for_wb_in(mem_jump_for_wb),
    
    // �����ź�����
    .pc_in(mem_pc),
    .alu_result_in(mem_alu_result),
    .mem_data_in(mem_data_out),
    .rd_addr_in(mem_rd_addr),
    
    // �����ź����
    .reg_write_out(wb_reg_write),
    .mem_to_reg_out(wb_mem_to_reg),
    .jump_for_wb_out(wb_jump_for_wb),
    
    // �����ź����
    .pc_out(wb_pc),
    .alu_result_out(wb_alu_result),
    .mem_data_out(wb_mem_data),
    .rd_addr_out(wb_rd_addr)
);

// ============================================================================
// === WB�׶� (д�ؽ׶�)
// ============================================================================

// ����PC+4������JALָ��д�أ�
assign wb_pc_plus4 = wb_pc + 4;

// д������ѡ����
assign wb_write_data = wb_jump_for_wb ? wb_pc_plus4 :      // JALָ�д��PC+4
                       wb_mem_to_reg ? wb_mem_data :       // Loadָ�д���ڴ�����
                       wb_alu_result;                      // ����ָ�д��ALU���

// ============================================================================
// === ð�ռ��ʹ���Ԫ
// ============================================================================

hazard_detection_unit hazard_detect (
    // ���룺��ǰָ���Դ�Ĵ���
    .id_rs1(id_rs1_addr),
    .id_rs2(id_rs2_addr),
    
    // ���룺EX�׶ε�Loadָ����Ϣ
    .ex_rd(ex_rd_addr),
    .ex_mem_read(ex_mem_read),
    
    // ���룺��֧��ת�źţ�����EX�׶Σ���ǰ��⣩
    .branch_taken(branch_taken),
    .jump(ex_jump),
    
    // �������ˮ�߿����ź�
    .pc_write(pc_write),          // ����PC�Ƿ����
    .if_id_write(if_id_write),    // ����IF/ID�Ĵ����Ƿ����
    .control_mux_sel(control_mux_sel), // �����Ƿ����NOP
    .flush_if_id(flush_if_id),    // ���IF/ID�Ĵ���
    .flush_id_ex(flush_id_ex),    // ���ID/EX�Ĵ���
    .flush_ex_mem(flush_ex_mem)   // ���EX/MEM�Ĵ���
);

// ============================================================================
// === ���Խӿ�
// ============================================================================

assign debug_pc = pc_current;        // �����ǰPC���ڵ���
assign debug_instruction = if_instruction; // �����ǰָ�����ڵ���

endmodule