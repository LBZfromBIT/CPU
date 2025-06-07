`timescale 1ns/1ps

// ����ת����Ԫ
module forwarding_unit (
    // EX�׶ε�Դ�Ĵ�����ַ
    input [4:0] ex_rs1,
    input [4:0] ex_rs2,
    
    // MEM�׶ε�Ŀ��Ĵ�����Ϣ
    input [4:0] mem_rd,
    input mem_reg_write,
    
    // WB�׶ε�Ŀ��Ĵ�����Ϣ
    input [4:0] wb_rd,
    input wb_reg_write,
    
    // ת�������ź�
    output reg [1:0] forward_a,    // ALU����A��ת������
    output reg [1:0] forward_b     // ALU����B��ת������
);

// ת�������źű���
parameter NO_FORWARD = 2'b00;      // ��ת����ʹ�üĴ����ļ�����
parameter FORWARD_MEM = 2'b01;     // ��MEM�׶�ת��
parameter FORWARD_WB = 2'b10;      // ��WB�׶�ת��

always @(*) begin
    // ALU����A��ת���߼�
    forward_a = NO_FORWARD;
    if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == ex_rs1)) begin
        forward_a = FORWARD_MEM;  // MEM�׶�ת�����ȼ�����
    end
    else if (wb_reg_write && (wb_rd != 5'b00000) && (wb_rd == ex_rs1)) begin
        forward_a = FORWARD_WB;
    end
    
    // ALU����B��ת���߼�
    forward_b = NO_FORWARD;
    if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == ex_rs2)) begin
        forward_b = FORWARD_MEM;  // MEM�׶�ת�����ȼ�����
    end
    else if (wb_reg_write && (wb_rd != 5'b00000) && (wb_rd == ex_rs2)) begin
        forward_b = FORWARD_WB;
    end
end

endmodule

// ð�ռ�ⵥԪ
module hazard_detection_unit (
    // ID�׶ε�Դ�Ĵ�����ַ
    input [4:0] id_rs1,
    input [4:0] id_rs2,
    
    // EX�׶ε�loadָ����Ϣ
    input [4:0] ex_rd,
    input ex_mem_read,
    
    // ��֧����ת�ź�
    input branch_taken,
    input jump,
    
    // �����ź����
    output reg pc_write,           // PCдʹ��
    output reg if_id_write,        // IF/ID�Ĵ���дʹ��
    output reg control_mux_sel,    // �����ź�ѡ�񣨲���bubble��
    output reg flush_if_id,        // ���IF/ID�Ĵ���
    output reg flush_id_ex,        // ���ID/EX�Ĵ���
    output reg flush_ex_mem        // ���EX/MEM�Ĵ���
);

always @(*) begin
    // Ĭ��ֵ��������ˮ�߲���
    pc_write = 1'b1;
    if_id_write = 1'b1;
    control_mux_sel = 1'b0;
    flush_if_id = 1'b0;
    flush_id_ex = 1'b0;
    flush_ex_mem = 1'b0;
    
    // Load-Use ð�ռ��
    // ��EX�׶���loadָ�����Ŀ��Ĵ�����ID�׶�ָ���Դ�Ĵ���ʱ
    if (ex_mem_read && 
        ((ex_rd == id_rs1) || (ex_rd == id_rs2)) && 
        (ex_rd != 5'b00000)) begin
        
        // ����һ��stall����
        pc_write = 1'b0;           // ��ͣPC����
        if_id_write = 1'b0;        // ��ͣIF/ID�Ĵ���
        control_mux_sel = 1'b1;    // ��ID/EX�в���bubble��NOP��
    end
    
    // ��֧��תð�մ���
    if (branch_taken || jump) begin
        flush_if_id = 1'b1;        // ���IF/ID��ȡ����ȡָ�
        flush_id_ex = 1'b1;        // ���ID/EX��ȡ��������ָ�
    end
end

endmodule

// ��֧ת����Ԫ�����ڷ�ָ֧������ڽ����
module branch_forwarding_unit (
    // ID�׶ε�Դ�Ĵ�����ַ
    input [4:0] id_rs1,
    input [4:0] id_rs2,
    
    // EX�׶εļĴ�����Ϣ
    input [4:0] ex_rd,
    input ex_reg_write,
    input [31:0] ex_alu_result,
    
    // MEM�׶εļĴ�����Ϣ
    input [4:0] mem_rd,
    input mem_reg_write,
    input [31:0] mem_data,
    
    // ԭʼ�Ĵ�������
    input [31:0] id_rs1_data,
    input [31:0] id_rs2_data,
    
    // ת����ļĴ�������
    output reg [31:0] forwarded_rs1_data,
    output reg [31:0] forwarded_rs2_data
);

always @(*) begin
    // RS1ת���߼�
    forwarded_rs1_data = id_rs1_data;  // Ĭ��ʹ��ԭʼ����
    if (ex_reg_write && (ex_rd != 5'b00000) && (ex_rd == id_rs1)) begin
        forwarded_rs1_data = ex_alu_result;  // ��EX�׶�ת��
    end
    else if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == id_rs1)) begin
        forwarded_rs1_data = mem_data;       // ��MEM�׶�ת��
    end
    
    // RS2ת���߼�
    forwarded_rs2_data = id_rs2_data;  // Ĭ��ʹ��ԭʼ����
    if (ex_reg_write && (ex_rd != 5'b00000) && (ex_rd == id_rs2)) begin
        forwarded_rs2_data = ex_alu_result;  // ��EX�׶�ת��
    end
    else if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == id_rs2)) begin
        forwarded_rs2_data = mem_data;       // ��MEM�׶�ת��
    end
end

endmodule

// ����ת����·ѡ����
module forwarding_mux (
    input [31:0] reg_data,         // ���ԼĴ����ļ�������
    input [31:0] mem_forward_data, // ����MEM�׶ε�ת������
    input [31:0] wb_forward_data,  // ����WB�׶ε�ת������
    input [1:0] forward_sel,       // ת��ѡ���ź�
    output reg [31:0] mux_out      // ��·ѡ�������
);

always @(*) begin
    case (forward_sel)
        2'b00: mux_out = reg_data;         // ��ת��
        2'b01: mux_out = mem_forward_data; // MEM�׶�ת��
        2'b10: mux_out = wb_forward_data;  // WB�׶�ת��
        default: mux_out = reg_data;
    endcase
end

endmodule

// ����ð�մ���Ԫ�����ڲ���NOP��
module control_hazard_mux (
    // ���������ź�
    input reg_write_in,
    input mem_read_in,
    input mem_write_in,
    input branch_in,
    input jump_in,
    input alu_src_in,
    input mem_to_reg_in,
    input [1:0] alu_op_in,
    input pc_src_in,
    
    // ����ѡ���ź�
    input control_mux_sel,  // 0: �����ź�, 1: ����NOP
    
    // ��������ź�
    output reg reg_write_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg branch_out,
    output reg jump_out,
    output reg alu_src_out,
    output reg mem_to_reg_out,
    output reg [1:0] alu_op_out,
    output reg pc_src_out
);

always @(*) begin
    if (control_mux_sel) begin
        // ����NOP�����п����ź���0��
        reg_write_out = 1'b0;
        mem_read_out = 1'b0;
        mem_write_out = 1'b0;
        branch_out = 1'b0;
        jump_out = 1'b0;
        alu_src_out = 1'b0;
        mem_to_reg_out = 1'b0;
        alu_op_out = 2'b00;
        pc_src_out = 1'b0;
    end
    else begin
        // �������ݿ����ź�
        reg_write_out = reg_write_in;
        mem_read_out = mem_read_in;
        mem_write_out = mem_write_in;
        branch_out = branch_in;
        jump_out = jump_in;
        alu_src_out = alu_src_in;
        mem_to_reg_out = mem_to_reg_in;
        alu_op_out = alu_op_in;
        pc_src_out = pc_src_in;
    end
end

endmodule