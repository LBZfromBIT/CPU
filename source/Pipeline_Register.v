`timescale 1ns/1ps

// IF/ID ��ˮ�߼Ĵ���
module if_id_register (
    input clk,
    input rst,
    input stall,                    // ��ˮ����ͣ�ź�
    input flush,                    // ��ˮ������ź�
    
    // �����ź�
    input [31:0] pc_in,
    input [31:0] instruction_in,
    
    // ����ź�
    output reg [31:0] pc_out,
    output reg [31:0] instruction_out
);

always @(posedge clk or posedge rst) begin
    if (rst || flush) begin
        pc_out <= 32'h00000000;
        instruction_out <= 32'h00000000;  // NOPָ��
    end
    else if (!stall) begin
        pc_out <= pc_in;
        instruction_out <= instruction_in;
    end
    // ���stallΪ1�����ֵ�ǰֵ����
end

endmodule

// ID/EX ��ˮ�߼Ĵ���
module id_ex_register (
    input clk,
    input rst,
    input stall,
    input flush,
    
    // �����ź�����
    input reg_write_in,
    input mem_read_in,
    input mem_write_in,
    input branch_in,
    input jump_in,
    input alu_src_in,
    input mem_to_reg_in,
    input [1:0] alu_op_in,
    input pc_src_in,
    
    // �����ź�����
    input [31:0] pc_in,
    input [31:0] rs1_data_in,
    input [31:0] rs2_data_in,
    input [31:0] immediate_in,
    input [4:0] rs1_addr_in,
    input [4:0] rs2_addr_in,
    input [4:0] rd_addr_in,
    input [2:0] funct3_in,
    input [6:0] funct7_in,
    input [6:0] opcode_in,
    
    // �����ź����
    output reg reg_write_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg branch_out,
    output reg jump_out,
    output reg alu_src_out,
    output reg mem_to_reg_out,
    output reg [1:0] alu_op_out,
    output reg pc_src_out,
    
    // �����ź����
    output reg [31:0] pc_out,
    output reg [31:0] rs1_data_out,
    output reg [31:0] rs2_data_out,
    output reg [31:0] immediate_out,
    output reg [4:0] rs1_addr_out,
    output reg [4:0] rs2_addr_out,
    output reg [4:0] rd_addr_out,
    output reg [2:0] funct3_out,
    output reg [6:0] funct7_out,
    output reg [6:0] opcode_out
);

always @(posedge clk or posedge rst) begin
    if (rst || flush) begin
        // �����ź�����
        reg_write_out <= 1'b0;
        mem_read_out <= 1'b0;
        mem_write_out <= 1'b0;
        branch_out <= 1'b0;
        jump_out <= 1'b0;
        alu_src_out <= 1'b0;
        mem_to_reg_out <= 1'b0;
        alu_op_out <= 2'b00;
        pc_src_out <= 1'b0;
        
        // �����ź�����
        pc_out <= 32'h00000000;
        rs1_data_out <= 32'h00000000;
        rs2_data_out <= 32'h00000000;
        immediate_out <= 32'h00000000;
        rs1_addr_out <= 5'b00000;
        rs2_addr_out <= 5'b00000;
        rd_addr_out <= 5'b00000;
        funct3_out <= 3'b000;
        funct7_out <= 7'b0000000;
        opcode_out <= 7'b0000000;
    end
    else if (!stall) begin
        // �����źŴ���
        reg_write_out <= reg_write_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        branch_out <= branch_in;
        jump_out <= jump_in;
        alu_src_out <= alu_src_in;
        mem_to_reg_out <= mem_to_reg_in;
        alu_op_out <= alu_op_in;
        pc_src_out <= pc_src_in;
        
        // �����źŴ���
        pc_out <= pc_in;
        rs1_data_out <= rs1_data_in;
        rs2_data_out <= rs2_data_in;
        immediate_out <= immediate_in;
        rs1_addr_out <= rs1_addr_in;
        rs2_addr_out <= rs2_addr_in;
        rd_addr_out <= rd_addr_in;
        funct3_out <= funct3_in;
        funct7_out <= funct7_in;
        opcode_out <= opcode_in;
    end
end

endmodule

// EX/MEM ��ˮ�߼Ĵ���
module ex_mem_register (
    input clk,
    input rst,
    input stall,
    input flush,
    
    // �����ź�����
    input reg_write_in,
    input mem_read_in,
    input mem_write_in,
    input mem_to_reg_in,
    input branch_taken_in,
    input jump_in,
    input jump_for_wb_in,
    
    // �����ź�����
    input [31:0] pc_in,
    input [31:0] alu_result_in,
    input [31:0] rs2_data_in,      // ����storeָ��
    input [31:0] branch_target_in,
    input [31:0] jump_target_in,
    input [4:0] rd_addr_in,
    input [2:0] funct3_in,         // ����load/storeָ����ֽ�/���ֿ���
    
    // �����ź����
    output reg reg_write_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg mem_to_reg_out,
    output reg branch_taken_out,
    output reg jump_out,
    output reg jump_for_wb_out,
    
    // �����ź����
    output reg [31:0] pc_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] rs2_data_out,
    output reg [31:0] branch_target_out,
    output reg [31:0] jump_target_out,
    output reg [4:0] rd_addr_out,
    output reg [2:0] funct3_out
);

always @(posedge clk or posedge rst) begin
    if (rst || flush) begin
        // �����ź�����
        reg_write_out <= 1'b0;
        mem_read_out <= 1'b0;
        mem_write_out <= 1'b0;
        mem_to_reg_out <= 1'b0;
        branch_taken_out <= 1'b0;
        jump_out <= 1'b0;
        jump_for_wb_out <= 1'b0;
        
        // �����ź�����
        pc_out <= 32'h00000000;
        alu_result_out <= 32'h00000000;
        rs2_data_out <= 32'h00000000;
        branch_target_out <= 32'h00000000;
        jump_target_out <= 32'h00000000;
        rd_addr_out <= 5'b00000;
        funct3_out <= 3'b000;
    end
    else if (!stall) begin
        // �����źŴ���
        reg_write_out <= reg_write_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        branch_taken_out <= branch_taken_in;
        jump_out <= jump_in;
        jump_for_wb_out <= jump_for_wb_in;
        
        // �����źŴ���
        pc_out <= pc_in;
        alu_result_out <= alu_result_in;
        rs2_data_out <= rs2_data_in;
        branch_target_out <= branch_target_in;
        jump_target_out <= jump_target_in;
        rd_addr_out <= rd_addr_in;
        funct3_out <= funct3_in;
    end
end

endmodule

// MEM/WB ��ˮ�߼Ĵ���
module mem_wb_register (
    input clk,
    input rst,
    input stall,
    input flush,
    
    // �����ź�����
    input reg_write_in,
    input mem_to_reg_in,
    input jump_for_wb_in,
    
    // �����ź�����
    input [31:0] pc_in,
    input [31:0] alu_result_in,
    input [31:0] mem_data_in,
    input [4:0] rd_addr_in,
    
    // �����ź����
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg jump_for_wb_out,
    
    // �����ź����
    output reg [31:0] pc_out,
    output reg [31:0] alu_result_out,
    output reg [31:0] mem_data_out,
    output reg [4:0] rd_addr_out
);

always @(posedge clk or posedge rst) begin
    if (rst || flush) begin
        // �����ź�����
        reg_write_out <= 1'b0;
        mem_to_reg_out <= 1'b0;
        jump_for_wb_out <= 1'b0;
        
        // �����ź�����
        pc_out <= 32'h00000000;
        alu_result_out <= 32'h00000000;
        mem_data_out <= 32'h00000000;
        rd_addr_out <= 5'b00000;
    end
    else if (!stall) begin
        // �����źŴ���
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        jump_for_wb_out <= jump_for_wb_in;
        
        // �����źŴ���
        pc_out <= pc_in;
        alu_result_out <= alu_result_in;
        mem_data_out <= mem_data_in;
        rd_addr_out <= rd_addr_in;
    end
end

endmodule