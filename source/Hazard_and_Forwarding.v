`timescale 1ns/1ps

// 数据转发单元
module forwarding_unit (
    // EX阶段的源寄存器地址
    input [4:0] ex_rs1,
    input [4:0] ex_rs2,
    
    // MEM阶段的目标寄存器信息
    input [4:0] mem_rd,
    input mem_reg_write,
    
    // WB阶段的目标寄存器信息
    input [4:0] wb_rd,
    input wb_reg_write,
    
    // 转发控制信号
    output reg [1:0] forward_a,    // ALU输入A的转发控制
    output reg [1:0] forward_b     // ALU输入B的转发控制
);

// 转发控制信号编码
parameter NO_FORWARD = 2'b00;      // 不转发，使用寄存器文件数据
parameter FORWARD_MEM = 2'b01;     // 从MEM阶段转发
parameter FORWARD_WB = 2'b10;      // 从WB阶段转发

always @(*) begin
    // ALU输入A的转发逻辑
    forward_a = NO_FORWARD;
    if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == ex_rs1)) begin
        forward_a = FORWARD_MEM;  // MEM阶段转发优先级更高
    end
    else if (wb_reg_write && (wb_rd != 5'b00000) && (wb_rd == ex_rs1)) begin
        forward_a = FORWARD_WB;
    end
    
    // ALU输入B的转发逻辑
    forward_b = NO_FORWARD;
    if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == ex_rs2)) begin
        forward_b = FORWARD_MEM;  // MEM阶段转发优先级更高
    end
    else if (wb_reg_write && (wb_rd != 5'b00000) && (wb_rd == ex_rs2)) begin
        forward_b = FORWARD_WB;
    end
end

endmodule

// 冒险检测单元
module hazard_detection_unit (
    // ID阶段的源寄存器地址
    input [4:0] id_rs1,
    input [4:0] id_rs2,
    
    // EX阶段的load指令信息
    input [4:0] ex_rd,
    input ex_mem_read,
    
    // 分支和跳转信号
    input branch_taken,
    input jump,
    
    // 控制信号输出
    output reg pc_write,           // PC写使能
    output reg if_id_write,        // IF/ID寄存器写使能
    output reg control_mux_sel,    // 控制信号选择（插入bubble）
    output reg flush_if_id,        // 清空IF/ID寄存器
    output reg flush_id_ex,        // 清空ID/EX寄存器
    output reg flush_ex_mem        // 清空EX/MEM寄存器
);

always @(*) begin
    // 默认值：正常流水线操作
    pc_write = 1'b1;
    if_id_write = 1'b1;
    control_mux_sel = 1'b0;
    flush_if_id = 1'b0;
    flush_id_ex = 1'b0;
    flush_ex_mem = 1'b0;
    
    // Load-Use 冒险检测
    // 当EX阶段是load指令，且其目标寄存器是ID阶段指令的源寄存器时
    if (ex_mem_read && 
        ((ex_rd == id_rs1) || (ex_rd == id_rs2)) && 
        (ex_rd != 5'b00000)) begin
        
        // 插入一个stall周期
        pc_write = 1'b0;           // 暂停PC更新
        if_id_write = 1'b0;        // 暂停IF/ID寄存器
        control_mux_sel = 1'b1;    // 在ID/EX中插入bubble（NOP）
    end
    
    // 分支跳转冒险处理
    if (branch_taken || jump) begin
        flush_if_id = 1'b1;        // 清空IF/ID（取消已取指令）
        flush_id_ex = 1'b1;        // 清空ID/EX（取消已译码指令）
    end
end

endmodule

// 分支转发单元（用于分支指令的早期解决）
module branch_forwarding_unit (
    // ID阶段的源寄存器地址
    input [4:0] id_rs1,
    input [4:0] id_rs2,
    
    // EX阶段的寄存器信息
    input [4:0] ex_rd,
    input ex_reg_write,
    input [31:0] ex_alu_result,
    
    // MEM阶段的寄存器信息
    input [4:0] mem_rd,
    input mem_reg_write,
    input [31:0] mem_data,
    
    // 原始寄存器数据
    input [31:0] id_rs1_data,
    input [31:0] id_rs2_data,
    
    // 转发后的寄存器数据
    output reg [31:0] forwarded_rs1_data,
    output reg [31:0] forwarded_rs2_data
);

always @(*) begin
    // RS1转发逻辑
    forwarded_rs1_data = id_rs1_data;  // 默认使用原始数据
    if (ex_reg_write && (ex_rd != 5'b00000) && (ex_rd == id_rs1)) begin
        forwarded_rs1_data = ex_alu_result;  // 从EX阶段转发
    end
    else if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == id_rs1)) begin
        forwarded_rs1_data = mem_data;       // 从MEM阶段转发
    end
    
    // RS2转发逻辑
    forwarded_rs2_data = id_rs2_data;  // 默认使用原始数据
    if (ex_reg_write && (ex_rd != 5'b00000) && (ex_rd == id_rs2)) begin
        forwarded_rs2_data = ex_alu_result;  // 从EX阶段转发
    end
    else if (mem_reg_write && (mem_rd != 5'b00000) && (mem_rd == id_rs2)) begin
        forwarded_rs2_data = mem_data;       // 从MEM阶段转发
    end
end

endmodule

// 数据转发多路选择器
module forwarding_mux (
    input [31:0] reg_data,         // 来自寄存器文件的数据
    input [31:0] mem_forward_data, // 来自MEM阶段的转发数据
    input [31:0] wb_forward_data,  // 来自WB阶段的转发数据
    input [1:0] forward_sel,       // 转发选择信号
    output reg [31:0] mux_out      // 多路选择器输出
);

always @(*) begin
    case (forward_sel)
        2'b00: mux_out = reg_data;         // 不转发
        2'b01: mux_out = mem_forward_data; // MEM阶段转发
        2'b10: mux_out = wb_forward_data;  // WB阶段转发
        default: mux_out = reg_data;
    endcase
end

endmodule

// 控制冒险处理单元（用于插入NOP）
module control_hazard_mux (
    // 正常控制信号
    input reg_write_in,
    input mem_read_in,
    input mem_write_in,
    input branch_in,
    input jump_in,
    input alu_src_in,
    input mem_to_reg_in,
    input [1:0] alu_op_in,
    input pc_src_in,
    
    // 控制选择信号
    input control_mux_sel,  // 0: 正常信号, 1: 插入NOP
    
    // 输出控制信号
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
        // 插入NOP（所有控制信号置0）
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
        // 正常传递控制信号
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