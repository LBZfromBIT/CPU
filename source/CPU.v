`timescale 1ns/1ps

module risc_v_cpu (
    input clk,
    input rst,
    
    // 指令存储器接口
    output [31:0] imem_addr,
    input [31:0] imem_data,
    
    // 数据存储器接口
    output [31:0] dmem_addr,
    output [31:0] dmem_wdata,
    output dmem_we,
    input [31:0] dmem_rdata,

    // 调试接口
    output [31:0] debug_pc,
    output [31:0] debug_instruction
);

// ========== 信号声明 ==========
// PC相关信号
wire [31:0] pc_current, pc_next, pc_plus4;
wire pc_write;

// IF阶段信号
wire [31:0] if_pc, if_instruction;

// ID阶段信号
wire [31:0] id_pc, id_instruction;
wire [31:0] id_rs1_data, id_rs2_data, id_immediate;
wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
wire [2:0] id_funct3;
wire [6:0] id_funct7, id_opcode;

// 控制信号
wire id_reg_write, id_mem_read, id_mem_write, id_branch, id_jump;
wire id_alu_src, id_mem_to_reg, id_pc_src;
wire [1:0] id_alu_op;

// 经过冒险处理后的控制信号
wire ctrl_reg_write, ctrl_mem_read, ctrl_mem_write, ctrl_branch, ctrl_jump;
wire ctrl_alu_src, ctrl_mem_to_reg, ctrl_pc_src;
wire [1:0] ctrl_alu_op;

// EX阶段信号
wire [31:0] ex_pc, ex_rs1_data, ex_rs2_data, ex_immediate;
wire [31:0] ex_alu_input_a, ex_alu_input_b, ex_alu_result;
wire [4:0] ex_rs1_addr, ex_rs2_addr, ex_rd_addr;
wire [2:0] ex_funct3;
wire [6:0] ex_funct7;
wire [6:0] ex_opcode;
wire [3:0] alu_control;
wire ex_reg_write, ex_mem_read, ex_mem_write, ex_branch, ex_jump;
wire ex_alu_src, ex_mem_to_reg, ex_pc_src;
wire [1:0] ex_alu_op;

// 分支跳转相关
wire branch_taken;
wire [31:0] branch_target, jump_target;

// MEM阶段信号
wire [31:0] mem_pc, mem_alu_result, mem_rs2_data, mem_branch_target, mem_jump_target;
wire [31:0] mem_data_out;
wire [4:0] mem_rd_addr;
wire [2:0] mem_funct3;
wire mem_reg_write, mem_mem_read, mem_mem_write, mem_mem_to_reg;
wire mem_branch_taken, mem_jump;
wire mem_jump_for_wb;

// WB阶段信号
wire [31:0] wb_pc, wb_alu_result, wb_mem_data, wb_write_data;
wire [4:0] wb_rd_addr;
wire wb_reg_write, wb_mem_to_reg;
wire wb_jump_for_wb;

// 冒险检测和转发信号
wire [1:0] forward_a, forward_b;
wire if_id_write, control_mux_sel;
wire flush_if_id, flush_id_ex, flush_ex_mem;
wire [31:0] forwarded_rs1_data, forwarded_rs2_data;



// ========== 取指阶段 (IF) ==========
// PC计算
assign pc_plus4 = pc_current + 4;

// PC选择逻辑
assign pc_next = (mem_branch_taken) ? mem_branch_target :
                 (mem_jump) ? mem_jump_target :
                 pc_plus4;

// PC寄存器
pc pc_reg (
    .clk(clk),
    .rst(rst),
    .pc_write(pc_write),
    .pc_next(pc_next),
    .pc_out(pc_current)
);

// 指令存储器接口
assign imem_addr = pc_current;
assign if_pc = pc_current;
assign if_instruction = imem_data;

// ========== IF/ID流水线寄存器 ==========
if_id_register if_id_reg (
    .clk(clk),
    .rst(rst),
    .stall(~if_id_write),
    .flush(flush_if_id),
    .pc_in(if_pc),
    .instruction_in(if_instruction),
    .pc_out(id_pc),
    .instruction_out(id_instruction)
);

// ========== 译码阶段 (ID) ==========
// 指令字段解析
assign id_opcode = id_instruction[6:0];
assign id_funct3 = id_instruction[14:12];
assign id_funct7 = id_instruction[31:25];
assign id_rs1_addr = id_instruction[19:15];
assign id_rs2_addr = id_instruction[24:20];
assign id_rd_addr = id_instruction[11:7];

// 寄存器文件
register_file reg_file (
    .clk(clk),
    .rst(rst),
    .reg_write(wb_reg_write),
    .rs1(id_rs1_addr),
    .rs2(id_rs2_addr),
    .rd(wb_rd_addr),
    .write_data(wb_write_data),
    .read_data1(id_rs1_data),
    .read_data2(id_rs2_data)
);

// 立即数生成器
immediate_generator imm_gen (
    .instruction(id_instruction),
    .immediate(id_immediate)
);

// 主控制单元
control_unit ctrl_unit (
    .opcode(id_opcode),
    .funct3(id_funct3),
    .funct7(id_funct7),
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

// 控制冒险处理
control_hazard_mux ctrl_hazard_mux (
    .reg_write_in(id_reg_write),
    .mem_read_in(id_mem_read),
    .mem_write_in(id_mem_write),
    .branch_in(id_branch),
    .jump_in(id_jump),
    .alu_src_in(id_alu_src),
    .mem_to_reg_in(id_mem_to_reg),
    .alu_op_in(id_alu_op),
    .pc_src_in(id_pc_src),
    .control_mux_sel(control_mux_sel),
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

// 分支转发单元
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

// ========== ID/EX流水线寄存器 ==========
id_ex_register id_ex_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .flush(flush_id_ex),
    // 控制信号
    .reg_write_in(ctrl_reg_write),
    .mem_read_in(ctrl_mem_read),
    .mem_write_in(ctrl_mem_write),
    .branch_in(ctrl_branch),
    .jump_in(ctrl_jump),
    .alu_src_in(ctrl_alu_src),
    .mem_to_reg_in(ctrl_mem_to_reg),
    .alu_op_in(ctrl_alu_op),
    .pc_src_in(ctrl_pc_src),
    // 数据信号
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
    // 输出
    .reg_write_out(ex_reg_write),
    .mem_read_out(ex_mem_read),
    .mem_write_out(ex_mem_write),
    .branch_out(ex_branch),
    .jump_out(ex_jump),
    .alu_src_out(ex_alu_src),
    .mem_to_reg_out(ex_mem_to_reg),
    .alu_op_out(ex_alu_op),
    .pc_src_out(ex_pc_src),
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

// ========== 执行阶段 (EX) ==========
// 数据转发单元
forwarding_unit forward_unit (
    .ex_rs1(ex_rs1_addr),
    .ex_rs2(ex_rs2_addr),
    .mem_rd(mem_rd_addr),
    .mem_reg_write(mem_reg_write),
    .wb_rd(wb_rd_addr),
    .wb_reg_write(wb_reg_write),
    .forward_a(forward_a),
    .forward_b(forward_b)
);

// ALU输入A的多路选择器
forwarding_mux alu_input_a_mux (
    .reg_data(ex_rs1_data),
    .mem_forward_data(mem_alu_result),
    .wb_forward_data(wb_write_data),
    .forward_sel(forward_a),
    .mux_out(ex_alu_input_a)
);

// ALU输入B的多路选择器
wire [31:0] ex_alu_input_b_temp;
forwarding_mux alu_input_b_mux (
    .reg_data(ex_rs2_data),
    .mem_forward_data(mem_alu_result),
    .wb_forward_data(wb_write_data),
    .forward_sel(forward_b),
    .mux_out(ex_alu_input_b_temp)
);

// ALU源数据选择（寄存器 vs 立即数）
assign ex_alu_input_b = ex_alu_src ? ex_immediate : ex_alu_input_b_temp;

// ALU控制单元
alu_control alu_ctrl (
    .alu_op(ex_alu_op),
    .funct3(ex_funct3),
    .funct7(ex_funct7),
    .alu_control_out(alu_control)
);

// ALU
alu main_alu (
    .a(ex_alu_input_a),
    .b(ex_alu_input_b),
    .alu_control(alu_control),
    .result(ex_alu_result),
    .zero(/* 未使用 */)
);

// 分支控制单元
branch_control branch_ctrl (
    .branch(ex_branch),
    .funct3(ex_funct3),
    .rs1_data(ex_alu_input_a),
    .rs2_data(ex_alu_input_b_temp),
    .branch_taken(branch_taken)
);

// 分支目标地址计算
assign branch_target = ex_pc + ex_immediate;
assign jump_target = ex_pc + ex_immediate;

// ========== EX/MEM流水线寄存器 ==========
ex_mem_register ex_mem_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .flush(flush_ex_mem),
    // 控制信号
    .reg_write_in(ex_reg_write),
    .mem_read_in(ex_mem_read),
    .mem_write_in(ex_mem_write),
    .mem_to_reg_in(ex_mem_to_reg),
    .branch_taken_in(branch_taken),
    .jump_in(ex_jump),
    .jump_for_wb_in(ex_jump), // 用于写回阶段的跳转信号
    // 数据信号
    .pc_in(ex_pc),
    .alu_result_in(ex_alu_result),
    .rs2_data_in(ex_alu_input_b_temp),
    .branch_target_in(branch_target),
    .jump_target_in(jump_target),
    .rd_addr_in(ex_rd_addr),
    .funct3_in(ex_funct3),
    // 输出
    .reg_write_out(mem_reg_write),
    .mem_read_out(mem_mem_read),
    .mem_write_out(mem_mem_write),
    .mem_to_reg_out(mem_mem_to_reg),
    .branch_taken_out(mem_branch_taken),
    .jump_out(mem_jump),
    .jump_for_wb_out(mem_jump_for_wb),
    .pc_out(mem_pc),
    .alu_result_out(mem_alu_result),
    .rs2_data_out(mem_rs2_data),
    .branch_target_out(mem_branch_target),
    .jump_target_out(mem_jump_target),
    .rd_addr_out(mem_rd_addr),
    .funct3_out(mem_funct3)
);

// ========== 访存阶段 (MEM) ==========
// 数据存储器接口
assign dmem_addr = mem_alu_result;
assign dmem_wdata = mem_rs2_data;
assign dmem_we = mem_mem_write;
assign mem_data_out = dmem_rdata;

// ========== MEM/WB流水线寄存器 ==========
mem_wb_register mem_wb_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .flush(1'b0),
    // 控制信号
    .reg_write_in(mem_reg_write),
    .mem_to_reg_in(mem_mem_to_reg),
    .jump_for_wb_in(mem_jump_for_wb),
    // 数据信号
    .pc_in(mem_pc),
    .alu_result_in(mem_alu_result),
    .mem_data_in(mem_data_out),
    .rd_addr_in(mem_rd_addr),
    // 输出
    .reg_write_out(wb_reg_write),
    .mem_to_reg_out(wb_mem_to_reg),
    .jump_for_wb_out(wb_jump_for_wb),
    .pc_out(wb_pc),
    .alu_result_out(wb_alu_result),
    .mem_data_out(wb_mem_data),
    .rd_addr_out(wb_rd_addr)
);

// ========== 写回阶段 (WB) ==========
// 写回数据选择
wire [31:0] wb_pc_plus4 = wb_pc + 4;
assign wb_write_data = wb_jump_for_wb ? wb_pc_plus4 :       // JAL指令写回PC+4
                       wb_mem_to_reg ? wb_mem_data :        // Load指令写回内存数据
                       wb_alu_result;                 

// ========== 冒险检测单元 ==========
hazard_detection_unit hazard_detect (
    .id_rs1(id_rs1_addr),
    .id_rs2(id_rs2_addr),
    .ex_rd(ex_rd_addr),
    .ex_mem_read(ex_mem_read),
    .branch_taken(mem_branch_taken),
    .jump(mem_jump),
    .pc_write(pc_write),
    .if_id_write(if_id_write),
    .control_mux_sel(control_mux_sel),
    .flush_if_id(flush_if_id),
    .flush_id_ex(flush_id_ex),
    .flush_ex_mem(flush_ex_mem)
);

// ========== 调试接口 ==========
assign debug_pc = pc_current;
assign debug_instruction = if_instruction;

endmodule