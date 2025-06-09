`timescale 1ns/1ps

module risc_v_cpu (
    input clk,                    // 系统时钟：驱动所有时序逻辑
    input rst,                    // 复位信号：系统启动时初始化所有状态
    
    // === 指令存储器接口 ===
    output [31:0] imem_addr,      // 发送给指令存储器的地址（PC值）
    input [31:0] imem_data,       // 从指令存储器读取的32位指令
    
    // === 数据存储器接口 ===
    output [31:0] dmem_addr,      // 发送给数据存储器的地址（Load/Store地址）
    output [31:0] dmem_wdata,     // 写入数据存储器的数据（Store数据）
    output dmem_we,               // 数据存储器写使能（1=写入，0=读取）
    input [31:0] dmem_rdata,      // 从数据存储器读取的数据（Load数据）

    // === 调试接口 ===
    output [31:0] debug_pc,       // 当前PC值，用于调试和仿真
    output [31:0] debug_instruction // 当前指令，用于调试和仿真
);

// ============================================================================
// === 信号声明
// ============================================================================

// === PC控制信号 ===
wire [31:0] pc_current;           // 当前PC值：指向当前正在取指的地址
wire [31:0] pc_next;              // 下一个PC值：下个周期PC将更新到的值
wire [31:0] pc_plus4;             // PC+4：顺序执行时的下一条指令地址
wire pc_write;                    // PC写使能：来自冒险检测，控制PC是否更新

// === IF阶段信号 ===
wire [31:0] if_pc;                // IF阶段的PC值：传递给IF/ID寄存器
wire [31:0] if_instruction;       // IF阶段取到的指令：从指令存储器读取

// === ID阶段信号 ===
// 来自IF/ID寄存器
wire [31:0] id_pc;                // ID阶段的PC值：用于计算分支/跳转目标
wire [31:0] id_instruction;       // ID阶段的指令：需要译码的32位指令

// 指令字段解析
wire [4:0] id_rs1_addr;           // 源寄存器1地址：instruction[19:15]
wire [4:0] id_rs2_addr;           // 源寄存器2地址：instruction[24:20]
wire [4:0] id_rd_addr;            // 目标寄存器地址：instruction[11:7]
wire [2:0] id_funct3;             // 功能字段3：instruction[14:12]
wire [6:0] id_funct7;             // 功能字段7：instruction[31:25]
wire [6:0] id_opcode;             // 操作码：instruction[6:0]

// 寄存器文件和立即数
wire [31:0] id_rs1_data;          // 源寄存器1的数据：reg_file[rs1]的值
wire [31:0] id_rs2_data;          // 源寄存器2的数据：reg_file[rs2]的值
wire [31:0] id_immediate;         // 从指令中解析的立即数：符号扩展后的值

// 控制信号（原始）
wire id_reg_write;                // 寄存器写使能：该指令是否写回寄存器
wire id_mem_read;                 // 内存读使能：是否为Load指令
wire id_mem_write;                // 内存写使能：是否为Store指令
wire id_branch;                   // 分支标志：是否为分支指令
wire id_jump;                     // 跳转标志：是否为跳转指令
wire id_alu_src;                  // ALU源选择：ALU第二个操作数选择
wire id_mem_to_reg;               // 写回选择：写回数据来源选择
wire id_pc_src;                   // PC源选择：PC更新方式选择
wire [1:0] id_alu_op;             // ALU操作类型：传递给ALU控制单元

// 控制信号（经过冒险处理）
wire ctrl_reg_write;              // 处理后的寄存器写使能
wire ctrl_mem_read;               // 处理后的内存读使能
wire ctrl_mem_write;              // 处理后的内存写使能
wire ctrl_branch;                 // 处理后的分支标志
wire ctrl_jump;                   // 处理后的跳转标志
wire ctrl_alu_src;                // 处理后的ALU源选择
wire ctrl_mem_to_reg;             // 处理后的写回选择
wire ctrl_pc_src;                 // 处理后的PC源选择
wire [1:0] ctrl_alu_op;           // 处理后的ALU操作类型

// 分支转发信号
wire [31:0] forwarded_rs1_data;   // 转发后的rs1数据：用于分支判断
wire [31:0] forwarded_rs2_data;   // 转发后的rs2数据：用于分支判断

// === EX阶段信号 ===
// 来自ID/EX寄存器
wire [31:0] ex_pc;                // EX阶段的PC值：用于计算分支/跳转目标
wire [31:0] ex_rs1_data;          // EX阶段的rs1数据：可能被转发修改
wire [31:0] ex_rs2_data;          // EX阶段的rs2数据：可能被转发修改
wire [31:0] ex_immediate;         // EX阶段的立即数：用于ALU计算

// 寄存器地址（用于转发检测）
wire [4:0] ex_rs1_addr;           // EX阶段的rs1地址：转发单元需要
wire [4:0] ex_rs2_addr;           // EX阶段的rs2地址：转发单元需要
wire [4:0] ex_rd_addr;            // EX阶段的rd地址：转发单元需要

// 指令字段（用于ALU控制）
wire [2:0] ex_funct3;             // EX阶段的funct3：ALU具体操作
wire [6:0] ex_funct7;             // EX阶段的funct7：ALU具体操作
wire [6:0] ex_opcode;             // EX阶段的opcode：跳转地址计算

// 控制信号
wire ex_reg_write, ex_mem_read, ex_mem_write, ex_branch, ex_jump;
wire ex_alu_src, ex_mem_to_reg, ex_pc_src;
wire [1:0] ex_alu_op;

// ALU相关信号
wire [31:0] ex_alu_input_a;       // ALU输入A：经过转发处理的数据
wire [31:0] ex_alu_input_b;       // ALU输入B：立即数或转发数据
wire [31:0] ex_alu_input_b_temp;  // ALU输入B临时信号：转发前的数据
wire [31:0] ex_alu_result;        // ALU输出：计算结果
wire [3:0] alu_control;           // ALU控制信号：具体的ALU操作码

// 分支跳转信号
wire branch_taken;                // 分支是否成立：分支条件判断结果
wire [31:0] branch_target;        // 分支目标地址：PC + branch_offset
wire [31:0] jump_target;          // 跳转目标地址：PC + jump_offset

// === MEM阶段信号 ===
// 来自EX/MEM寄存器
wire [31:0] mem_pc;               // MEM阶段的PC值：用于JAL写回
wire [31:0] mem_alu_result;       // MEM阶段的ALU结果：内存地址或运算结果
wire [31:0] mem_rs2_data;         // MEM阶段的rs2数据：Store指令的写入数据
wire [31:0] mem_branch_target;    // MEM阶段的分支目标：用于PC更新
wire [31:0] mem_jump_target;      // MEM阶段的跳转目标：用于PC更新

// 控制和状态信号
wire [4:0] mem_rd_addr;           // MEM阶段的rd地址：写回地址
wire [2:0] mem_funct3;            // MEM阶段的funct3：Load/Store宽度控制
wire mem_reg_write, mem_mem_read, mem_mem_write, mem_mem_to_reg;
wire mem_branch_taken;            // MEM阶段的分支结果：用于PC控制
wire mem_jump;                    // MEM阶段的跳转信号：用于PC控制
wire mem_jump_for_wb;             // MEM阶段的跳转写回标志：用于PC+4写回

// 内存访问信号
wire [31:0] mem_data_out;         // 从数据存储器读取的数据

// === WB阶段信号 ===
// 来自MEM/WB寄存器
wire [31:0] wb_pc;                // WB阶段的PC值：用于计算PC+4
wire [31:0] wb_alu_result;        // WB阶段的ALU结果：算术运算结果
wire [31:0] wb_mem_data;          // WB阶段的内存数据：Load指令数据
wire [31:0] wb_write_data;        // 最终写回数据：经过选择的写回值
wire [31:0] wb_pc_plus4;          // WB阶段的PC+4：JAL指令写回值

// 写回控制
wire [4:0] wb_rd_addr;            // 写回寄存器地址：目标寄存器
wire wb_reg_write;                // 写回使能：是否执行写回
wire wb_mem_to_reg;               // 数据源选择：ALU结果 vs 内存数据
wire wb_jump_for_wb;              // 跳转写回标志：是否写回PC+4

// === 冒险检测和转发信号 ===
// 数据转发控制
wire [1:0] forward_a;             // ALU输入A的转发选择：00=寄存器,01=MEM,10=WB
wire [1:0] forward_b;             // ALU输入B的转发选择：同上

// 流水线控制信号
wire if_id_write;                 // IF/ID寄存器写使能：0=暂停
wire control_mux_sel;             // 控制信号选择：1=插入NOP
wire flush_if_id;                 // 清空IF/ID寄存器：分支预测错误
wire flush_id_ex;                 // 清空ID/EX寄存器：分支预测错误
wire flush_ex_mem;                // 清空EX/MEM寄存器：异常处理

// ============================================================================
// === IF阶段 (取指阶段) 
// ============================================================================

// PC计算逻辑
assign pc_plus4 = pc_current + 4;

// PC选择逻辑：决定下一个PC的来源
assign pc_next = (branch_taken && ex_branch) ? branch_target :  // 分支跳转 (EX阶段)
                 (ex_jump) ? jump_target :                       // 无条件跳转 (EX阶段)
                 pc_plus4;                                       // 顺序执行

// PC寄存器
pc pc_reg (
    .clk(clk),
    .rst(rst),
    .pc_write(pc_write),      // 冒险检测控制PC是否更新
    .pc_next(pc_next),
    .pc_out(pc_current)
);

// 指令存储器接口连接
assign imem_addr = pc_current;    // PC作为指令存储器地址
assign if_pc = pc_current;        // 传递给IF/ID寄存器
assign if_instruction = imem_data; // 从指令存储器读取指令

// ============================================================================
// === IF/ID流水线寄存器
// ============================================================================

if_id_register if_id_reg (
    .clk(clk),
    .rst(rst),
    .stall(~if_id_write),         // 冒险检测控制是否暂停
    .flush(flush_if_id),          // 分支/跳转时清空
    .pc_in(if_pc),
    .instruction_in(if_instruction),
    .pc_out(id_pc),
    .instruction_out(id_instruction)
);

// ============================================================================
// === ID阶段 (译码阶段) 
// ============================================================================

// 指令字段解析
assign id_opcode = id_instruction[6:0];     // 操作码字段
assign id_funct3 = id_instruction[14:12];   // 功能字段3
assign id_funct7 = id_instruction[31:25];   // 功能字段7
assign id_rs1_addr = id_instruction[19:15]; // 源寄存器1地址
assign id_rs2_addr = id_instruction[24:20]; // 源寄存器2地址
assign id_rd_addr = id_instruction[11:7];   // 目标寄存器地址

// 寄存器文件
register_file reg_file (
    .clk(clk),
    .rst(rst),
    .reg_write(wb_reg_write),     // 写控制来自WB阶段
    .rs1(id_rs1_addr),            // 读端口1地址
    .rs2(id_rs2_addr),            // 读端口2地址
    .rd(wb_rd_addr),              // 写端口地址来自WB阶段
    .write_data(wb_write_data),   // 写数据来自WB阶段
    .read_data1(id_rs1_data),     // 读端口1数据
    .read_data2(id_rs2_data)      // 读端口2数据
);

// 立即数生成器
immediate_generator imm_gen (
    .instruction(id_instruction),
    .immediate(id_immediate)      // 生成符号扩展的立即数
);

// 主控制单元
control_unit ctrl_unit (
    .opcode(id_opcode),
    .funct3(id_funct3),
    .funct7(id_funct7),
    // 控制信号输出
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

// 控制冒险处理器（Load-Use冒险时插入NOP）
control_hazard_mux ctrl_hazard_mux (
    // 输入：原始控制信号
    .reg_write_in(id_reg_write),
    .mem_read_in(id_mem_read),
    .mem_write_in(id_mem_write),
    .branch_in(id_branch),
    .jump_in(id_jump),
    .alu_src_in(id_alu_src),
    .mem_to_reg_in(id_mem_to_reg),
    .alu_op_in(id_alu_op),
    .pc_src_in(id_pc_src),
    // 控制信号
    .control_mux_sel(control_mux_sel), // 1=插入NOP，0=正常传递
    // 输出：处理后的控制信号
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

// 分支转发单元（处理分支指令的数据转发）
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
// === ID/EX流水线寄存器
// ============================================================================

id_ex_register id_ex_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 
    .flush(flush_id_ex),          // 分支/跳转时清空
    
    // 控制信号输入
    .reg_write_in(ctrl_reg_write),
    .mem_read_in(ctrl_mem_read),
    .mem_write_in(ctrl_mem_write),
    .branch_in(ctrl_branch),
    .jump_in(ctrl_jump),
    .alu_src_in(ctrl_alu_src),
    .mem_to_reg_in(ctrl_mem_to_reg),
    .alu_op_in(ctrl_alu_op),
    .pc_src_in(ctrl_pc_src),
    
    // 数据信号输入
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
    
    // 控制信号输出
    .reg_write_out(ex_reg_write),
    .mem_read_out(ex_mem_read),
    .mem_write_out(ex_mem_write),
    .branch_out(ex_branch),
    .jump_out(ex_jump),
    .alu_src_out(ex_alu_src),
    .mem_to_reg_out(ex_mem_to_reg),
    .alu_op_out(ex_alu_op),
    .pc_src_out(ex_pc_src),
    
    // 数据信号输出
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
// === EX阶段 (执行阶段)
// ============================================================================

// 数据转发单元
forwarding_unit forward_unit (
    .ex_rs1(ex_rs1_addr),         // EX阶段的源寄存器1
    .ex_rs2(ex_rs2_addr),         // EX阶段的源寄存器2
    .mem_rd(mem_rd_addr),         // MEM阶段的目标寄存器
    .mem_reg_write(mem_reg_write), // MEM阶段是否写回
    .wb_rd(wb_rd_addr),           // WB阶段的目标寄存器
    .wb_reg_write(wb_reg_write),  // WB阶段是否写回
    .forward_a(forward_a),        // ALU输入A的转发控制
    .forward_b(forward_b)         // ALU输入B的转发控制
);

// ALU输入A的转发多路选择器
forwarding_mux alu_input_a_mux (
    .reg_data(ex_rs1_data),       // 来自寄存器的原始数据
    .mem_forward_data(mem_alu_result), // 从MEM阶段转发的数据
    .wb_forward_data(wb_write_data),   // 从WB阶段转发的数据
    .forward_sel(forward_a),      // 转发选择信号
    .mux_out(ex_alu_input_a)      // 最终的ALU输入A
);

// ALU输入B的转发多路选择器
forwarding_mux alu_input_b_mux (
    .reg_data(ex_rs2_data),       // 来自寄存器的原始数据
    .mem_forward_data(mem_alu_result), // 从MEM阶段转发的数据
    .wb_forward_data(wb_write_data),   // 从WB阶段转发的数据
    .forward_sel(forward_b),      // 转发选择信号
    .mux_out(ex_alu_input_b_temp) // 转发后的rs2数据
);

// ALU源数据选择器（寄存器数据 vs 立即数）
assign ex_alu_input_b = ex_alu_src ? ex_immediate : ex_alu_input_b_temp;

// ALU控制单元
alu_control alu_ctrl (
    .alu_op(ex_alu_op),           // 来自控制单元的操作类型
    .funct3(ex_funct3),           // 指令的funct3字段
    .funct7(ex_funct7),           // 指令的funct7字段
    .alu_control_out(alu_control) // 生成具体的ALU控制信号
);

// ALU (算术逻辑单元)
alu main_alu (
    .a(ex_alu_input_a),           // ALU输入A
    .b(ex_alu_input_b),           // ALU输入B
    .alu_control(alu_control),    // ALU控制信号
    .result(ex_alu_result)        // ALU计算结果
);

// 分支控制单元
branch_control branch_ctrl (
    .branch(ex_branch),           // 是否为分支指令
    .funct3(ex_funct3),           // 分支类型 (BEQ, BNE, BLT等)
    .rs1_data(ex_alu_input_a),    // 比较数据1 (转发后)
    .rs2_data(ex_alu_input_b_temp), // 比较数据2 (转发后的rs2，不是立即数)
    .branch_taken(branch_taken)   // 分支是否成立
);

// 分支和跳转目标地址计算
assign branch_target = ex_pc + ex_immediate; // 分支目标地址
assign jump_target = ex_pc + ex_immediate;   // 跳转目标地址

// ============================================================================
// === EX/MEM流水线寄存器
// ============================================================================

ex_mem_register ex_mem_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 
    .flush(flush_ex_mem),         // 异常时清空
    
    // 控制信号输入
    .reg_write_in(ex_reg_write),
    .mem_read_in(ex_mem_read),
    .mem_write_in(ex_mem_write),
    .mem_to_reg_in(ex_mem_to_reg),
    .branch_taken_in(branch_taken),
    .jump_in(ex_jump),
    .jump_for_wb_in(ex_jump),     // JAL指令需要写回PC+4
    
    // 数据信号输入
    .pc_in(ex_pc),
    .alu_result_in(ex_alu_result),
    .rs2_data_in(ex_alu_input_b_temp), // Store指令的写数据
    .branch_target_in(branch_target),
    .jump_target_in(jump_target),
    .rd_addr_in(ex_rd_addr),
    .funct3_in(ex_funct3),
    
    // 控制信号输出
    .reg_write_out(mem_reg_write),
    .mem_read_out(mem_mem_read),
    .mem_write_out(mem_mem_write),
    .mem_to_reg_out(mem_mem_to_reg),
    .branch_taken_out(mem_branch_taken),
    .jump_out(mem_jump),
    .jump_for_wb_out(mem_jump_for_wb),
    
    // 数据信号输出
    .pc_out(mem_pc),
    .alu_result_out(mem_alu_result),
    .rs2_data_out(mem_rs2_data),
    .branch_target_out(mem_branch_target),
    .jump_target_out(mem_jump_target),
    .rd_addr_out(mem_rd_addr),
    .funct3_out(mem_funct3)
);

// ============================================================================
// === MEM阶段 (访存阶段) 
// ============================================================================

// 数据存储器接口连接
assign dmem_addr = mem_alu_result;    // 内存地址来自ALU计算结果
assign dmem_wdata = mem_rs2_data;     // 写数据来自rs2寄存器
assign dmem_we = mem_mem_write;       // 写使能来自控制信号
assign mem_data_out = dmem_rdata;     // 读数据来自存储器

// ============================================================================
// === MEM/WB流水线寄存器
// ============================================================================

mem_wb_register mem_wb_reg (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),                 // WB阶段不需要暂停
    .flush(1'b0),                 // WB阶段不需要清空
    
    // 控制信号输入
    .reg_write_in(mem_reg_write),
    .mem_to_reg_in(mem_mem_to_reg),
    .jump_for_wb_in(mem_jump_for_wb),
    
    // 数据信号输入
    .pc_in(mem_pc),
    .alu_result_in(mem_alu_result),
    .mem_data_in(mem_data_out),
    .rd_addr_in(mem_rd_addr),
    
    // 控制信号输出
    .reg_write_out(wb_reg_write),
    .mem_to_reg_out(wb_mem_to_reg),
    .jump_for_wb_out(wb_jump_for_wb),
    
    // 数据信号输出
    .pc_out(wb_pc),
    .alu_result_out(wb_alu_result),
    .mem_data_out(wb_mem_data),
    .rd_addr_out(wb_rd_addr)
);

// ============================================================================
// === WB阶段 (写回阶段)
// ============================================================================

// 计算PC+4（用于JAL指令写回）
assign wb_pc_plus4 = wb_pc + 4;

// 写回数据选择器
assign wb_write_data = wb_jump_for_wb ? wb_pc_plus4 :      // JAL指令：写回PC+4
                       wb_mem_to_reg ? wb_mem_data :       // Load指令：写回内存数据
                       wb_alu_result;                      // 其他指令：写回ALU结果

// ============================================================================
// === 冒险检测和处理单元
// ============================================================================

hazard_detection_unit hazard_detect (
    // 输入：当前指令的源寄存器
    .id_rs1(id_rs1_addr),
    .id_rs2(id_rs2_addr),
    
    // 输入：EX阶段的Load指令信息
    .ex_rd(ex_rd_addr),
    .ex_mem_read(ex_mem_read),
    
    // 输入：分支跳转信号（来自EX阶段，提前检测）
    .branch_taken(branch_taken),
    .jump(ex_jump),
    
    // 输出：流水线控制信号
    .pc_write(pc_write),          // 控制PC是否更新
    .if_id_write(if_id_write),    // 控制IF/ID寄存器是否更新
    .control_mux_sel(control_mux_sel), // 控制是否插入NOP
    .flush_if_id(flush_if_id),    // 清空IF/ID寄存器
    .flush_id_ex(flush_id_ex),    // 清空ID/EX寄存器
    .flush_ex_mem(flush_ex_mem)   // 清空EX/MEM寄存器
);

// ============================================================================
// === 调试接口
// ============================================================================

assign debug_pc = pc_current;        // 输出当前PC用于调试
assign debug_instruction = if_instruction; // 输出当前指令用于调试

endmodule