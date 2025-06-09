`timescale 1ns/1ps

module cpu_testbench();

// Debug control parameters
parameter DEBUG_LEVEL = 3; // 0=minimal, 1=normal, 2=detailed, 3=verbose
parameter SHOW_PIPELINE = 1; // Show pipeline stages
parameter SHOW_REGISTERS = 1; // Show register values
parameter SHOW_MEMORY = 1; // Show memory operations
parameter SHOW_HAZARDS = 1; // Show hazard detection
parameter MAX_CYCLES = 1000; // Maximum cycles before timeout

// Clock and reset signals
reg clk;
reg rst;

// Instruction memory interface
wire [31:0] imem_addr;
reg [31:0] imem_data;

// Data memory interface
wire [31:0] dmem_addr;
wire [31:0] dmem_wdata;
wire dmem_we;
reg [31:0] dmem_rdata;

// Debug interface
wire [31:0] debug_pc;
wire [31:0] debug_instruction;

// Internal memory arrays - no external Memory.v dependency
reg [31:0] instruction_memory [0:255]; // 256 instructions
reg [31:0] data_memory [0:255]; // 256 words of data memory

// Instantiate CPU
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

// Clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns period, 100MHz
end

// Instruction memory read logic (combinational)
always @(*) begin
    if (imem_addr[9:2] < 256)
        imem_data = instruction_memory[imem_addr[9:2]]; // Word-addressed
    else
        imem_data = 32'h00000013; // NOP for out of range
end

// Data memory read/write logic
always @(posedge clk) begin
    if (dmem_we && dmem_addr[9:2] < 256) begin
        data_memory[dmem_addr[9:2]] <= dmem_wdata;
    end
end

always @(*) begin
    if (dmem_addr[9:2] < 256)
        dmem_rdata = data_memory[dmem_addr[9:2]];
    else
        dmem_rdata = 32'h00000000;
end

// Test variables
integer pass_count;
integer total_tests;
integer i; // for loop variable

// Reset and test control with detailed monitoring
initial begin

    data_memory[0] = 32'h00000004;  // array[0] = 4  (地址0x0 = 0/4 = 0)
    data_memory[1] = 32'h00000005;  // array[1] = 5  (地址0x4 = 4/4 = 1)
    data_memory[2] = 32'h00000003;  // array[2] = 3  (地址0x8 = 8/4 = 2)
    data_memory[3] = 32'h00000001;  // array[3] = 1  (地址0xC = 12/4 = 3)
    data_memory[4] = 32'h00000002;  // array[4] = 2  (地址0x10 = 16/4 = 4)
    for (i = 5; i < 256; i = i + 1) begin
        data_memory[i] = 32'h00000000;
    end
    $display("Data Load Init Finish.");
    $display("dmem[0] = %d", data_memory[0]);
    $display("dmem[1] = %d", data_memory[1]);
    $display("dmem[2] = %d", data_memory[2]);
    $display("dmem[3] = %d", data_memory[3]);
    $display("dmem[4] = %d", data_memory[4]);
    
    instruction_memory[0]  = 32'h00000413;  // addi x8,x0,0x0
    instruction_memory[1]  = 32'h00000293;  // addi x5,x0,0
    instruction_memory[2]  = 32'h00400313;  // addi x6,x0,4
    instruction_memory[3]  = 32'h0462da63;  // bge x5,x6,exit_outer
    instruction_memory[4]  = 32'h00000393;  // addi x7,x0,0
    instruction_memory[5]  = 32'h40530e33;  // sub x28,x6,x5        <- 修正
    instruction_memory[6]  = 32'h05c3d063;  // bge x7,x28,exit_inner
    instruction_memory[7]  = 32'h00239e93;  // slli x29,x7,2
    instruction_memory[8]  = 32'h01d40eb3;  // add x29,x8,x29       <- 修正
    instruction_memory[9]  = 32'h000eaf03;  // lw x30,0(x29)
    instruction_memory[10] = 32'h00138f93;  // addi x31,x7,1
    instruction_memory[11] = 32'h002f9f93;  // slli x31,x31,2
    instruction_memory[12] = 32'h01f40fb3;  // add x31,x8,x31       <- 修正
    instruction_memory[13] = 32'h000faf83;  // lw x31,0(x31)        <- 修正
    instruction_memory[14] = 32'h01ff5c63;  // bge x30,x31,skip_swap <- 修正
    instruction_memory[15] = 32'h01fea023;  // sw x31,0(x29)
    instruction_memory[16] = 32'h00138e13;  // addi x28,x7,1
    instruction_memory[17] = 32'h002e1e13;  // slli x28,x28,2
    instruction_memory[18] = 32'h01c40e33;  // add x28,x8,x28
    instruction_memory[19] = 32'h01ee2023;  // sw x30,0(x28)
    instruction_memory[20] = 32'h00138393;  // addi x7,x7,1
    instruction_memory[21] = 32'hfc1ff06f;  // jal x0,inner_loop
    instruction_memory[22] = 32'h00128293;  // addi x5,x5,1
    instruction_memory[23] = 32'hfb1ff06f;  // jal x0,outer_loop
    instruction_memory[24] = 32'h0000006f;  // jal x0,exit_outer/ addi x31, x30, 30   ; x31 = x30 + 30 = 61 (forwarding)
    
    for (i = 25; i < 1024; i = i + 1) begin
        instruction_memory[i] = 32'h00000013;  // NOP (addi x0, x0, 0)
    end
    
    // Start test sequence
    rst = 1;
    $display("=== Enhanced CPU Testbench with Internal Memory ===");
    $display("Debug Level: %0d, Show Pipeline: %0d", DEBUG_LEVEL, SHOW_PIPELINE);
    #20;
    rst = 0;
    $display("Reset released at time %0t - Starting execution...", $time);
    
    // Run for sufficient cycles
    #2000;
    
    // Final results analysis
    $display("\n=== FINAL RESULTS ===");
    
    // Verify test results
    $display("Register Values:");
    $display("x1  = %2d ",   cpu_inst.reg_file.registers[1]);
    $display("x2  = %2d ",   cpu_inst.reg_file.registers[2]);
    $display("x3  = %2d ",   cpu_inst.reg_file.registers[3]);
    $display("x4  = %2d ",   cpu_inst.reg_file.registers[4]);
    $display("x5  = %2d ",   cpu_inst.reg_file.registers[5]);
    $display("x6  = %2d ",   cpu_inst.reg_file.registers[6]);
    $display("x7  = %2d ",   cpu_inst.reg_file.registers[7]);
    $display("x8  = %2d ",   cpu_inst.reg_file.registers[8]);
    $display("x9  = %2d ",   cpu_inst.reg_file.registers[9]);
    $display("x10 = %2d ",   cpu_inst.reg_file.registers[10]);
    $display("x11 = %2d ",   cpu_inst.reg_file.registers[11]);
    $display("x12 = %2d ",   cpu_inst.reg_file.registers[12]);
    $display("x13 = %2d ",   cpu_inst.reg_file.registers[13]);
    $display("x14 = %2d ",   cpu_inst.reg_file.registers[14]);
    $display("x15 = %2d ",   cpu_inst.reg_file.registers[15]);
    $display("x16 = %2d ",   cpu_inst.reg_file.registers[16]);
    $display("x17 = %2d ",   cpu_inst.reg_file.registers[17]);
    $display("x18 = %2d ",   cpu_inst.reg_file.registers[18]);
    $display("x19 = %2d ",   cpu_inst.reg_file.registers[19]);
    $display("x20 = %2d ",   cpu_inst.reg_file.registers[20]);
    $display("x21 = %2d ",   cpu_inst.reg_file.registers[21]);
    $display("x22 = %2d ",   cpu_inst.reg_file.registers[22]);
    $display("x23 = %2d ",   cpu_inst.reg_file.registers[23]);
    $display("x24 = %2d ",   cpu_inst.reg_file.registers[24]);
    $display("x25 = %2d ",   cpu_inst.reg_file.registers[25]);
    $display("x26 = %2d ",   cpu_inst.reg_file.registers[26]);
    $display("x27 = %2d ",   cpu_inst.reg_file.registers[27]);
    $display("x28 = %2d ",   cpu_inst.reg_file.registers[28]);
    $display("x29 = %2d ",   cpu_inst.reg_file.registers[29]);
    $display("x30 = %2d ",   cpu_inst.reg_file.registers[30]);
    $display("x31 = %2d ",   cpu_inst.reg_file.registers[31]);
      
    // Memory verification
    $display("\nMemory State:");
    $display("mem[0x0] = %d", data_memory[0]);
    $display("mem[0x1] = %d", data_memory[1]);
    $display("mem[0x2] = %d", data_memory[2]);
    $display("mem[0x3] = %d", data_memory[3]);
    $display("mem[0x4] = %d", data_memory[4]);
    
    $display("Total execution cycles: %0d", cycle_count);
    $display("Final PC: 0x%08h", debug_pc);
    $finish;
end

// Detailed debugging monitoring with enhanced format (similar to mixed_hazard_test.v)
integer cycle_count = 0;

// Function to get instruction name for detailed output
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
        7'b1100111: get_instruction_name = "             JALR";  // Jump register
        default:    get_instruction_name = "              NOP";  // Unknown/NOP
    endcase
endfunction

// Comprehensive pipeline monitoring with enhanced debug output (inspired by mixed_hazard_test.v)
always @(posedge clk) begin
    if (!rst && $time >= 20) begin
        cycle_count = cycle_count + 1;
        
        // Timeout protection
        if (cycle_count > MAX_CYCLES) begin
            $display("ERROR: Maximum cycles (%0d) reached. Possible infinite loop!", MAX_CYCLES);
            $finish;
        end
        
        // Main execution display (similar to mixed_hazard_test.v format)
        if (DEBUG_LEVEL >= 1) begin
            $display("Time: %0t, PC: 0x%08h, Inst: 0x%08h, Description: %s", 
                     $time, debug_pc, debug_instruction, get_instruction_name(debug_instruction));
        end
        
        // Pipeline register addresses
        if (DEBUG_LEVEL >= 2 && SHOW_PIPELINE) begin
            $display("  -> Pipeline Register Addresses:");
            $display("     ID: rs1=%2d, rs2=%2d, rd=%2d", 
                     cpu_inst.id_rs1_addr, cpu_inst.id_rs2_addr, cpu_inst.id_rd_addr);
            $display("     EX: rs1=%2d, rs2=%2d, rd=%2d", 
                     cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr, cpu_inst.ex_rd_addr);
        end
        
        // Branch/Jump control transfer detection
        if (DEBUG_LEVEL >= 1) begin
            if (cpu_inst.ex_branch || cpu_inst.ex_jump) begin
                $display("  -> Control Transfer:");
                $display("     EX Branch: %b, Jump: %b", cpu_inst.ex_branch, cpu_inst.ex_jump);
                if (cpu_inst.ex_branch) begin
                    $display("     Branch Type (funct3): %b", cpu_inst.ex_funct3);
                    if (cpu_inst.branch_taken) begin
                        $display("     Branch Taken: %b", cpu_inst.branch_taken);
                        $display("     Branch Target: 0x%08h", cpu_inst.branch_target);
                    end
                end
                if (cpu_inst.ex_jump) begin
                    $display("     Jump Target: 0x%08h", cpu_inst.jump_target);
                end
            end
        end
        
        // PC control information (matching mixed_hazard_test.v)
        if (DEBUG_LEVEL >= 1 && SHOW_PIPELINE) begin
            $display("  -> PC Control:");
            $display("     Current PC: 0x%08h, Next PC: 0x%08h, PC+4: 0x%08h",
                     cpu_inst.pc_current, cpu_inst.pc_next, cpu_inst.pc_plus4);
            $display("     pc_write=%b (1=update, 0=stall)", cpu_inst.pc_write);
        end
        
        // Pipeline flush detection
        if (DEBUG_LEVEL >= 1) begin
            if ((cpu_inst.branch_taken && cpu_inst.ex_branch) || cpu_inst.ex_jump) begin
                $display("  -> PIPELINE FLUSH!");
                $display("     Reason: %s", cpu_inst.ex_branch ? "Branch taken" : "Jump instruction");
                $display("     Flushing IF/ID and ID/EX stages");
            end
        end
          // Register file operations
        if (DEBUG_LEVEL >= 2 && SHOW_REGISTERS) begin
            $display("  -> Register File:");
            $display("     Read: rs1_data=%9d, rs2_data=%9d", 
                     cpu_inst.id_rs1_data, cpu_inst.id_rs2_data);
            
            // Register write detection
            if (cpu_inst.wb_reg_write && cpu_inst.wb_rd_addr != 0) begin
                $display("     Write: rd=%2d, data=%9d", 
                         cpu_inst.wb_rd_addr, cpu_inst.wb_write_data);
            end
            
            // Critical register values for debugging
            $display("     Key Registers: x5=%d, x6=%d, x7=%d, x28=%d", 
                     cpu_inst.reg_file.registers[5], cpu_inst.reg_file.registers[6],
                     cpu_inst.reg_file.registers[7], cpu_inst.reg_file.registers[28]);
        end
        
        // Stall detection
        if (DEBUG_LEVEL >= 1 && SHOW_HAZARDS) begin
            if (!cpu_inst.pc_write) begin
                $display("  -> STALL DETECTED! Load-Use hazard causing pipeline stall");
            end
        end 
          // SUB instruction debug - specialized for debugging SUB execution
        if (DEBUG_LEVEL >= 1 && cpu_inst.ex_opcode == 7'b0110011 && cpu_inst.ex_funct3 == 3'b000 && cpu_inst.ex_funct7[5] == 1'b1) begin
            $display("  -> SUB INSTRUCTION DEBUG:");
            $display("     Instruction: SUB x%d, x%d, x%d", cpu_inst.ex_rd_addr, cpu_inst.ex_rs1_addr, cpu_inst.ex_rs2_addr);
            $display("     Operands: rs1_data(x%d)=%d, rs2_data(x%d)=%d", 
                     cpu_inst.ex_rs1_addr, cpu_inst.ex_rs1_data, cpu_inst.ex_rs2_addr, cpu_inst.ex_rs2_data);
            $display("     ALU Control: alu_op=%b, funct3=%b, funct7=%b", 
                     cpu_inst.ex_alu_op, cpu_inst.ex_funct3, cpu_inst.ex_funct7);
            $display("     ALU Control Signal: %b (should be 0001 for SUB)", cpu_inst.alu_control);
            $display("     ALU Inputs: A=%d, B=%d", cpu_inst.ex_alu_input_a, cpu_inst.ex_alu_input_b);
            $display("     ALU Result: %d (should be %d - %d = %d)", 
                     cpu_inst.ex_alu_result, cpu_inst.ex_alu_input_a, cpu_inst.ex_alu_input_b, 
                     cpu_inst.ex_alu_input_a - cpu_inst.ex_alu_input_b);
            $display("     Forwarding: Forward A=%b, Forward B=%b", cpu_inst.forward_a, cpu_inst.forward_b);
            $display("     Pipeline Status:");
            $display("       MEM stage: rd=%d, reg_write=%b, alu_result=%d", 
                     cpu_inst.mem_rd_addr, cpu_inst.mem_reg_write, cpu_inst.mem_alu_result);
            $display("       WB stage: rd=%d, reg_write=%b, write_data=%d", 
                     cpu_inst.wb_rd_addr, cpu_inst.wb_reg_write, cpu_inst.wb_write_data);            $display("     Forwarding Check:");
            $display("       Forward A: MEM condition: (mem_rd=%d == ex_rs1=%d) && mem_reg_write=%b = %b", 
                     cpu_inst.mem_rd_addr, cpu_inst.ex_rs1_addr, cpu_inst.mem_reg_write,
                     (cpu_inst.mem_rd_addr == cpu_inst.ex_rs1_addr) && cpu_inst.mem_reg_write);
            $display("       Forward A: WB condition: (wb_rd=%d == ex_rs1=%d) && wb_reg_write=%b = %b", 
                     cpu_inst.wb_rd_addr, cpu_inst.ex_rs1_addr, cpu_inst.wb_reg_write,
                     (cpu_inst.wb_rd_addr == cpu_inst.ex_rs1_addr) && cpu_inst.wb_reg_write);
            $display("       Forward B: MEM condition: (mem_rd=%d == ex_rs2=%d) && mem_reg_write=%b = %b", 
                     cpu_inst.mem_rd_addr, cpu_inst.ex_rs2_addr, cpu_inst.mem_reg_write,
                     (cpu_inst.mem_rd_addr == cpu_inst.ex_rs2_addr) && cpu_inst.mem_reg_write);
            $display("       Forward B: WB condition: (wb_rd=%d == ex_rs2=%d) && wb_reg_write=%b = %b", 
                     cpu_inst.wb_rd_addr, cpu_inst.ex_rs2_addr, cpu_inst.wb_reg_write,
                     (cpu_inst.wb_rd_addr == cpu_inst.ex_rs2_addr) && cpu_inst.wb_reg_write);
            $display("     Expected Forward A = %b (01=MEM, 10=WB)", 
                     (cpu_inst.mem_rd_addr == cpu_inst.ex_rs1_addr && cpu_inst.mem_reg_write) ? 2'b01 : 
                     (cpu_inst.wb_rd_addr == cpu_inst.ex_rs1_addr && cpu_inst.wb_reg_write) ? 2'b10 : 2'b00);
        end

        // Data forwarding detection
        if (DEBUG_LEVEL >= 1 && SHOW_HAZARDS) begin
            if (cpu_inst.forward_a != 2'b00 || cpu_inst.forward_b != 2'b00) begin
                $display("  -> DATA FORWARDING:");
                $display("     Forward A: %b, Forward B: %b", cpu_inst.forward_a, cpu_inst.forward_b);
                $display("     ALU Input A (forwarded): %d, ALU Input B_temp (forwarded): %d", 
                         cpu_inst.ex_alu_input_a, cpu_inst.ex_alu_input_b_temp);
                // 如果是分支指令，显示分支比较的详细信息
                if (cpu_inst.ex_branch) begin
                    $display("     Branch comparison: rs1_data=%d >= rs2_data=%d ? %s", 
                             $signed(cpu_inst.ex_alu_input_a), $signed(cpu_inst.ex_alu_input_b_temp),
                             (cpu_inst.branch_taken ? "TRUE" : "FALSE"));
                end
            end
        end
        
        // Memory operations
        if (DEBUG_LEVEL >= 1 && SHOW_MEMORY) begin
            if (cpu_inst.ex_mem_read || cpu_inst.ex_mem_write) begin
                $display("  -> MEMORY OPERATION:");
                if (cpu_inst.ex_mem_read)
                    $display("     Load from address: 0x%08h", cpu_inst.ex_alu_result);
                if (cpu_inst.ex_mem_write)
                    $display("     Store to address: 0x%08h, data: %d", 
                             cpu_inst.ex_alu_result, cpu_inst.ex_alu_input_b_temp);
            end
        end
        
        // Empty line for readability
        if (DEBUG_LEVEL >= 1) begin
            $display("");
        end
    end
end

// Additional monitoring for debugging consistency
always @(posedge clk) begin
    if (!rst && debug_pc !== 0) begin
        // Track execution flow for potential issues
        if (DEBUG_LEVEL >= 3) begin
            $display("Debug: PC consistency check at time %0t", $time);
        end
    end
end

endmodule