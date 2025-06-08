`timescale 1ns/1ps

module instruction_memory (
    input [31:0] pc,
    output [31:0] instruction
);

reg [31:0] imem [0:1023];  // 指令存储器

integer i;

// 初始化指令存储器
initial begin
    // 初始化所有位置为NOP指令
    for (i = 0; i < 1024; i = i + 1) begin
        imem[i] = 32'h00000013;  // NOP (addi x0, x0, 0)
    end
    
    imem[0]  = 32'h10000413;  // addi x8,x0,0x100
    imem[1]  = 32'h00000293;  // addi x5,x0,0
    imem[2]  = 32'h00400313;  // addi x6,x0,4
    imem[3]  = 32'h0462da63;  // bge x5,x6,exit_outer
    imem[4]  = 32'h00000393;  // addi x7,x0,0
    imem[5]  = 32'h40530e33;  // sub x28,x6,x5        <- 修正
    imem[6]  = 32'h05c3d063;  // bge x7,x28,exit_inner
    imem[7]  = 32'h00239e93;  // slli x29,x7,2
    imem[8]  = 32'h01d40eb3;  // add x29,x8,x29       <- 修正
    imem[9]  = 32'h000eaf03;  // lw x30,0(x29)
    imem[10] = 32'h00138f93;  // addi x31,x7,1
    imem[11] = 32'h002f9f93;  // slli x31,x31,2
    imem[12] = 32'h01f40fb3;  // add x31,x8,x31       <- 修正
    imem[13] = 32'h000faf83;  // lw x31,0(x31)        <- 修正
    imem[14] = 32'h01ff5c63;  // bge x30,x31,skip_swap <- 修正
    imem[15] = 32'h01fea023;  // sw x31,0(x29)
    imem[16] = 32'h00138e13;  // addi x28,x7,1
    imem[17] = 32'h002e1e13;  // slli x28,x28,2
    imem[18] = 32'h01c40e33;  // add x28,x8,x28
    imem[19] = 32'h01ee2023;  // sw x30,0(x28)
    imem[20] = 32'h00138393;  // addi x7,x7,1
    imem[21] = 32'hfc1ff06f;  // jal x0,inner_loop
    imem[22] = 32'h00128293;  // addi x5,x5,1
    imem[23] = 32'hfb1ff06f;  // jal x0,outer_loop
    imem[24] = 32'h0000006f;  // jal x0,exit_outer
    
    $display("指令存储器初始化完成");
end

// 按字对齐访问
assign instruction = imem[pc[31:2]];

endmodule


module data_memory (
    input clk,
    input mem_read,
    input mem_write,
    input [31:0] address,
    input [31:0] write_data,
    output reg [31:0] read_data
);

reg [31:0] dmem [0:1023];  // 数据存储器

integer j;

// 初始化数据存储器
initial begin
    // 初始化所有位置为0
    for (j = 0; j < 1024; j = j + 1) begin
        dmem[j] = 32'h00000000;
    end
    
    // 在地址0x100处初始化数组数据
    dmem[64] = 32'h00000004;  // array[0] = 4  (地址0x100 = 256/4 = 64)
    dmem[65] = 32'h00000005;  // array[1] = 5  (地址0x104 = 260/4 = 65)
    dmem[66] = 32'h00000003;  // array[2] = 3  (地址0x108 = 264/4 = 66)
    dmem[67] = 32'h00000001;  // array[3] = 1  (地址0x10C = 268/4 = 67)
    dmem[68] = 32'h00000002;  // array[4] = 2  (地址0x110 = 272/4 = 68)
    
    $display("数据存储器初始化完成");
    $display("dmem[64] = %d", dmem[64]);
    $display("dmem[65] = %d", dmem[65]);
    $display("dmem[66] = %d", dmem[66]);
    $display("dmem[67] = %d", dmem[67]);
    $display("dmem[68] = %d", dmem[68]);
end

// 按字对齐的地址
wire [31:0] word_address = address[31:2];

// 读操作
always @(*) begin
    if (mem_read) begin
        if (word_address < 1024) begin
            read_data = dmem[word_address];
        end else begin
            read_data = 32'h00000000;
        end
    end else begin
        read_data = 32'h00000000;
    end
end

// 写操作
always @(posedge clk) begin
    if (mem_write) begin
        if (word_address < 1024) begin
            dmem[word_address] <= write_data;
            $display("Memory Write: addr=0x%h (word_addr=%d), data=%d", 
                     address, word_address, write_data);
        end
    end
end

endmodule