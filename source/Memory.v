`timescale 1ns/1ps

//instruction memory module
module instruction_memory(
    input [31:0] pc,  //PC value
    output [31:0] instruction //output instruction
);
reg [31:0] imem [0:1023]; // 1024*32 instruction memory

initial begin
    // Load instructions into memory (example instructions)
end

assign instruction = imem[pc[11:2]]; // PC is aligned to 4 bytes, using bits [11:2] to index

endmodule


// data memory module
module data_memory (
    input clk,
    input mem_read,          // read enable
    input mem_write,         // write enable
    input [31:0] address,    
    input [31:0] write_data, 
    output [31:0] read_data  
);

reg [31:0] dmem [0:1023]; // 1024 * 32 data memory

// initial memory content
initial begin
    dmem[64] = 32'd4;  // init_address 0x100 is 64 
    dmem[65] = 32'd5;
    dmem[66] = 32'd3;
    dmem[67] = 32'd1;
    dmem[68] = 32'd2;
end

// write synchronously
always @(posedge clk) begin
    if (mem_write)
        dmem[address[11:2]] <= write_data;
end

// read asynchronously
assign read_data = mem_read ? dmem[address[11:2]] : 32'h00000000; //default to zero if not reading

endmodule