`timescale 1ns/1ps

module register_file (
    input clk,
    input rst,
    input reg_write,         // write enable signal
    input [4:0] rs1,         // source1
    input [4:0] rs2,         // source2  
    input [4:0] rd,          // destination
    input [31:0] write_data, 
    output [31:0] read_data1,
    output [31:0] read_data2 
);

reg [31:0] registers [31:0]; // 32*32 register file

integer i;
always @(posedge clk or posedge rst) begin
    if (rst) begin  // initialization after reset
        for (i = 0; i < 32; i = i + 1)
            registers[i] <= 32'h00000000;
    end
    else if (reg_write && rd != 5'b00000) begin // 5'b00000 is the zero,always be null,cannot be written
        registers[rd] <= write_data;
    end
end

// Read operations
// 5'b00000 is the zero,always be null
assign read_data1 = (rs1 == 5'b00000) ? 32'h00000000 : registers[rs1];
assign read_data2 = (rs2 == 5'b00000) ? 32'h00000000 : registers[rs2];

endmodule