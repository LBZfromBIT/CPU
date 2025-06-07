`timescale 1ns / 1ps

module pc(
    input clk,
    input rst,
    input pc_write,  //write enable for PC
    input [31:0] pc_next, //input to PC ,the next instruction address
    output reg [31:0] pc_out //output of PC,the current instruction address
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        pc_out <= 32'b0; // Reset PC to 0 on reset signal
    end else if (pc_write) begin
        pc_out <= pc_next; // Update PC with the next instruction address
    end
end

endmodule