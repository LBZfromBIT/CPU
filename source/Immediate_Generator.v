//immediate generate from instruction
module immediate_generator (
    input [31:0] instruction,
    output reg [31:0] immediate
);

wire [6:0] opcode = instruction[6:0];// opcode field of the instruction

always @(*) begin
    case (opcode)
        7'b0010011: // I-type
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b0000011: // Load 
            immediate = {{20{instruction[31]}}, instruction[31:20]};
        7'b0100011: // S-type (store)
            immediate = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        7'b1100011: // B-type (branch)
            immediate = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        7'b1101111: // J-type (jal)
            immediate = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        default:
            immediate = 32'h00000000;
    endcase
end

endmodule