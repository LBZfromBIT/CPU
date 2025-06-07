module alu (
    input [31:0] a,          // op A
    input [31:0] b,          // op B
    input [3:0] alu_control, // ALU control signal
    output reg [31:0] result,// ALU result
    output zero              // zero flag
);

parameter ADD  = 4'b0000;
parameter SUB  = 4'b0001;
parameter AND  = 4'b0010;
parameter OR   = 4'b0011;
parameter XOR  = 4'b0100;
parameter SLT  = 4'b0101;  // set less than
parameter SLTU = 4'b0110;  // set less than unsigned

//All shift oprations use b[4:0] as shift amount
parameter SLL  = 4'b0111;  // shift left logical
parameter SRL  = 4'b1000;  // shift right logical
parameter SRA  = 4'b1001;  // shift right arithmetic

always @(*) begin
    case (alu_control)
        ADD:  result = a + b;
        SUB:  result = a - b;
        AND:  result = a & b;
        OR:   result = a | b;
        XOR:  result = a ^ b;
        SLT:  result = ($signed(a) < $signed(b)) ? 32'h00000001 : 32'h00000000;
        SLTU: result = (a < b) ? 32'h00000001 : 32'h00000000;
        SLL:  result = a << b[4:0];
        SRL:  result = a >> b[4:0];
        SRA:  result = $signed(a) >>> b[4:0];
        default: result = 32'h00000000;
    endcase
end

assign zero = (result == 32'h00000000);

endmodule