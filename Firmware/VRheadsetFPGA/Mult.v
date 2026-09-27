module fixedPointMult(
	input wire signed [15:0] a,
	input wire signed [15:0] b,
	output wire signed [31:0] result
);
assign result = a * b;
endmodule