module mul_26b (
	frac_in1,
	frac_in2,
	frac_out,
	overflow
);
	input [25:0] frac_in1;
	input [25:0] frac_in2;
	output [25:0] frac_out;
	output overflow;
	reg [51:0] frac_out_52b;
	assign overflow = frac_out_52b[51];
	assign frac_out = frac_out_52b[50:25];
	always @(*) begin : MULTIPLY
		frac_out_52b = frac_in1 * frac_in2;
	end
endmodule
