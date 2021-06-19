module MUL_step1 (
	fp1_in,
	fp2_in,
	sign1,
	sign2,
	exp1,
	exp2,
	product,
	carry_out
);
	input [31:0] fp1_in;
	input [31:0] fp2_in;
	output sign1;
	output sign2;
	output [7:0] exp1;
	output [7:0] exp2;
	output [25:0] product;
	output carry_out;
	assign sign1 = fp1_in[31];
	assign sign2 = fp2_in[31];
	assign exp1 = fp1_in[30:23];
	assign exp2 = fp2_in[30:23];
	mul_26b MUL(
		.frac_in1({1'b1, fp1_in[22:0], 2'b00}),
		.frac_in2({1'b1, fp2_in[22:0], 2'b00}),
		.frac_out(product),
		.overflow(carry_out)
	);
endmodule
