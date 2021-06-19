module ADD_step1 (
	floating_point1_in,
	floating_point2_in,
	sign_shifted,
	frac_shifted,
	sign_not_shifted,
	frac_not_shifted,
	exp_max
);
	input [31:0] floating_point1_in;
	input [31:0] floating_point2_in;
	output sign_shifted;
	output [25:0] frac_shifted;
	output sign_not_shifted;
	output [25:0] frac_not_shifted;
	output [7:0] exp_max;
	reg [7:0] unsigned_exp_diff;
	reg cmp_out;
	wire [31:0] floating_point_shift;
	wire [31:0] floating_point_not_shift;
	reg [31:0] shifted_floating_point;
	wire [8:1] sv2v_tmp_cmp_exponents_u_diff;
	always @(*) unsigned_exp_diff = sv2v_tmp_cmp_exponents_u_diff;
	wire [1:1] sv2v_tmp_cmp_exponents_cmp_out;
	always @(*) cmp_out = sv2v_tmp_cmp_exponents_cmp_out;
	int_compare cmp_exponents(
		.exp1(floating_point1_in[30:23]),
		.exp2(floating_point2_in[30:23]),
		.u_diff(sv2v_tmp_cmp_exponents_u_diff),
		.cmp_out(sv2v_tmp_cmp_exponents_cmp_out)
	);
	assign floating_point_shift = (cmp_out ? floating_point1_in : floating_point2_in);
	assign floating_point_not_shift = (cmp_out ? floating_point2_in : floating_point1_in);
	assign exp_max = (cmp_out ? floating_point2_in[30:23] : floating_point1_in[30:23]);
	right_shift shift_frac_with_smaller_exp(
		.fraction({1'b1, floating_point_shift[22:0], 2'b00}),
		.shift_amount(unsigned_exp_diff),
		.result(frac_shifted)
	);
	assign frac_not_shifted = {1'b1, floating_point_not_shift[22:0], 2'b00};
	assign sign_not_shifted = floating_point_not_shift[31];
	assign sign_shifted = floating_point_shift[31];
endmodule
