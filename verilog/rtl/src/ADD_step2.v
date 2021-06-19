module ADD_step2 (
	frac1,
	sign1,
	frac2,
	sign2,
	exp_max_in,
	sign_out,
	sum,
	carry_out,
	exp_max_out
);
	input [25:0] frac1;
	input sign1;
	input [25:0] frac2;
	input sign2;
	input [7:0] exp_max_in;
	output sign_out;
	output [25:0] sum;
	output carry_out;
	output reg [7:0] exp_max_out;
	reg [26:0] frac1_signed;
	reg [26:0] frac2_signed;
	reg [26:0] sum_signed;
	always @(*) begin : exp_max_assignment
		if (sum_signed == 0)
			exp_max_out = 8'b00000000;
		else
			exp_max_out = exp_max_in;
	end
	wire [27:1] sv2v_tmp_change_to_signed1_frac_signed;
	always @(*) frac1_signed = sv2v_tmp_change_to_signed1_frac_signed;
	u_to_s change_to_signed1(
		.sign(sign1),
		.frac_unsigned(frac1),
		.frac_signed(sv2v_tmp_change_to_signed1_frac_signed)
	);
	wire [27:1] sv2v_tmp_change_to_signed2_frac_signed;
	always @(*) frac2_signed = sv2v_tmp_change_to_signed2_frac_signed;
	u_to_s change_to_signed2(
		.sign(sign2),
		.frac_unsigned(frac2),
		.frac_signed(sv2v_tmp_change_to_signed2_frac_signed)
	);
	wire [27:1] sv2v_tmp_add_signed_fracs_sum;
	always @(*) sum_signed = sv2v_tmp_add_signed_fracs_sum;
	adder_26b add_signed_fracs(
		.frac1(frac1_signed),
		.frac2(frac2_signed),
		.sum(sv2v_tmp_add_signed_fracs_sum),
		.ovf(carry_out)
	);
	s_to_u change_to_unsigned(
		.frac_signed(sum_signed),
		.sign(sign_out),
		.frac_unsigned(sum)
	);
endmodule
