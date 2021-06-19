module MUL_step2 (
	sign1,
	sign2,
	exp1,
	exp2,
	sign_out,
	sum_exp,
	ovf,
	unf,
	carry
);
	input sign1;
	input sign2;
	input [7:0] exp1;
	input [7:0] exp2;
	output sign_out;
	output [7:0] sum_exp;
	output reg ovf;
	output reg unf;
	input carry;
	wire [1:1] sv2v_tmp_add_EXPs_ovf;
	always @(*) ovf = sv2v_tmp_add_EXPs_ovf;
	wire [1:1] sv2v_tmp_add_EXPs_unf;
	always @(*) unf = sv2v_tmp_add_EXPs_unf;
	adder_8b add_EXPs(
		.carry(carry),
		.exp1(exp1),
		.exp2(exp2),
		.sum(sum_exp),
		.ovf(sv2v_tmp_add_EXPs_ovf),
		.unf(sv2v_tmp_add_EXPs_unf)
	);
	assign sign_out = sign1 ^ sign2;
endmodule
