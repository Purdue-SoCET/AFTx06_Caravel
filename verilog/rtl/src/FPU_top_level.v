module FPU_top_level (
	clk,
	nrst,
	floating_point1,
	floating_point2,
	frm,
	funct7,
	floating_point_out,
	flags
);
	input clk;
	input nrst;
	input [31:0] floating_point1;
	input [31:0] floating_point2;
	input [2:0] frm;
	input [6:0] funct7;
	output [31:0] floating_point_out;
	output [4:0] flags;
	reg [2:0] frm2;
	reg [2:0] frm3;
	reg [6:0] funct7_2;
	reg [6:0] funct7_3;
	localparam ADD = 7'b0100000;
	localparam MUL = 7'b0000010;
	reg sign_shifted;
	reg [25:0] frac_shifted;
	reg sign_not_shifted;
	reg [25:0] frac_not_shifted;
	reg [7:0] exp_max;
	reg mul_sign1;
	reg mul_sign2;
	reg [7:0] mul_exp1;
	reg [7:0] mul_exp2;
	reg [25:0] product;
	reg mul_carry_out;
	reg [61:0] step1_to_step2;
	reg [61:0] nxt_step1_to_step2;
	reg add_sign_out;
	reg [25:0] add_sum;
	reg add_carry_out;
	reg [7:0] add_exp_max;
	reg mul_sign_out;
	reg [7:0] sum_exp;
	reg mul_ovf;
	reg mul_unf;
	reg inv;
	reg inv2;
	reg inv3;
	reg [37:0] step2_to_step3;
	reg [37:0] nxt_step2_to_step3;
	wire [1:1] sv2v_tmp_addStep1_sign_shifted;
	always @(*) sign_shifted = sv2v_tmp_addStep1_sign_shifted;
	wire [26:1] sv2v_tmp_addStep1_frac_shifted;
	always @(*) frac_shifted = sv2v_tmp_addStep1_frac_shifted;
	wire [1:1] sv2v_tmp_addStep1_sign_not_shifted;
	always @(*) sign_not_shifted = sv2v_tmp_addStep1_sign_not_shifted;
	wire [26:1] sv2v_tmp_addStep1_frac_not_shifted;
	always @(*) frac_not_shifted = sv2v_tmp_addStep1_frac_not_shifted;
	wire [8:1] sv2v_tmp_addStep1_exp_max;
	always @(*) exp_max = sv2v_tmp_addStep1_exp_max;
	ADD_step1 addStep1(
		.floating_point1_in(floating_point1),
		.floating_point2_in(floating_point2),
		.sign_shifted(sv2v_tmp_addStep1_sign_shifted),
		.frac_shifted(sv2v_tmp_addStep1_frac_shifted),
		.sign_not_shifted(sv2v_tmp_addStep1_sign_not_shifted),
		.frac_not_shifted(sv2v_tmp_addStep1_frac_not_shifted),
		.exp_max(sv2v_tmp_addStep1_exp_max)
	);
	wire [1:1] sv2v_tmp_mulStep1_sign1;
	always @(*) mul_sign1 = sv2v_tmp_mulStep1_sign1;
	wire [1:1] sv2v_tmp_mulStep1_sign2;
	always @(*) mul_sign2 = sv2v_tmp_mulStep1_sign2;
	wire [8:1] sv2v_tmp_mulStep1_exp1;
	always @(*) mul_exp1 = sv2v_tmp_mulStep1_exp1;
	wire [8:1] sv2v_tmp_mulStep1_exp2;
	always @(*) mul_exp2 = sv2v_tmp_mulStep1_exp2;
	wire [26:1] sv2v_tmp_mulStep1_product;
	always @(*) product = sv2v_tmp_mulStep1_product;
	wire [1:1] sv2v_tmp_mulStep1_carry_out;
	always @(*) mul_carry_out = sv2v_tmp_mulStep1_carry_out;
	MUL_step1 mulStep1(
		.fp1_in(floating_point1),
		.fp2_in(floating_point2),
		.sign1(sv2v_tmp_mulStep1_sign1),
		.sign2(sv2v_tmp_mulStep1_sign2),
		.exp1(sv2v_tmp_mulStep1_exp1),
		.exp2(sv2v_tmp_mulStep1_exp2),
		.product(sv2v_tmp_mulStep1_product),
		.carry_out(sv2v_tmp_mulStep1_carry_out)
	);
	always @(*) begin : check_for_invalid_op
		inv = 0;
		if (funct7 == ADD)
			if (((floating_point1[30:0] == 31'h7f800000) && (floating_point2[30:0] == 31'h7f800000)) && (floating_point1[31] ^ floating_point2[31]))
				inv = 1;
		if (funct7 == MUL)
			if (((floating_point1[30:0] == 31'h00000000) && (floating_point2[30:0] == 31'h7f800000)) || ((floating_point1[30:0] == 31'h7f800000) && (floating_point2[30:0] == 31'h00000000)))
				inv = 1;
	end
	always @(*) begin : select_op_step1to2
		case (funct7)
			ADD: begin
				nxt_step1_to_step2[61] = sign_shifted;
				nxt_step1_to_step2[60:35] = frac_shifted;
				nxt_step1_to_step2[34] = sign_not_shifted;
				nxt_step1_to_step2[33:8] = frac_not_shifted;
				nxt_step1_to_step2[7:0] = exp_max;
			end
			MUL: begin
				nxt_step1_to_step2[61] = mul_sign1;
				nxt_step1_to_step2[60] = mul_sign2;
				nxt_step1_to_step2[59:52] = mul_exp1;
				nxt_step1_to_step2[51:44] = mul_exp2;
				nxt_step1_to_step2[43:18] = product;
				nxt_step1_to_step2[17] = mul_carry_out;
			end
		endcase
	end
	always @(posedge clk or negedge nrst) begin : STEP1_to_STEP2
		if (nrst == 0) begin
			frm2 <= 0;
			step1_to_step2 <= 0;
			funct7_2 <= 0;
			inv2 <= 0;
		end
		else begin
			frm2 <= frm;
			step1_to_step2 <= nxt_step1_to_step2;
			funct7_2 <= funct7;
			inv2 <= inv;
		end
	end
	wire [1:1] sv2v_tmp_add_step2_sign_out;
	always @(*) add_sign_out = sv2v_tmp_add_step2_sign_out;
	wire [26:1] sv2v_tmp_add_step2_sum;
	always @(*) add_sum = sv2v_tmp_add_step2_sum;
	wire [1:1] sv2v_tmp_add_step2_carry_out;
	always @(*) add_carry_out = sv2v_tmp_add_step2_carry_out;
	wire [8:1] sv2v_tmp_add_step2_exp_max_out;
	always @(*) add_exp_max = sv2v_tmp_add_step2_exp_max_out;
	ADD_step2 add_step2(
		.frac1(step1_to_step2[60:35]),
		.sign1(step1_to_step2[61]),
		.frac2(step1_to_step2[33:8]),
		.sign2(step1_to_step2[34]),
		.exp_max_in(step1_to_step2[7:0]),
		.sign_out(sv2v_tmp_add_step2_sign_out),
		.sum(sv2v_tmp_add_step2_sum),
		.carry_out(sv2v_tmp_add_step2_carry_out),
		.exp_max_out(sv2v_tmp_add_step2_exp_max_out)
	);
	wire [1:1] sv2v_tmp_mul_step2_sign_out;
	always @(*) mul_sign_out = sv2v_tmp_mul_step2_sign_out;
	wire [8:1] sv2v_tmp_mul_step2_sum_exp;
	always @(*) sum_exp = sv2v_tmp_mul_step2_sum_exp;
	wire [1:1] sv2v_tmp_mul_step2_ovf;
	always @(*) mul_ovf = sv2v_tmp_mul_step2_ovf;
	wire [1:1] sv2v_tmp_mul_step2_unf;
	always @(*) mul_unf = sv2v_tmp_mul_step2_unf;
	MUL_step2 mul_step2(
		.sign1(step1_to_step2[61]),
		.sign2(step1_to_step2[60]),
		.exp1(step1_to_step2[59:52]),
		.exp2(step1_to_step2[51:44]),
		.sign_out(sv2v_tmp_mul_step2_sign_out),
		.sum_exp(sv2v_tmp_mul_step2_sum_exp),
		.ovf(sv2v_tmp_mul_step2_ovf),
		.unf(sv2v_tmp_mul_step2_unf),
		.carry(step1_to_step2[17])
	);
	always @(*) begin : select_op_step2to3
		case (funct7_2)
			ADD: begin
				nxt_step2_to_step3[37:36] = 2'b00;
				nxt_step2_to_step3[35] = add_sign_out;
				nxt_step2_to_step3[34:9] = add_sum;
				nxt_step2_to_step3[8] = add_carry_out;
				nxt_step2_to_step3[7:0] = add_exp_max;
			end
			MUL: begin
				nxt_step2_to_step3[37] = mul_ovf;
				nxt_step2_to_step3[36] = mul_unf;
				nxt_step2_to_step3[35] = mul_sign_out;
				nxt_step2_to_step3[34:9] = step1_to_step2[43:18];
				nxt_step2_to_step3[8] = step1_to_step2[17];
				nxt_step2_to_step3[7:0] = sum_exp;
			end
		endcase
	end
	always @(posedge clk or negedge nrst) begin : STEP2_to_STEP3
		if (nrst == 0) begin
			funct7_3 <= 0;
			step2_to_step3 <= 0;
			frm3 <= 0;
			inv3 <= 0;
		end
		else begin
			funct7_3 <= funct7_2;
			step2_to_step3 <= nxt_step2_to_step3;
			frm3 <= frm2;
			inv3 <= inv2;
		end
	end
	reg o;
	always @(*)
		if (((step2_to_step3[7:0] == 8'b11111111) && (step2_to_step3[36] == 1'b0)) && (step2_to_step3[8] == 0))
			o = 1;
		else
			o = step2_to_step3[37];
	ADD_step3 step3(
		.ovf_in(o),
		.unf_in(step2_to_step3[36]),
		.dz(1'b0),
		.inv(inv3),
		.frm(frm3),
		.exponent_max_in(step2_to_step3[7:0]),
		.sign_in(step2_to_step3[35]),
		.frac_in(step2_to_step3[34:9]),
		.carry_out(step2_to_step3[8]),
		.floating_point_out(floating_point_out),
		.flags(flags)
	);
endmodule
