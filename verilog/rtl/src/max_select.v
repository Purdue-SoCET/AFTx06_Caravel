module max_select (
	exp1,
	exp2,
	max
);
	input [7:0] exp1;
	input [7:0] exp2;
	output reg [7:0] max;
	reg [8:0] u_exp1;
	reg [8:0] u_exp2;
	reg [8:0] diff;
	wire [9:1] sv2v_tmp_B2FF4;
	assign sv2v_tmp_B2FF4 = {1'b0, exp1};
	always @(*) u_exp1 = sv2v_tmp_B2FF4;
	wire [9:1] sv2v_tmp_D9199;
	assign sv2v_tmp_D9199 = {1'b0, exp2};
	always @(*) u_exp2 = sv2v_tmp_D9199;
	wire [9:1] sv2v_tmp_19057;
	assign sv2v_tmp_19057 = u_exp1 - u_exp2;
	always @(*) diff = sv2v_tmp_19057;
	always @(*)
		if (diff[8] == 0)
			max = exp1;
		else
			max = exp2;
endmodule
