module shift_add_multiplier (
	CLK,
	nRST,
	multiplicand,
	multiplier,
	is_signed,
	start,
	product,
	finished
);
	parameter N = 32;
	input wire CLK;
	input wire nRST;
	input wire [N - 1:0] multiplicand;
	input wire [N - 1:0] multiplier;
	input wire [1:0] is_signed;
	input wire start;
	output reg [(N * 2) - 1:0] product;
	output reg finished;
	reg [(N * 2) - 1:0] multiplier_reg;
	reg [(N * 2) - 1:0] multiplicand_reg;
	wire [(N * 2) - 1:0] multiplier_ext;
	wire [(N * 2) - 1:0] multiplicand_ext;
	wire [(N * 2) - 1:0] partial_product;
	wire mult_complete;
	wire adjust_product;
	assign mult_complete = !(|multiplier_reg);
	assign adjust_product = (is_signed[0] & multiplier[N - 1]) ^ (is_signed[1] & multiplicand[N - 1]);
	assign partial_product = (multiplier_reg[0] ? multiplicand_reg : {N * 2 {1'sb0}});
	assign multiplier_ext = ~{{N {multiplier[N - 1]}}, multiplier} + 1;
	assign multiplicand_ext = ~{{N {multiplicand[N - 1]}}, multiplicand} + 1;
	always @(posedge CLK or negedge nRST)
		if (~nRST)
			finished <= 1'b0;
		else if (start)
			finished <= 1'b0;
		else if (mult_complete)
			finished <= 1'b1;
	always @(posedge CLK or negedge nRST)
		if (~nRST) begin
			multiplicand_reg <= {N * 2 {1'sb0}};
			multiplier_reg <= {N * 2 {1'sb0}};
			product <= {N * 2 {1'sb0}};
		end
		else if (start) begin
			multiplicand_reg <= (is_signed[1] && multiplicand[N - 1] ? multiplicand_ext : {{N {1'b0}}, multiplicand});
			multiplier_reg <= (is_signed[0] && multiplier[N - 1] ? multiplier_ext : {{N {1'b0}}, multiplier});
			product <= {N * 2 {1'sb0}};
		end
		else if (mult_complete & ~finished) begin
			multiplicand_reg <= multiplicand_reg;
			multiplier_reg <= multiplier_reg;
			product <= (adjust_product ? ~product + 1 : product);
		end
		else if (~finished) begin
			multiplicand_reg <= multiplicand_reg << 1;
			multiplier_reg <= multiplier_reg >> 1;
			product <= product + partial_product;
		end
endmodule
