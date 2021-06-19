module int_compare (
	exp1,
	exp2,
	u_diff,
	cmp_out
);
	input [7:0] exp1;
	input [7:0] exp2;
	output [7:0] u_diff;
	output reg cmp_out;
	wire [8:0] u_exp1 = {1'b0, exp1};
	wire [8:0] u_exp2 = {1'b0, exp2};
	reg [8:0] diff;
	assign u_diff = diff[7:0];
	always @(*) begin
		diff = u_exp1 - u_exp2;
		case (diff[8])
			1'b0: cmp_out = 1'b0;
			1'b1: begin
				cmp_out = 1'b1;
				diff = -diff;
			end
		endcase
	end
endmodule
