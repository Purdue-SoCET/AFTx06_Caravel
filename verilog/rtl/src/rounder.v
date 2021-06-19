module rounder (
	frm,
	sign,
	exp_in,
	fraction,
	round_out,
	rounded
);
	input [2:0] frm;
	input sign;
	input [7:0] exp_in;
	input [24:0] fraction;
	output reg [31:0] round_out;
	output rounded;
	reg round_amount;
	localparam RNE = 3'b000;
	localparam RZE = 3'b001;
	localparam RDN = 3'b010;
	localparam RUP = 3'b011;
	localparam RMM = 3'b100;
	always @(*) begin
		round_amount = 0;
		if (fraction[24:2] != {23 {1'sb1}})
			if (frm == RNE) begin
				if (fraction[1:0] == 2'b11)
					round_amount = 1;
			end
			else if (frm == RZE)
				round_amount = 0;
			else if (frm == RDN) begin
				if ((sign == 1) && ((fraction[0] == 1) || (fraction[1] == 1)))
					round_amount = 1;
			end
			else if (frm == RUP) begin
				if ((sign == 0) && ((fraction[0] == 1) || (fraction[1] == 1)))
					round_amount = 1;
			end
			else if (frm == RMM)
				if (fraction[1] == 1)
					round_amount = 1;
	end
	assign rounded = round_amount;
	wire [32:1] sv2v_tmp_2BA9D;
	assign sv2v_tmp_2BA9D = {sign, exp_in, fraction[24:2] + round_amount};
	always @(*) round_out = sv2v_tmp_2BA9D;
endmodule
