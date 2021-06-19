module u_to_s (
	sign,
	frac_unsigned,
	frac_signed
);
	input sign;
	input [25:0] frac_unsigned;
	output reg [26:0] frac_signed;
	always @(*) begin
		frac_signed = {1'b0, frac_unsigned};
		if (sign == 1)
			frac_signed = -frac_signed;
	end
endmodule
