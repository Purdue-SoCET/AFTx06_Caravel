module carry_save_adder (
	x,
	y,
	z,
	cout,
	sum
);
	parameter BIT_WIDTH = 32;
	input wire [BIT_WIDTH - 1:0] x;
	input wire [BIT_WIDTH - 1:0] y;
	input wire [BIT_WIDTH - 1:0] z;
	output wire [BIT_WIDTH - 1:0] cout;
	output wire [BIT_WIDTH - 1:0] sum;
	genvar i;
	wire [BIT_WIDTH - 1:0] c;
	generate
		for (i = 0; i < BIT_WIDTH; i = i + 1) full_adder FA(
			.x(x[i]),
			.y(y[i]),
			.cin(z[i]),
			.cout(c[i]),
			.sum(sum[i])
		);
	endgenerate
	assign cout = c << 1;
endmodule
