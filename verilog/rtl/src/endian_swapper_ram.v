module endian_swapper_ram (
	word_in,
	word_out
);
	input wire [31:0] word_in;
	output wire [31:0] word_out;
	parameter WORD_SIZE = 32;
	generate
		genvar i;
		for (i = 0; i < (WORD_SIZE / 8); i = i + 1) begin : word_assign
			assign word_out[(WORD_SIZE - (8 * i)) - 1:WORD_SIZE - (8 * (i + 1))] = word_in[((i + 1) * 8) - 1:i * 8];
		end
	endgenerate
endmodule
