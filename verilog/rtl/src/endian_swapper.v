module endian_swapper (
	word_in,
	word_out
);
	localparam rv32i_types_pkg_WORD_SIZE = 32;
	parameter N_BYTES = 4;
	parameter N_BITS = N_BYTES * 8;
	input [N_BITS - 1:0] word_in;
	output [N_BITS - 1:0] word_out;
	generate
		genvar i;
		for (i = 0; i < N_BYTES; i = i + 1) begin : word_assign
			assign word_out[(N_BITS - (8 * i)) - 1:N_BITS - (8 * (i + 1))] = word_in[((i + 1) * 8) - 1:i * 8];
		end
	endgenerate
endmodule
