module byte_counter (
	clk,
	n_rst,
	decrement,
	load_buffer,
	packet_length,
	zero,
	one
);
	parameter BITS = 6;
	input clk;
	input n_rst;
	input decrement;
	input load_buffer;
	input [BITS - 1:0] packet_length;
	output zero;
	output one;
	reg [BITS:0] count;
	reg [BITS:0] next_count;
	wire [BITS:0] load_value;
	assign load_value = (packet_length == 32'b00000000000000000000000000000000 ? 32'b00000000000000000000000000000001 << BITS : {1'b0, packet_length});
	always @(*)
		if (load_buffer)
			next_count = load_value;
		else if (decrement)
			next_count = count - 1;
		else
			next_count = count;
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			count <= 0;
		else
			count <= next_count;
	assign zero = count == 0;
	assign one = count == 1;
endmodule
