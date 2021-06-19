module flexPTP (
	data_in,
	clk,
	n_rst,
	shift_enable,
	data_out
);
	parameter NUM_BITS = 11;
	input reg [NUM_BITS:0] data_in;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	output reg [NUM_BITS:0] data_out;
	reg [NUM_BITS:0] next_data_out;
	always @(posedge clk or negedge n_rst)
		if (0 == n_rst)
			data_out <= {(NUM_BITS >= 0 ? NUM_BITS + 1 : 1 - NUM_BITS) {1'sb0}};
		else
			data_out <= next_data_out;
	always @(*)
		if (shift_enable)
			next_data_out = data_in;
		else
			next_data_out = data_out;
endmodule
