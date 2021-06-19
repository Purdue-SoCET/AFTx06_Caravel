module p2p (
	data_in,
	clk,
	n_rst,
	shift_enable,
	data_out
);
	input reg [7:0] data_in;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	output reg [7:0] data_out;
	reg [7:0] next_data_out;
	always @(posedge clk or negedge n_rst)
		if (0 == n_rst)
			data_out <= 8'b00000000;
		else
			data_out <= next_data_out;
	always @(*)
		if (shift_enable)
			next_data_out = data_in;
		else
			next_data_out = data_out;
endmodule
