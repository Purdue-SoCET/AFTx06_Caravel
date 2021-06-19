module controlReg (
	data_in,
	clk,
	n_rst,
	shift_enable,
	clear_begin_trans,
	data_out
);
	parameter MAX_BIT = 11;
	input reg [MAX_BIT:0] data_in;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	input wire clear_begin_trans;
	output reg [MAX_BIT:0] data_out;
	reg [MAX_BIT:0] next_data_out;
	always @(posedge clk or negedge n_rst)
		if (0 == n_rst)
			data_out <= {(MAX_BIT >= 0 ? MAX_BIT + 1 : 1 - MAX_BIT) {1'sb0}};
		else
			data_out <= next_data_out;
	always @(*) begin
		next_data_out = data_out;
		if (shift_enable)
			next_data_out = data_in;
		else begin
			next_data_out[11] = 0;
			if (clear_begin_trans)
				next_data_out[9] = 0;
		end
	end
endmodule
