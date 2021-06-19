module flex_stp_sr (
	clk,
	n_rst,
	shift_enable,
	serial_in,
	parallel_out
);
	parameter NUM_BITS = 4;
	parameter SHIFT_MSB = 0;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	input wire serial_in;
	output reg [NUM_BITS - 1:0] parallel_out;
	integer i;
	reg [NUM_BITS - 1:0] next_state_logic;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			for (i = 0; i <= (NUM_BITS - 1); i = i + 1)
				parallel_out[i] <= 1'b1;
		end
		else
			parallel_out <= next_state_logic;
	always @(*)
		if (shift_enable == 1'b1) begin
			if (SHIFT_MSB == 0)
				next_state_logic = {serial_in, parallel_out[NUM_BITS - 1:1]};
			else
				next_state_logic = {parallel_out[NUM_BITS - 2:0], serial_in};
		end
		else
			next_state_logic = parallel_out;
endmodule
