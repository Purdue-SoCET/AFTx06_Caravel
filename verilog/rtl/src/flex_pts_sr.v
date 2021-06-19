module flex_pts_sr (
	clk,
	n_rst,
	shift_enable,
	load_enable,
	parallel_in,
	serial_out
);
	parameter NUM_BITS = 4;
	parameter SHIFT_MSB = 0;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	input wire load_enable;
	input reg [NUM_BITS - 1:0] parallel_in;
	output wire serial_out;
	reg [NUM_BITS - 1:0] output_logic;
	reg [NUM_BITS - 1:0] next_state_logic;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0)
			output_logic <= {NUM_BITS {1'sb1}};
		else
			output_logic <= next_state_logic;
	always @(*)
		if (load_enable == 1'b1)
			next_state_logic = parallel_in;
		else if (shift_enable == 1'b1) begin
			if (SHIFT_MSB == 1)
				next_state_logic = {output_logic[NUM_BITS - 2:0], 1'b1};
			else
				next_state_logic = {1'b1, output_logic[NUM_BITS - 1:1]};
		end
		else
			next_state_logic = output_logic;
	generate
		if (SHIFT_MSB == 1) begin
			assign serial_out = output_logic[NUM_BITS - 1];
		end
		else assign serial_out = output_logic[0];
	endgenerate
endmodule
