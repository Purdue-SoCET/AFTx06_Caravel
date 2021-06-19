module flex_counter_reg (
	clk,
	n_rst,
	clear,
	save_count,
	revert_count,
	count_enable,
	rollover_val,
	count_out,
	rollover_flag
);
	parameter NUM_CNT_BITS = 4;
	input wire clk;
	input wire n_rst;
	input wire clear;
	input wire save_count;
	input wire revert_count;
	input wire count_enable;
	input wire [NUM_CNT_BITS - 1:0] rollover_val;
	output reg [NUM_CNT_BITS - 1:0] count_out;
	output reg rollover_flag;
	reg [NUM_CNT_BITS - 1:0] next_count;
	reg rollover;
	reg saved_count_out;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			count_out <= {NUM_CNT_BITS {1'sb0}};
			saved_count_out <= 1'b0;
			rollover_flag <= 1'b0;
		end
		else begin
			if (save_count)
				saved_count_out <= count_out;
			count_out <= next_count;
			rollover_flag <= rollover;
		end
	always @(*) begin
		if (~clear & (rollover_val == count_out))
			rollover = 1;
		else
			rollover = 0;
		if (clear == 1)
			next_count = 0;
		else if (revert_count)
			next_count = saved_count_out;
		else if (count_enable) begin
			if (rollover == 1'b0)
				next_count = count_out + 1'b1;
			else
				next_count = 1'b1;
		end
		else
			next_count = count_out;
	end
endmodule
