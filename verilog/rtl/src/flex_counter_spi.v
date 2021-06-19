module flex_counter_spi (
	clk,
	n_rst,
	clear,
	count_enable,
	rollover_val,
	count_out,
	rollover_flag
);
	parameter NUM_CNT_BITS = 4;
	input wire clk;
	input wire n_rst;
	input wire clear;
	input wire count_enable;
	input wire [NUM_CNT_BITS - 1:0] rollover_val;
	output wire [NUM_CNT_BITS - 1:0] count_out;
	output wire rollover_flag;
	reg rollover_flag_curr;
	reg rollover_flag_next;
	reg [NUM_CNT_BITS - 1:0] curr_count;
	reg [NUM_CNT_BITS - 1:0] next_count;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0) begin
			curr_count <= {NUM_CNT_BITS {1'sb0}};
			rollover_flag_curr <= 0;
		end
		else begin
			curr_count <= next_count;
			rollover_flag_curr <= rollover_flag_next;
		end
	always @(*)
		if (clear) begin
			next_count = 0;
			rollover_flag_next = 0;
		end
		else if (count_enable) begin
			next_count = curr_count + 1;
			rollover_flag_next = 0;
			if (curr_count == rollover_val)
				next_count = 1;
			if (next_count == rollover_val)
				rollover_flag_next = 1;
		end
		else begin
			next_count = curr_count;
			rollover_flag_next = (rollover_flag_curr ? 0 : rollover_flag_curr);
		end
	assign count_out = curr_count;
	assign rollover_flag = rollover_flag_curr;
endmodule
