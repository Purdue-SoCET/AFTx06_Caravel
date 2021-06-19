module flex_counter_master (
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
	output reg [NUM_CNT_BITS - 1:0] count_out;
	output reg rollover_flag;
	reg [NUM_CNT_BITS - 1:0] incremented_count;
	reg [NUM_CNT_BITS:0] carrys;
	wire [NUM_CNT_BITS - 1:0] pre_rollover = (rollover_val == 1 ? 1 : rollover_val - 1);
	wire [1:1] sv2v_tmp_3ABDA;
	assign sv2v_tmp_3ABDA = 1'b1;
	always @(*) carrys[0] = sv2v_tmp_3ABDA;
	genvar i;
	generate
		for (i = 0; i < NUM_CNT_BITS; i = i + 1) begin
			wire [1:1] sv2v_tmp_9B198;
			assign sv2v_tmp_9B198 = carrys[i] ^ count_out[i];
			always @(*) incremented_count[i] = sv2v_tmp_9B198;
			wire [1:1] sv2v_tmp_E201A;
			assign sv2v_tmp_E201A = carrys[i] & count_out[i];
			always @(*) carrys[i + 1] = sv2v_tmp_E201A;
		end
	endgenerate
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			rollover_flag <= 0;
		else if (count_enable && !clear) begin
			if (count_out == pre_rollover)
				rollover_flag <= 1'b1;
			else
				rollover_flag <= 1'b0;
		end
		else if (clear)
			rollover_flag <= 0;
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			count_out <= 0;
		else if (clear)
			count_out <= 0;
		else if (count_enable) begin
			if (count_out == rollover_val)
				count_out <= 1;
			else
				count_out <= incremented_count;
		end
		else
			count_out <= count_out;
endmodule
