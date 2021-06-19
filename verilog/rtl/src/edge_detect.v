module edge_detect (
	clk,
	n_rst,
	SCL_sync,
	rising_edge,
	falling_edge
);
	input wire clk;
	input wire n_rst;
	input wire SCL_sync;
	output wire rising_edge;
	output wire falling_edge;
	reg stage1;
	reg stage2;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			stage1 <= 1'b0;
			stage2 <= 1'b0;
		end
		else begin
			stage1 <= SCL_sync;
			stage2 <= stage1;
		end
	assign rising_edge = stage1 & ~stage2;
	assign falling_edge = ~stage1 & stage2;
endmodule
