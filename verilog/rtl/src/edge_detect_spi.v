module edge_detect_spi (
	clk,
	n_rst,
	edge_mode,
	d_plus,
	d_edge
);
	input wire clk;
	input wire n_rst;
	input wire edge_mode;
	input wire d_plus;
	output wire d_edge;
	reg dprev;
	reg dcurr;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0) begin
			dprev <= 1;
			dcurr <= 1;
		end
		else begin
			dprev <= dcurr;
			dcurr <= d_plus;
		end
	assign d_edge = (edge_mode ? ~dprev & dcurr : dprev & ~dcurr);
endmodule
