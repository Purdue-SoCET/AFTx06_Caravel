module edge_detector (
	clk,
	n_rst,
	signal,
	pos_edge,
	neg_edge
);
	parameter WIDTH = 1;
	input wire clk;
	input wire n_rst;
	input wire [WIDTH - 1:0] signal;
	output wire [WIDTH - 1:0] pos_edge;
	output wire [WIDTH - 1:0] neg_edge;
	reg [WIDTH - 1:0] signal_r;
	always @(posedge clk or negedge n_rst)
		if (~n_rst)
			signal_r <= {WIDTH {1'sb0}};
		else
			signal_r <= signal;
	assign pos_edge = signal & ~signal_r;
	assign neg_edge = ~signal & signal_r;
endmodule
