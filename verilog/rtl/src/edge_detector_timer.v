module edge_detector_timer (
	clk,
	n_rst,
	signal,
	EDGEnA,
	EDGEnB,
	edge_detected
);
	parameter WIDTH = 8;
	input wire clk;
	input wire n_rst;
	input wire [WIDTH - 1:0] signal;
	input wire [WIDTH - 1:0] EDGEnA;
	input wire [WIDTH - 1:0] EDGEnB;
	output wire [WIDTH - 1:0] edge_detected;
	reg [WIDTH - 1:0] signal_r;
	wire [WIDTH - 1:0] pos_edge;
	wire [WIDTH - 1:0] neg_edge;
	always @(posedge clk or negedge n_rst)
		if (~n_rst)
			signal_r <= {WIDTH {1'sb0}};
		else
			signal_r <= signal;
	assign pos_edge = (signal & ~signal_r) & EDGEnA;
	assign neg_edge = (~signal & signal_r) & EDGEnB;
	assign edge_detected = pos_edge | neg_edge;
endmodule
