module interruptLogic (
	clk,
	n_rst,
	status,
	interrupt
);
	input wire clk;
	input wire n_rst;
	input wire [12:0] status;
	output reg interrupt;
	reg next_interrupt;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			interrupt <= 1'b0;
		else
			interrupt <= next_interrupt;
	always @(*)
		if ((((status[0] | status[2]) | status[3]) | status[5]) | status[6])
			next_interrupt = 1'b1;
		else
			next_interrupt = 1'b0;
endmodule
