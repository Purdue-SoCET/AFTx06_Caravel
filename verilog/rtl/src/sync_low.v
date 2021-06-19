module sync_low (
	clk,
	n_rst,
	async_in,
	sync_out
);
	input wire clk;
	input wire n_rst;
	input wire async_in;
	output reg sync_out;
	reg sync;
	always @(posedge clk or negedge n_rst)
		if (1'b0 == n_rst) begin
			sync_out <= 1'b0;
			sync <= 1'b0;
		end
		else begin
			sync <= async_in;
			sync_out <= sync;
		end
endmodule
