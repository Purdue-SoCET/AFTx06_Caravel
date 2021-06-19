module pts_sr_tx (
	clk,
	n_rst,
	falling_edge,
	tx_enable,
	load_data,
	tx_data,
	tx_out
);
	input wire clk;
	input wire n_rst;
	input wire falling_edge;
	input wire tx_enable;
	input wire load_data;
	input wire [7:0] tx_data;
	output reg tx_out;
	wire [1:1] sv2v_tmp_PTS_SR_TX_serial_out;
	always @(*) tx_out = sv2v_tmp_PTS_SR_TX_serial_out;
	flex_pts_sr #(
		.NUM_BITS(8),
		.SHIFT_MSB(1)
	) PTS_SR_TX(
		.clk(clk),
		.n_rst(n_rst),
		.shift_enable(tx_enable && falling_edge),
		.load_enable(load_data),
		.parallel_in(tx_data),
		.serial_out(sv2v_tmp_PTS_SR_TX_serial_out)
	);
endmodule
