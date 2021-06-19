module stp_sr_rx (
	clk,
	n_rst,
	SDA_sync,
	rising_edge,
	rx_enable,
	rx_data
);
	input wire clk;
	input wire n_rst;
	input wire SDA_sync;
	input wire rising_edge;
	input wire rx_enable;
	output reg [7:0] rx_data;
	wire [8:1] sv2v_tmp_STP_SR_RX_parallel_out;
	always @(*) rx_data = sv2v_tmp_STP_SR_RX_parallel_out;
	flex_stp_sr #(
		.NUM_BITS(8),
		.SHIFT_MSB(1)
	) STP_SR_RX(
		.clk(clk),
		.n_rst(n_rst),
		.shift_enable(rx_enable && rising_edge),
		.serial_in(SDA_sync),
		.parallel_out(sv2v_tmp_STP_SR_RX_parallel_out)
	);
endmodule
