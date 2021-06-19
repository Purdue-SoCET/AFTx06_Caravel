module slave_inner (
	clk,
	n_rst,
	tx_data,
	address_mode,
	ms_select,
	bus_address,
	en_clock_strech,
	TX_fifo_empty,
	RX_fifo_full,
	RX_fifo_almost_full,
	SDA_sync,
	SCL_sync,
	rx_data_slave,
	set_transaction_complete_slave,
	ack_error_set_slave,
	busy_slave,
	TX_read_enable_slave,
	RX_write_enable_slave,
	SDA_out_slave,
	SCL_out_slave
);
	input wire clk;
	input wire n_rst;
	input wire [7:0] tx_data;
	input wire address_mode;
	input wire ms_select;
	input wire [9:0] bus_address;
	input wire en_clock_strech;
	input wire TX_fifo_empty;
	input wire RX_fifo_full;
	input wire RX_fifo_almost_full;
	input wire SDA_sync;
	input wire SCL_sync;
	output reg [7:0] rx_data_slave;
	output reg set_transaction_complete_slave;
	output reg ack_error_set_slave;
	output reg busy_slave;
	output reg TX_read_enable_slave;
	output reg RX_write_enable_slave;
	output reg SDA_out_slave;
	output reg SCL_out_slave;
	reg rising_edge;
	reg falling_edge;
	reg rx_enable;
	reg [7:0] rx_data;
	reg rw_mode;
	reg [1:0] address_match;
	reg start;
	reg stop;
	reg byte_received;
	reg ack_prep;
	reg ack_check;
	reg ack_done;
	reg [1:0] sda_mode;
	reg load_data;
	reg tx_enable;
	reg tx_out;
	reg rw_store;
	wire [1:1] sv2v_tmp_EDGE_DETECTOR_rising_edge;
	always @(*) rising_edge = sv2v_tmp_EDGE_DETECTOR_rising_edge;
	wire [1:1] sv2v_tmp_EDGE_DETECTOR_falling_edge;
	always @(*) falling_edge = sv2v_tmp_EDGE_DETECTOR_falling_edge;
	edge_detect EDGE_DETECTOR(
		.clk(clk),
		.n_rst(n_rst),
		.SCL_sync(SCL_sync),
		.rising_edge(sv2v_tmp_EDGE_DETECTOR_rising_edge),
		.falling_edge(sv2v_tmp_EDGE_DETECTOR_falling_edge)
	);
	wire [8:1] sv2v_tmp_STP_SR_RX_rx_data;
	always @(*) rx_data = sv2v_tmp_STP_SR_RX_rx_data;
	stp_sr_rx STP_SR_RX(
		.clk(clk),
		.n_rst(n_rst),
		.SDA_sync(SDA_sync),
		.rising_edge(rising_edge),
		.rx_enable(rx_enable),
		.rx_data(sv2v_tmp_STP_SR_RX_rx_data)
	);
	wire [8:1] sv2v_tmp_C9696;
	assign sv2v_tmp_C9696 = rx_data;
	always @(*) rx_data_slave = sv2v_tmp_C9696;
	wire [1:1] sv2v_tmp_CHECKER_BLOCK_rw_mode;
	always @(*) rw_mode = sv2v_tmp_CHECKER_BLOCK_rw_mode;
	wire [2:1] sv2v_tmp_CHECKER_BLOCK_address_match;
	always @(*) address_match = sv2v_tmp_CHECKER_BLOCK_address_match;
	wire [1:1] sv2v_tmp_CHECKER_BLOCK_start;
	always @(*) start = sv2v_tmp_CHECKER_BLOCK_start;
	wire [1:1] sv2v_tmp_CHECKER_BLOCK_stop;
	always @(*) stop = sv2v_tmp_CHECKER_BLOCK_stop;
	checker_block CHECKER_BLOCK(
		.clk(clk),
		.n_rst(n_rst),
		.SDA_sync(SDA_sync),
		.SCL_sync(SCL_sync),
		.rx_data(rx_data),
		.bus_address(bus_address),
		.address_mode(address_mode),
		.rw_store(rw_store),
		.rw_mode(sv2v_tmp_CHECKER_BLOCK_rw_mode),
		.address_match(sv2v_tmp_CHECKER_BLOCK_address_match),
		.start(sv2v_tmp_CHECKER_BLOCK_start),
		.stop(sv2v_tmp_CHECKER_BLOCK_stop)
	);
	wire [1:1] sv2v_tmp_AF438;
	assign sv2v_tmp_AF438 = stop;
	always @(*) set_transaction_complete_slave = sv2v_tmp_AF438;
	wire [1:1] sv2v_tmp_TIMER_byte_received;
	always @(*) byte_received = sv2v_tmp_TIMER_byte_received;
	wire [1:1] sv2v_tmp_TIMER_ack_prep;
	always @(*) ack_prep = sv2v_tmp_TIMER_ack_prep;
	wire [1:1] sv2v_tmp_TIMER_ack_check;
	always @(*) ack_check = sv2v_tmp_TIMER_ack_check;
	wire [1:1] sv2v_tmp_TIMER_ack_done;
	always @(*) ack_done = sv2v_tmp_TIMER_ack_done;
	slave_timer TIMER(
		.clk(clk),
		.n_rst(n_rst),
		.start(start),
		.stop(stop),
		.rising_edge(rising_edge),
		.falling_edge(falling_edge),
		.byte_received(sv2v_tmp_TIMER_byte_received),
		.ack_prep(sv2v_tmp_TIMER_ack_prep),
		.ack_check(sv2v_tmp_TIMER_ack_check),
		.ack_done(sv2v_tmp_TIMER_ack_done)
	);
	wire [1:1] sv2v_tmp_CONTROLLER_rx_enable;
	always @(*) rx_enable = sv2v_tmp_CONTROLLER_rx_enable;
	wire [1:1] sv2v_tmp_CONTROLLER_SCL_out_slave;
	always @(*) SCL_out_slave = sv2v_tmp_CONTROLLER_SCL_out_slave;
	wire [1:1] sv2v_tmp_CONTROLLER_busy_slave;
	always @(*) busy_slave = sv2v_tmp_CONTROLLER_busy_slave;
	wire [1:1] sv2v_tmp_CONTROLLER_TX_read_enable_slave;
	always @(*) TX_read_enable_slave = sv2v_tmp_CONTROLLER_TX_read_enable_slave;
	wire [1:1] sv2v_tmp_CONTROLLER_RX_write_enable_slave;
	always @(*) RX_write_enable_slave = sv2v_tmp_CONTROLLER_RX_write_enable_slave;
	wire [1:1] sv2v_tmp_CONTROLLER_ack_error_set_slave;
	always @(*) ack_error_set_slave = sv2v_tmp_CONTROLLER_ack_error_set_slave;
	wire [2:1] sv2v_tmp_CONTROLLER_sda_mode;
	always @(*) sda_mode = sv2v_tmp_CONTROLLER_sda_mode;
	wire [1:1] sv2v_tmp_CONTROLLER_load_data;
	always @(*) load_data = sv2v_tmp_CONTROLLER_load_data;
	wire [1:1] sv2v_tmp_CONTROLLER_tx_enable;
	always @(*) tx_enable = sv2v_tmp_CONTROLLER_tx_enable;
	wire [1:1] sv2v_tmp_CONTROLLER_rw_store;
	always @(*) rw_store = sv2v_tmp_CONTROLLER_rw_store;
	slave_controller CONTROLLER(
		.clk(clk),
		.n_rst(n_rst),
		.start(start),
		.stop(stop),
		.address_match(address_match),
		.rw_mode(rw_mode),
		.SDA_sync(SDA_sync),
		.address_mode(address_mode),
		.TX_fifo_empty(TX_fifo_empty),
		.RX_fifo_full(RX_fifo_full),
		.en_clock_strech(en_clock_strech),
		.RX_fifo_almost_full(RX_fifo_almost_full),
		.byte_received(byte_received),
		.ack_prep(ack_prep),
		.ack_check(ack_check),
		.ack_done(ack_done),
		.rx_enable(sv2v_tmp_CONTROLLER_rx_enable),
		.SCL_out_slave(sv2v_tmp_CONTROLLER_SCL_out_slave),
		.busy_slave(sv2v_tmp_CONTROLLER_busy_slave),
		.TX_read_enable_slave(sv2v_tmp_CONTROLLER_TX_read_enable_slave),
		.RX_write_enable_slave(sv2v_tmp_CONTROLLER_RX_write_enable_slave),
		.ack_error_set_slave(sv2v_tmp_CONTROLLER_ack_error_set_slave),
		.sda_mode(sv2v_tmp_CONTROLLER_sda_mode),
		.load_data(sv2v_tmp_CONTROLLER_load_data),
		.tx_enable(sv2v_tmp_CONTROLLER_tx_enable),
		.rw_store(sv2v_tmp_CONTROLLER_rw_store)
	);
	wire [1:1] sv2v_tmp_PTS_SR_TX_tx_out;
	always @(*) tx_out = sv2v_tmp_PTS_SR_TX_tx_out;
	pts_sr_tx PTS_SR_TX(
		.clk(clk),
		.n_rst(n_rst),
		.falling_edge(falling_edge),
		.tx_enable(tx_enable),
		.load_data(load_data),
		.tx_data(tx_data),
		.tx_out(sv2v_tmp_PTS_SR_TX_tx_out)
	);
	wire [1:1] sv2v_tmp_SDA_SECLECT_SDA_out_slave;
	always @(*) SDA_out_slave = sv2v_tmp_SDA_SECLECT_SDA_out_slave;
	sda_select SDA_SECLECT(
		.sda_mode(sda_mode),
		.tx_out(tx_out),
		.SDA_out_slave(sv2v_tmp_SDA_SECLECT_SDA_out_slave)
	);
endmodule
