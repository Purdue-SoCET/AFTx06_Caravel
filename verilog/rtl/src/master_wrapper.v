module master_wrapper (
	clk,
	n_rst,
	tx_data,
	address_mode,
	data_direction,
	ms_select,
	bus_address,
	packet_size,
	en_clock_strech,
	TX_fifo_empty,
	RX_fifo_full,
	RX_fifo_almost_full,
	transaction_begin,
	SDA_sync,
	SCL_sync,
	clk_divider,
	rx_data_master,
	set_transaction_complete_master,
	set_arbitration_lost,
	ack_error_set_master,
	transaction_begin_clear,
	busy_master,
	TX_read_enable_master,
	RX_write_enable_master,
	SDA_out_master,
	SCL_out_master,
	line_busy
);
	input wire clk;
	input wire n_rst;
	input wire [7:0] tx_data;
	input wire address_mode;
	input wire data_direction;
	input wire ms_select;
	input wire [9:0] bus_address;
	input wire [7:0] packet_size;
	input wire en_clock_strech;
	input wire TX_fifo_empty;
	input wire RX_fifo_full;
	input wire RX_fifo_almost_full;
	input wire transaction_begin;
	input wire SDA_sync;
	input wire SCL_sync;
	input wire [31:0] clk_divider;
	output wire [7:0] rx_data_master;
	output wire set_transaction_complete_master;
	output wire set_arbitration_lost;
	output wire ack_error_set_master;
	output wire transaction_begin_clear;
	output wire busy_master;
	output wire TX_read_enable_master;
	output wire RX_write_enable_master;
	output wire SDA_out_master;
	output wire SCL_out_master;
	output wire line_busy;
	generate
		if (1) begin : bus
			wire [7:0] tx_data;
			wire [7:0] rx_data_slave;
			wire [7:0] rx_data_master;
			wire [7:0] rx_data;
			wire address_mode;
			wire data_direction;
			wire ms_select;
			wire [9:0] bus_address;
			wire [5:0] packet_size;
			wire en_clock_strech;
			wire TX_fifo_empty;
			wire RX_fifo_full;
			wire RX_fifo_almost_full;
			wire transaction_begin;
			wire [31:0] clk_divider;
			wire ack_error_set_slave;
			wire ack_error_set_master;
			wire ack_error_set;
			wire transaction_begin_clear;
			wire busy_slave;
			wire busy_master;
			wire busy;
			wire TX_read_enable_slave;
			wire TX_read_enable_master;
			wire TX_read_enable;
			wire RX_write_enable_slave;
			wire RX_write_enable_master;
			wire RX_write_enable;
			wire line_busy;
			wire set_transaction_complete;
			wire set_transaction_complete_slave;
			wire set_transaction_complete_master;
			wire set_arbitration_lost;
			wire SDA_sync;
			wire SCL_sync;
			wire SDA_out_slave;
			wire SDA_out_master;
			wire SCL_out_slave;
			wire SCL_out_master;
		end
	endgenerate
	localparam [0:0] MASTER = 0;
	generate
		if (1) begin : MASTER
			wire clk;
			wire n_rst;
			wire abort;
			wire ack_bit;
			wire ack_gen;
			wire bus_busy;
			wire byte_complete;
			wire decrement_byte_counter;
			wire load_buffers;
			wire output_wait_expired;
			wire one;
			wire shift_load;
			wire shift_strobe;
			wire should_nack;
			wire timer_active;
			wire zero;
			wire shift_direction;
			wire [1:0] output_select;
			wire [1:0] shift_input_select;
			wire SDA_sync;
			wire SCL_sync;
			wire SDA_out;
			wire SCL_out;
			assign SDA_sync = master_wrapper.bus.SDA_sync;
			assign SCL_sync = master_wrapper.bus.SCL_sync;
			wire SDA_out_shift_register;
			wire SDA_out_start_stop_gen;
			wire SCL_out_timer;
			wire SCL_out_start_stop_gen;
			wire [9:0] bus_address;
			wire [31:0] clock_div;
			wire clock_stretch_enabled;
			wire data_direction;
			wire address_mode;
			assign should_nack = (!master_wrapper.bus.en_clock_strech && master_wrapper.bus.RX_fifo_almost_full) || zero;
			assign master_wrapper.bus.line_busy = bus_busy;
			assign master_wrapper.bus.SDA_out_master = SDA_out;
			assign master_wrapper.bus.SCL_out_master = SCL_out;
			wire start;
			wire stop;
			master_controller MASTER_CONTROLLER(
				.clk(clk),
				.n_rst(n_rst),
				.address_mode(address_mode),
				.ms_select(master_wrapper.bus.ms_select),
				.bus_busy(bus_busy),
				.begin_transaction_flag(master_wrapper.bus.transaction_begin),
				.ack_bit(ack_bit),
				.data_direction(data_direction),
				.output_wait_expired(output_wait_expired),
				.byte_complete(byte_complete),
				.zero_bytes_left(zero),
				.abort(abort),
				.stretch_enabled(clock_stretch_enabled),
				.rx_fifo_full(master_wrapper.bus.RX_fifo_full),
				.tx_fifo_empty(master_wrapper.bus.TX_fifo_empty),
				.shift_input_select(shift_input_select),
				.output_select(output_select),
				.shift_direction(shift_direction),
				.shift_load(shift_load),
				.timer_active(timer_active),
				.load_buffers(load_buffers),
				.decrement_byte_counter(decrement_byte_counter),
				.set_ack_error(master_wrapper.bus.ack_error_set_master),
				.set_arbitration_lost(master_wrapper.bus.set_arbitration_lost),
				.clear_transaction_begin(master_wrapper.bus.transaction_begin_clear),
				.start(start),
				.stop(stop),
				.tx_fifo_enable(master_wrapper.bus.TX_read_enable_master),
				.rx_fifo_enable(master_wrapper.bus.RX_write_enable_master),
				.busy(master_wrapper.bus.busy_master),
				.set_transaction_complete(master_wrapper.bus.set_transaction_complete_master)
			);
			master_timer TIMER(
				.clk(clk),
				.n_rst(n_rst),
				.timer_active(timer_active),
				.direction(shift_direction),
				.should_nack(should_nack),
				.SDA_sync(SDA_sync),
				.SCL_sync(SCL_sync),
				.SDA_out(SDA_out_shift_register),
				.clock_div(clock_div),
				.SCL_out(SCL_out_timer),
				.shift_strobe(shift_strobe),
				.byte_complete(byte_complete),
				.ack_gen(ack_gen),
				.ack(ack_bit),
				.abort(abort)
			);
			control_buffer CONTROL_BUFFER(
				.clk(clk),
				.n_rst(n_rst),
				.u_bus_address(master_wrapper.bus.bus_address),
				.u_data_direction(master_wrapper.bus.data_direction),
				.u_address_mode(master_wrapper.bus.address_mode),
				.u_clock_div(master_wrapper.bus.clk_divider),
				.u_stretch_enabled(master_wrapper.bus.en_clock_strech),
				.load_buffer(load_buffers),
				.bus_address(bus_address),
				.data_direction(data_direction),
				.address_mode(address_mode),
				.stretch_enabled(clock_stretch_enabled),
				.clock_div(clock_div)
			);
			shift_register SHIFT_REGISTER(
				.clk(clk),
				.n_rst(n_rst),
				.bus_address(bus_address),
				.tx_data(master_wrapper.bus.tx_data),
				.shift_input_select(shift_input_select),
				.data_direction(data_direction),
				.shift_direction(shift_direction),
				.shift_in(SDA_sync),
				.shift_load(shift_load),
				.shift_strobe(shift_strobe),
				.shift_out(SDA_out_shift_register),
				.data_out(master_wrapper.bus.rx_data_master)
			);
			start_stop_gen START_STOP_GEN(
				.clk(clk),
				.n_rst(n_rst),
				.start(start),
				.stop(stop),
				.clock_div(clock_div),
				.bus_busy(bus_busy),
				.SDA(SDA_out_start_stop_gen),
				.SCL(SCL_out_start_stop_gen),
				.done(output_wait_expired)
			);
			output_mux OUTPUT_MUX(
				.drive_select(output_select),
				.tx_SDA(SDA_out_shift_register),
				.tx_SCL(SCL_out_timer),
				.rx_SDA(!ack_gen),
				.rx_SCL(SCL_out_timer),
				.start_stop_SDA(SDA_out_start_stop_gen),
				.start_stop_SCL(SCL_out_start_stop_gen),
				.SDA_out(SDA_out),
				.SCL_out(SCL_out)
			);
			busy_tester BUSY_TESTER(
				.clk(clk),
				.n_rst(n_rst),
				.SDA_sync(SDA_sync),
				.SCL_sync(SCL_sync),
				.bus_busy(bus_busy)
			);
			byte_counter BYTE_COUNTER(
				.clk(clk),
				.n_rst(n_rst),
				.decrement(decrement_byte_counter),
				.load_buffer(load_buffers),
				.packet_length(master_wrapper.bus.packet_size[5:0]),
				.zero(zero),
				.one(one)
			);
		end
		assign MASTER.clk = clk;
		assign MASTER.n_rst = n_rst;
	endgenerate
	assign bus.tx_data = tx_data;
	assign bus.address_mode = address_mode;
	assign bus.data_direction = data_direction;
	assign bus.ms_select = ms_select;
	assign bus.bus_address = bus_address;
	assign bus.packet_size = packet_size;
	assign bus.en_clock_strech = en_clock_strech;
	assign bus.TX_fifo_empty = TX_fifo_empty;
	assign bus.RX_fifo_full = RX_fifo_full;
	assign bus.RX_fifo_almost_full = RX_fifo_almost_full;
	assign bus.transaction_begin = transaction_begin;
	assign bus.SDA_sync = SDA_sync;
	assign bus.SCL_sync = SCL_sync;
	assign bus.clk_divider = clk_divider;
	assign rx_data_master = bus.rx_data_master;
	assign set_transaction_complete_master = bus.set_transaction_complete_master;
	assign set_arbitration_lost = bus.set_arbitration_lost;
	assign ack_error_set_master = bus.ack_error_set_master;
	assign transaction_begin_clear = bus.transaction_begin_clear;
	assign busy_master = bus.busy_master;
	assign TX_read_enable_master = bus.TX_read_enable_master;
	assign RX_write_enable_master = bus.RX_write_enable_master;
	assign SDA_out_master = bus.SDA_out_master;
	assign SCL_out_master = bus.SCL_out_master;
	assign line_busy = bus.line_busy;
endmodule
