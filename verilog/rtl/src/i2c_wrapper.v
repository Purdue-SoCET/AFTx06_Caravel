module i2c_wrapper (
	clk,
	n_rst,
	SDA,
	SCL,
	PWRITE,
	PENABLE,
	PSEL,
	PWDATA,
	PADDR,
	PRDATA,
	SDA_out,
	SCL_out,
	interrupt
);
	input clk;
	input n_rst;
	input SDA;
	input SCL;
	input PWRITE;
	input PENABLE;
	input PSEL;
	input [31:0] PWDATA;
	input [31:0] PADDR;
	output [31:0] PRDATA;
	output SDA_out;
	output SCL_out;
	output interrupt;
	generate
		if (1) begin : i2c
			wire SDA;
			wire SCL;
			reg SDA_out;
			reg SCL_out;
			wire interrupt;
		end
	endgenerate
	generate
		if (1) begin : apb_s
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	assign i2c.SDA = SDA;
	assign i2c.SCL = SCL;
	assign SDA_out = i2c.SDA_out;
	assign SCL_out = i2c.SCL_out;
	assign interrupt = i2c.interrupt;
	assign apb_s.PENABLE = PENABLE;
	assign apb_s.PSEL = PSEL;
	assign apb_s.PWRITE = PWRITE;
	assign PRDATA = apb_s.PRDATA;
	assign apb_s.PWDATA = PWDATA;
	assign apb_s.PADDR = PADDR;
	localparam [0:0] MASTER = 0;
	localparam [0:0] SLAVE = 1;
	generate
		if (1) begin : IX
			wire clk;
			wire n_rst;
			wire master_active;
			wire master_intermediate;
			if (1) begin : bus
				wire [7:0] tx_data;
				wire [7:0] rx_data_slave;
				wire [7:0] rx_data_master;
				reg [7:0] rx_data;
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
				reg ack_error_set;
				wire transaction_begin_clear;
				wire busy_slave;
				wire busy_master;
				reg busy;
				wire TX_read_enable_slave;
				wire TX_read_enable_master;
				reg TX_read_enable;
				wire RX_write_enable_slave;
				wire RX_write_enable_master;
				reg RX_write_enable;
				wire line_busy;
				reg set_transaction_complete;
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
			sync_high SDA_SYNC(
				.clk(clk),
				.n_rst(n_rst),
				.async_in(i2c_wrapper.i2c.SDA),
				.sync_out(bus.SDA_sync)
			);
			sync_high SCL_SYNC(
				.clk(clk),
				.n_rst(n_rst),
				.async_in(i2c_wrapper.i2c.SCL),
				.sync_out(bus.SCL_sync)
			);
			assign master_intermediate = bus.ms_select;
			assign master_active = (master_intermediate == MASTER ? 1'b1 : 1'b0);
			if (1) begin : REGISTER
				wire clk;
				wire n_rst;
				wire interrupt;
				apbSlave IX(
					.pclk(clk),
					.n_rst(n_rst),
					.pdata(i2c_wrapper.apb_s.PWDATA),
					.paddr(i2c_wrapper.apb_s.PADDR),
					.penable(i2c_wrapper.apb_s.PENABLE),
					.psel(i2c_wrapper.apb_s.PSEL),
					.pwrite(i2c_wrapper.apb_s.PWRITE),
					.rx_data(i2c_wrapper.IX.bus.rx_data),
					.rx_w_ena(i2c_wrapper.IX.bus.RX_write_enable),
					.i2c_status({i2c_wrapper.IX.bus.transaction_begin_clear, i2c_wrapper.IX.bus.busy, i2c_wrapper.IX.bus.set_arbitration_lost, i2c_wrapper.IX.bus.set_transaction_complete, i2c_wrapper.IX.bus.line_busy, i2c_wrapper.IX.bus.ack_error_set}),
					.scl(clk),
					.tx_r_ena(i2c_wrapper.IX.bus.TX_read_enable),
					.prdata(i2c_wrapper.apb_s.PRDATA),
					.i2c_interrupt(interrupt),
					.tx_data(i2c_wrapper.IX.bus.tx_data),
					.rx_full(i2c_wrapper.IX.bus.RX_fifo_full),
					.rx_almost_full(i2c_wrapper.IX.bus.RX_fifo_almost_full),
					.control_out({i2c_wrapper.IX.bus.en_clock_strech, i2c_wrapper.IX.bus.transaction_begin, i2c_wrapper.IX.bus.data_direction, i2c_wrapper.IX.bus.packet_size[5:0], i2c_wrapper.IX.bus.ms_select, i2c_wrapper.IX.bus.address_mode}),
					.address(i2c_wrapper.IX.bus.bus_address),
					.clk_out(i2c_wrapper.IX.bus.clk_divider),
					.tx_empty(i2c_wrapper.IX.bus.TX_fifo_empty)
				);
			end
			assign REGISTER.clk = clk;
			assign REGISTER.n_rst = n_rst;
			assign i2c_wrapper.i2c.interrupt = REGISTER.interrupt;
			if (1) begin : SLAVE
				wire clk;
				wire n_rst;
				slave_inner SLAVE_INNER(
					.clk(clk),
					.n_rst(n_rst),
					.tx_data(i2c_wrapper.IX.bus.tx_data),
					.address_mode(i2c_wrapper.IX.bus.address_mode),
					.ms_select(i2c_wrapper.IX.bus.ms_select),
					.bus_address(i2c_wrapper.IX.bus.bus_address),
					.en_clock_strech(i2c_wrapper.IX.bus.en_clock_strech),
					.TX_fifo_empty(i2c_wrapper.IX.bus.TX_fifo_empty),
					.RX_fifo_full(i2c_wrapper.IX.bus.RX_fifo_full),
					.RX_fifo_almost_full(i2c_wrapper.IX.bus.RX_fifo_almost_full),
					.SDA_sync(i2c_wrapper.IX.bus.SDA_sync),
					.SCL_sync(i2c_wrapper.IX.bus.SCL_sync),
					.rx_data_slave(i2c_wrapper.IX.bus.rx_data_slave),
					.set_transaction_complete_slave(i2c_wrapper.IX.bus.set_transaction_complete_slave),
					.ack_error_set_slave(i2c_wrapper.IX.bus.ack_error_set_slave),
					.busy_slave(i2c_wrapper.IX.bus.busy_slave),
					.TX_read_enable_slave(i2c_wrapper.IX.bus.TX_read_enable_slave),
					.RX_write_enable_slave(i2c_wrapper.IX.bus.RX_write_enable_slave),
					.SDA_out_slave(i2c_wrapper.IX.bus.SDA_out_slave),
					.SCL_out_slave(i2c_wrapper.IX.bus.SCL_out_slave)
				);
			end
			assign SLAVE.clk = clk;
			assign SLAVE.n_rst = n_rst;
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
				assign SDA_sync = i2c_wrapper.IX.bus.SDA_sync;
				assign SCL_sync = i2c_wrapper.IX.bus.SCL_sync;
				wire SDA_out_shift_register;
				wire SDA_out_start_stop_gen;
				wire SCL_out_timer;
				wire SCL_out_start_stop_gen;
				wire [9:0] bus_address;
				wire [31:0] clock_div;
				wire clock_stretch_enabled;
				wire data_direction;
				wire address_mode;
				assign should_nack = (!i2c_wrapper.IX.bus.en_clock_strech && i2c_wrapper.IX.bus.RX_fifo_almost_full) || zero;
				assign i2c_wrapper.IX.bus.line_busy = bus_busy;
				assign i2c_wrapper.IX.bus.SDA_out_master = SDA_out;
				assign i2c_wrapper.IX.bus.SCL_out_master = SCL_out;
				wire start;
				wire stop;
				master_controller MASTER_CONTROLLER(
					.clk(clk),
					.n_rst(n_rst),
					.address_mode(address_mode),
					.ms_select(i2c_wrapper.IX.bus.ms_select),
					.bus_busy(bus_busy),
					.begin_transaction_flag(i2c_wrapper.IX.bus.transaction_begin),
					.ack_bit(ack_bit),
					.data_direction(data_direction),
					.output_wait_expired(output_wait_expired),
					.byte_complete(byte_complete),
					.zero_bytes_left(zero),
					.abort(abort),
					.stretch_enabled(clock_stretch_enabled),
					.rx_fifo_full(i2c_wrapper.IX.bus.RX_fifo_full),
					.tx_fifo_empty(i2c_wrapper.IX.bus.TX_fifo_empty),
					.shift_input_select(shift_input_select),
					.output_select(output_select),
					.shift_direction(shift_direction),
					.shift_load(shift_load),
					.timer_active(timer_active),
					.load_buffers(load_buffers),
					.decrement_byte_counter(decrement_byte_counter),
					.set_ack_error(i2c_wrapper.IX.bus.ack_error_set_master),
					.set_arbitration_lost(i2c_wrapper.IX.bus.set_arbitration_lost),
					.clear_transaction_begin(i2c_wrapper.IX.bus.transaction_begin_clear),
					.start(start),
					.stop(stop),
					.tx_fifo_enable(i2c_wrapper.IX.bus.TX_read_enable_master),
					.rx_fifo_enable(i2c_wrapper.IX.bus.RX_write_enable_master),
					.busy(i2c_wrapper.IX.bus.busy_master),
					.set_transaction_complete(i2c_wrapper.IX.bus.set_transaction_complete_master)
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
					.u_bus_address(i2c_wrapper.IX.bus.bus_address),
					.u_data_direction(i2c_wrapper.IX.bus.data_direction),
					.u_address_mode(i2c_wrapper.IX.bus.address_mode),
					.u_clock_div(i2c_wrapper.IX.bus.clk_divider),
					.u_stretch_enabled(i2c_wrapper.IX.bus.en_clock_strech),
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
					.tx_data(i2c_wrapper.IX.bus.tx_data),
					.shift_input_select(shift_input_select),
					.data_direction(data_direction),
					.shift_direction(shift_direction),
					.shift_in(SDA_sync),
					.shift_load(shift_load),
					.shift_strobe(shift_strobe),
					.shift_out(SDA_out_shift_register),
					.data_out(i2c_wrapper.IX.bus.rx_data_master)
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
					.packet_length(i2c_wrapper.IX.bus.packet_size[5:0]),
					.zero(zero),
					.one(one)
				);
			end
			assign MASTER.clk = clk;
			assign MASTER.n_rst = n_rst;
			reg SDA_out_next;
			reg SCL_out_next;
			always @(posedge clk or negedge n_rst)
				if (!n_rst) begin
					i2c_wrapper.i2c.SDA_out = 1;
					i2c_wrapper.i2c.SCL_out = 1;
				end
				else begin
					i2c_wrapper.i2c.SDA_out = SDA_out_next;
					i2c_wrapper.i2c.SCL_out = SCL_out_next;
				end
			always @(*)
				if (bus.ms_select == 0) begin
					bus.rx_data = bus.rx_data_master;
					bus.set_transaction_complete = bus.set_transaction_complete_master;
					bus.ack_error_set = bus.ack_error_set_master;
					bus.busy = bus.busy_master;
					bus.TX_read_enable = bus.TX_read_enable_master;
					bus.RX_write_enable = bus.RX_write_enable_master;
					SDA_out_next = bus.SDA_out_master;
					SCL_out_next = bus.SCL_out_master;
				end
				else begin
					bus.rx_data = bus.rx_data_slave;
					bus.set_transaction_complete = bus.set_transaction_complete_slave;
					bus.ack_error_set = bus.ack_error_set_slave;
					bus.busy = bus.busy_slave;
					bus.TX_read_enable = bus.TX_read_enable_slave;
					bus.RX_write_enable = bus.RX_write_enable_slave;
					SDA_out_next = bus.SDA_out_slave;
					SCL_out_next = bus.SCL_out_slave;
				end
		end
		assign IX.clk = clk;
		assign IX.n_rst = n_rst;
	endgenerate
endmodule
