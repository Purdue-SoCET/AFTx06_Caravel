module slave_controller (
	clk,
	n_rst,
	start,
	stop,
	address_match,
	rw_mode,
	SDA_sync,
	address_mode,
	TX_fifo_empty,
	RX_fifo_full,
	en_clock_strech,
	RX_fifo_almost_full,
	byte_received,
	ack_prep,
	ack_check,
	ack_done,
	rx_enable,
	SCL_out_slave,
	busy_slave,
	TX_read_enable_slave,
	RX_write_enable_slave,
	ack_error_set_slave,
	sda_mode,
	load_data,
	tx_enable,
	rw_store
);
	input wire clk;
	input wire n_rst;
	input wire start;
	input wire stop;
	input wire [1:0] address_match;
	input wire rw_mode;
	input wire SDA_sync;
	input wire address_mode;
	input wire TX_fifo_empty;
	input wire RX_fifo_full;
	input wire en_clock_strech;
	input wire RX_fifo_almost_full;
	input wire byte_received;
	input wire ack_prep;
	input wire ack_check;
	input wire ack_done;
	output reg rx_enable;
	output reg SCL_out_slave;
	output reg busy_slave;
	output reg TX_read_enable_slave;
	output reg RX_write_enable_slave;
	output reg ack_error_set_slave;
	output reg [1:0] sda_mode;
	output reg load_data;
	output reg tx_enable;
	output reg rw_store;
	reg [4:0] state;
	reg [4:0] next_state;
	reg temp_rx_enable;
	reg temp_SCL_out_slave;
	reg temp_busy_slave;
	reg temp_TX_read_enable_slave;
	reg temp_RX_write_enable_slave;
	reg temp_ack_error_set_slave;
	reg [1:0] temp_sda_mode;
	reg temp_load_data;
	reg temp_tx_enable;
	localparam [4:0] IDLE = 0;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			state <= IDLE;
			rx_enable <= 1'b0;
			SCL_out_slave <= 1'b1;
			busy_slave <= 1'b0;
			TX_read_enable_slave <= 1'b0;
			RX_write_enable_slave <= 1'b0;
			ack_error_set_slave <= 1'b0;
			sda_mode <= 2'b00;
			load_data <= 1'b0;
			tx_enable <= 1'b0;
		end
		else begin
			state <= next_state;
			rx_enable <= temp_rx_enable;
			SCL_out_slave <= temp_SCL_out_slave;
			busy_slave <= temp_busy_slave;
			TX_read_enable_slave <= temp_TX_read_enable_slave;
			RX_write_enable_slave <= temp_RX_write_enable_slave;
			ack_error_set_slave <= temp_ack_error_set_slave;
			sda_mode <= temp_sda_mode;
			load_data <= temp_load_data;
			tx_enable <= temp_tx_enable;
		end
	localparam [4:0] ACK_CHECK = 12;
	localparam [4:0] ACK_SEND_1 = 4;
	localparam [4:0] ACK_SEND_2 = 7;
	localparam [4:0] CHECK_ADDR_1 = 2;
	localparam [4:0] CHECK_ADDR_2 = 6;
	localparam [4:0] DATA_START = 10;
	localparam [4:0] DATA_STOP = 11;
	localparam [4:0] FIFO_CHK_RX = 16;
	localparam [4:0] FIFO_CHK_TX = 8;
	localparam [4:0] GET_ADDR_1 = 1;
	localparam [4:0] GET_ADDR_2 = 5;
	localparam [4:0] LOAD = 9;
	localparam [4:0] NO_MATCH = 3;
	localparam [4:0] READ_ENABLE = 13;
	localparam [4:0] RE_ACK = 15;
	localparam [4:0] RE_NACK = 14;
	localparam [4:0] SEND_1 = 17;
	localparam [4:0] SEND_2 = 20;
	localparam [4:0] SEND_3 = 23;
	localparam [4:0] SEND_ACK = 18;
	localparam [4:0] SEND_NACK = 19;
	localparam [4:0] STRETCH_LOAD = 22;
	localparam [4:0] STRETCH_TX = 21;
	always @(*) begin
		next_state = state;
		case (state)
			IDLE:
				if (start == 1'b1)
					next_state = GET_ADDR_1;
			GET_ADDR_1:
				if (ack_prep == 1'b1)
					next_state = CHECK_ADDR_1;
			CHECK_ADDR_1:
				if ((address_mode == 1'b0) && (address_match[1] == 1'b1))
					next_state = ACK_SEND_2;
				else if ((address_mode == 1'b1) && (address_match[1] == 1'b1))
					next_state = ACK_SEND_1;
				else if (address_match[1] == 1'b0)
					next_state = NO_MATCH;
			NO_MATCH:
				if (ack_done == 1'b1)
					next_state = IDLE;
			ACK_SEND_1:
				if (ack_done == 1'b1)
					next_state = GET_ADDR_2;
			GET_ADDR_2:
				if (ack_prep == 1'b1)
					next_state = CHECK_ADDR_2;
			CHECK_ADDR_2:
				if (address_match[0] == 1'b1)
					next_state = ACK_SEND_2;
				else if (address_match[0] == 1'b0)
					next_state = NO_MATCH;
			ACK_SEND_2:
				if ((rw_mode == 1'b1) && (ack_done == 1'b1))
					next_state = FIFO_CHK_TX;
				else if ((rw_mode == 1'b0) && (ack_done == 1'b1))
					next_state = FIFO_CHK_RX;
			FIFO_CHK_TX:
				if ((en_clock_strech == 1'b1) && (TX_fifo_empty == 1'b1))
					next_state = STRETCH_TX;
				else
					next_state = LOAD;
			STRETCH_TX:
				if ((en_clock_strech == 1'b1) && (TX_fifo_empty == 1'b1))
					next_state = STRETCH_TX;
				else
					next_state = STRETCH_LOAD;
			LOAD: next_state = DATA_START;
			STRETCH_LOAD: next_state = DATA_START;
			DATA_START:
				if (ack_prep == 1'b1)
					next_state = DATA_STOP;
			DATA_STOP:
				if (ack_check == 1'b1)
					next_state = ACK_CHECK;
				else if (stop == 1'b1)
					next_state = IDLE;
			ACK_CHECK:
				if (stop == 1'b1)
					next_state = IDLE;
				else
					next_state = READ_ENABLE;
			READ_ENABLE:
				if (stop == 1'b1)
					next_state = IDLE;
				else if (SDA_sync == 1'b0)
					next_state = RE_ACK;
				else
					next_state = RE_NACK;
			RE_ACK:
				if (ack_done == 1'b1)
					next_state = FIFO_CHK_TX;
				else if (stop == 1'b1)
					next_state = IDLE;
			RE_NACK:
				if (stop == 1'b1)
					next_state = IDLE;
				else if (start == 1'b1)
					next_state = GET_ADDR_1;
				else
					next_state = RE_NACK;
			FIFO_CHK_RX:
				if ((en_clock_strech == 1'b1) && (RX_fifo_full == 1'b1))
					next_state = FIFO_CHK_RX;
				else
					next_state = SEND_1;
			SEND_1:
				if (stop == 1'b1)
					next_state = IDLE;
				else if (start == 1'b1)
					next_state = GET_ADDR_1;
				else if ((RX_fifo_full == 1'b1) && (ack_prep == 1'b1))
					next_state = SEND_NACK;
				else if ((RX_fifo_full == 1'b0) && (ack_prep == 1'b1))
					next_state = SEND_ACK;
			SEND_ACK:
				if (ack_done == 1'b1)
					next_state = SEND_2;
			SEND_NACK:
				if (ack_done == 1'b1)
					next_state = SEND_2;
			SEND_2:
				if (stop == 1'b1)
					next_state = IDLE;
				else if (start == 1'b1)
					next_state = GET_ADDR_1;
				else
					next_state = SEND_3;
			SEND_3: next_state = FIFO_CHK_RX;
		endcase
	end
	always @(state) begin
		temp_rx_enable = 1'b0;
		temp_SCL_out_slave = 1'b1;
		temp_busy_slave = 1'b0;
		temp_TX_read_enable_slave = 1'b0;
		temp_RX_write_enable_slave = 1'b0;
		temp_ack_error_set_slave = 1'b0;
		temp_sda_mode = 2'b00;
		temp_load_data = 1'b0;
		temp_tx_enable = 1'b0;
		rw_store = 1'b0;
		case (state)
			IDLE: begin
				temp_rx_enable = 1'b1;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b0;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			GET_ADDR_1: begin
				temp_rx_enable = 1'b1;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
				rw_store = 1'b1;
			end
			CHECK_ADDR_1: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			NO_MATCH: begin
				temp_rx_enable = 1'b1;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b10;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			ACK_SEND_1: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b01;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			GET_ADDR_2: begin
				temp_rx_enable = 1'b1;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			CHECK_ADDR_2: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			ACK_SEND_2: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b01;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			FIFO_CHK_TX, STRETCH_TX: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b0;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			LOAD: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b1;
				temp_tx_enable = 1'b0;
			end
			STRETCH_LOAD: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b0;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b1;
				temp_tx_enable = 1'b0;
			end
			DATA_START: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b11;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b1;
			end
			DATA_STOP: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			ACK_CHECK: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			READ_ENABLE: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b1;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			RE_NACK: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			RE_ACK: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			FIFO_CHK_RX: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b0;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			SEND_1: begin
				temp_rx_enable = 1'b1;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			SEND_ACK: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b01;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			SEND_NACK: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b1;
				temp_sda_mode = 2'b10;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			SEND_2: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b1;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			SEND_3: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b0;
				temp_busy_slave = 1'b1;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
			default: begin
				temp_rx_enable = 1'b0;
				temp_SCL_out_slave = 1'b1;
				temp_busy_slave = 1'b0;
				temp_TX_read_enable_slave = 1'b0;
				temp_RX_write_enable_slave = 1'b0;
				temp_ack_error_set_slave = 1'b0;
				temp_sda_mode = 2'b00;
				temp_load_data = 1'b0;
				temp_tx_enable = 1'b0;
			end
		endcase
	end
endmodule
