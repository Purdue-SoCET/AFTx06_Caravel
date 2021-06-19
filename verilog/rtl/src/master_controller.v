module master_controller (
	clk,
	n_rst,
	address_mode,
	ms_select,
	bus_busy,
	begin_transaction_flag,
	ack_bit,
	data_direction,
	output_wait_expired,
	byte_complete,
	zero_bytes_left,
	abort,
	stretch_enabled,
	rx_fifo_full,
	tx_fifo_empty,
	shift_input_select,
	output_select,
	shift_direction,
	shift_load,
	timer_active,
	load_buffers,
	decrement_byte_counter,
	set_ack_error,
	set_arbitration_lost,
	clear_transaction_begin,
	start,
	stop,
	tx_fifo_enable,
	rx_fifo_enable,
	busy,
	set_transaction_complete
);
	input clk;
	input n_rst;
	input address_mode;
	input ms_select;
	input bus_busy;
	input begin_transaction_flag;
	input ack_bit;
	input data_direction;
	input output_wait_expired;
	input byte_complete;
	input zero_bytes_left;
	input abort;
	input stretch_enabled;
	input rx_fifo_full;
	input tx_fifo_empty;
	output reg [1:0] shift_input_select;
	output reg [1:0] output_select;
	output reg shift_direction;
	output reg shift_load;
	output reg timer_active;
	output reg load_buffers;
	output reg decrement_byte_counter;
	output reg set_ack_error;
	output reg set_arbitration_lost;
	output reg clear_transaction_begin;
	output reg start;
	output reg stop;
	output reg tx_fifo_enable;
	output reg rx_fifo_enable;
	output reg busy;
	output reg set_transaction_complete;
	reg [4:0] state;
	reg [4:0] next_state;
	localparam [0:0] ADDR_7_BIT = 0;
	localparam [4:0] CHK_ADD_ACK1 = 10;
	localparam [4:0] CHK_ADD_ACK2 = 11;
	localparam [4:0] CHK_BYTE_COUNT = 12;
	localparam [4:0] CHK_T_ACK = 18;
	localparam [4:0] DEC_BYTE_COUNT = 13;
	localparam [4:0] FLAG_CLEAR = 1;
	localparam [4:0] IDLE = 0;
	localparam [4:0] LOAD_10ADDR1 = 6;
	localparam [4:0] LOAD_10ADDR2 = 8;
	localparam [4:0] LOAD_7ADDR = 4;
	localparam [4:0] LOAD_BUFFER = 2;
	localparam [4:0] LOAD_BYTE = 15;
	localparam [4:0] RECEIVE = 17;
	localparam [0:0] RX = 0;
	localparam [4:0] SAVE_BYTE = 19;
	localparam [4:0] SEND_10ADDR1 = 7;
	localparam [4:0] SEND_10ADDR2 = 9;
	localparam [4:0] SEND_7ADDR = 5;
	localparam [4:0] SEND_START = 3;
	localparam [4:0] SEND_STOP = 24;
	localparam [4:0] SET_ABORT = 21;
	localparam [4:0] SET_COMPLETE = 22;
	localparam [4:0] SET_ERROR = 20;
	localparam [4:0] SR_SET_COMPLETE = 23;
	localparam [4:0] STRETCH = 14;
	localparam [4:0] TRANSMIT = 16;
	always @(*) begin
		next_state = state;
		case (state)
			IDLE:
				if (begin_transaction_flag && !bus_busy)
					next_state = FLAG_CLEAR;
			FLAG_CLEAR: next_state = LOAD_BUFFER;
			LOAD_BUFFER: next_state = SEND_START;
			SEND_START:
				if (output_wait_expired)
					if (address_mode == ADDR_7_BIT)
						next_state = LOAD_7ADDR;
					else
						next_state = LOAD_10ADDR1;
			LOAD_7ADDR: next_state = SEND_7ADDR;
			LOAD_10ADDR1: next_state = SEND_10ADDR1;
			LOAD_10ADDR2: next_state = SEND_10ADDR2;
			SEND_7ADDR: begin
				if (abort)
					next_state = SET_ABORT;
				if (byte_complete)
					next_state = CHK_ADD_ACK2;
			end
			SEND_10ADDR1: begin
				if (abort)
					next_state = SET_ABORT;
				if (byte_complete)
					next_state = CHK_ADD_ACK1;
			end
			SEND_10ADDR2: begin
				if (abort)
					next_state = SET_ABORT;
				if (byte_complete)
					next_state = CHK_ADD_ACK2;
			end
			CHK_ADD_ACK1: next_state = (!ack_bit ? LOAD_10ADDR2 : SET_ERROR);
			CHK_ADD_ACK2: next_state = (!ack_bit ? CHK_BYTE_COUNT : SET_ERROR);
			CHK_BYTE_COUNT:
				if (zero_bytes_left)
					next_state = (begin_transaction_flag ? SR_SET_COMPLETE : SEND_STOP);
				else
					next_state = DEC_BYTE_COUNT;
			SET_COMPLETE: next_state = IDLE;
			SR_SET_COMPLETE: next_state = FLAG_CLEAR;
			DEC_BYTE_COUNT: next_state = STRETCH;
			STRETCH:
				if (data_direction == RX) begin
					if (rx_fifo_full && stretch_enabled)
						next_state = STRETCH;
					else if (rx_fifo_full)
						next_state = SEND_STOP;
					else
						next_state = RECEIVE;
				end
				else if (tx_fifo_empty && stretch_enabled)
					next_state = STRETCH;
				else if (tx_fifo_empty)
					next_state = SEND_STOP;
				else
					next_state = LOAD_BYTE;
			RECEIVE: next_state = (byte_complete ? SAVE_BYTE : RECEIVE);
			SAVE_BYTE: next_state = CHK_BYTE_COUNT;
			LOAD_BYTE: next_state = TRANSMIT;
			TRANSMIT:
				if (abort)
					next_state = SET_ABORT;
				else
					next_state = (byte_complete ? CHK_T_ACK : TRANSMIT);
			CHK_T_ACK: next_state = (ack_bit == 0 ? CHK_BYTE_COUNT : SET_ERROR);
			SET_ERROR:
				if (begin_transaction_flag)
					next_state = SR_SET_COMPLETE;
				else
					next_state = SEND_STOP;
			SET_ABORT: next_state = SET_COMPLETE;
			SEND_STOP: next_state = (output_wait_expired ? SET_COMPLETE : SEND_STOP);
		endcase
	end
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			state <= IDLE;
		else
			state <= next_state;
	localparam [1:0] DS_IDLE = 0;
	localparam [1:0] DS_RECEIVE = 2;
	localparam [1:0] DS_START_STOP = 1;
	localparam [1:0] DS_TRANSMIT = 3;
	localparam [1:0] SS_10_BIT_ADDRESS_BYTE_1 = 0;
	localparam [1:0] SS_10_BIT_ADDRESS_BYTE_2 = 1;
	localparam [1:0] SS_7_BIT_ADDRESS = 2;
	localparam [1:0] SS_TX_FIFO = 3;
	localparam [0:0] TX = 1;
	always @(*) begin
		shift_input_select = SS_TX_FIFO;
		output_select = DS_RECEIVE;
		shift_direction = RX;
		shift_load = 0;
		timer_active = 0;
		load_buffers = 0;
		decrement_byte_counter = 0;
		set_ack_error = 0;
		set_arbitration_lost = 0;
		clear_transaction_begin = 0;
		stop = 0;
		start = 0;
		busy = 1;
		set_transaction_complete = 0;
		tx_fifo_enable = 0;
		rx_fifo_enable = 0;
		case (state)
			IDLE: begin
				busy = 0;
				output_select = DS_IDLE;
			end
			FLAG_CLEAR: begin
				clear_transaction_begin = 1;
				output_select = DS_IDLE;
			end
			LOAD_BUFFER: begin
				load_buffers = 1;
				output_select = DS_IDLE;
			end
			SEND_START: begin
				output_select = DS_START_STOP;
				start = 1;
			end
			SEND_STOP: begin
				output_select = DS_START_STOP;
				stop = 1;
			end
			LOAD_7ADDR: begin
				shift_input_select = SS_7_BIT_ADDRESS;
				shift_load = 1;
			end
			LOAD_10ADDR1: begin
				shift_input_select = SS_10_BIT_ADDRESS_BYTE_1;
				shift_load = 1;
			end
			LOAD_10ADDR2: begin
				shift_input_select = SS_10_BIT_ADDRESS_BYTE_2;
				shift_load = 1;
			end
			LOAD_BYTE: begin
				shift_input_select = SS_TX_FIFO;
				shift_load = 1;
				tx_fifo_enable = 1;
			end
			SEND_7ADDR: begin
				timer_active = 1;
				shift_direction = TX;
				output_select = DS_TRANSMIT;
			end
			SEND_10ADDR1: begin
				timer_active = 1;
				shift_direction = TX;
				output_select = DS_TRANSMIT;
			end
			SEND_10ADDR2: begin
				timer_active = 1;
				shift_direction = TX;
				output_select = DS_TRANSMIT;
			end
			TRANSMIT: begin
				timer_active = 1;
				shift_direction = TX;
				output_select = DS_TRANSMIT;
			end
			DEC_BYTE_COUNT: decrement_byte_counter = 1;
			RECEIVE: begin
				timer_active = 1;
				shift_direction = RX;
			end
			SAVE_BYTE: rx_fifo_enable = 1;
			SET_ERROR: set_ack_error = 1;
			SET_ABORT: set_arbitration_lost = 1;
			SET_COMPLETE: begin
				set_transaction_complete = 1;
				output_select = DS_IDLE;
			end
			SR_SET_COMPLETE: set_transaction_complete = 1;
		endcase
	end
endmodule
