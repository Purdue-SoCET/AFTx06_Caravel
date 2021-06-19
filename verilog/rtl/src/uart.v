module uart (
	clk,
	n_rst,
	rx,
	tx,
	transmit,
	tx_byte,
	received,
	rx_byte,
	is_receiving,
	is_transmitting,
	recv_error,
	sent
);
	input clk;
	input n_rst;
	input rx;
	output tx;
	input transmit;
	input [7:0] tx_byte;
	output received;
	output [7:0] rx_byte;
	output is_receiving;
	output is_transmitting;
	output recv_error;
	output sent;
	parameter CLOCK_DIVIDE = 108;
	reg [28:0] rx_s;
	reg [28:0] tx_s;
	reg [2:0] rx_state;
	reg [2:0] tx_state;
	reg tx_out;
	localparam [2:0] RX_RECEIVED = 6;
	assign received = rx_state == RX_RECEIVED;
	localparam [2:0] RX_ERROR = 5;
	assign recv_error = rx_state == RX_ERROR;
	localparam [2:0] RX_IDLE = 0;
	assign is_receiving = rx_state != RX_IDLE;
	assign rx_byte = rx_s[18-:8];
	assign tx = tx_out;
	localparam [2:0] TX_IDLE = 0;
	assign is_transmitting = tx_state != TX_IDLE;
	localparam [2:0] TX_BYTE_SENT = 3;
	assign sent = tx_state == TX_BYTE_SENT;
	localparam [2:0] RX_CHECK_START = 1;
	localparam [2:0] RX_CHECK_STOP = 3;
	localparam [2:0] RX_DELAY_RESTART = 4;
	localparam [2:0] RX_READ_BITS = 2;
	localparam [2:0] TX_DELAY_RESTART = 2;
	localparam [2:0] TX_SENDING = 1;
	localparam [2:0] TX_STOP = 4;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			rx_state = RX_IDLE;
			tx_state = TX_IDLE;
			rx_s[28-:6] = {6 {1'sb0}};
			rx_s[22-:4] = {4 {1'sb0}};
			rx_s[18-:8] = {8 {1'sb0}};
			rx_s[10-:11] = {11 {1'sb0}};
			tx_s[28-:6] = {6 {1'sb0}};
			tx_s[22-:4] = {4 {1'sb0}};
			tx_s[18-:8] = {8 {1'sb0}};
			tx_s[10-:11] = {11 {1'sb0}};
			tx_out = 0;
		end
		else begin
			rx_s[10-:11] = rx_s[10-:11] - 1;
			if (!rx_s[10-:11]) begin
				rx_s[10-:11] = CLOCK_DIVIDE;
				rx_s[28-:6] = rx_s[28-:6] - 1;
			end
			tx_s[10-:11] = tx_s[10-:11] - 1;
			if (!tx_s[10-:11]) begin
				tx_s[10-:11] = CLOCK_DIVIDE;
				tx_s[28-:6] = tx_s[28-:6] - 1;
			end
			case (rx_state)
				RX_IDLE:
					if (!rx) begin
						rx_s[10-:11] = CLOCK_DIVIDE;
						rx_s[28-:6] = 2;
						rx_state = RX_CHECK_START;
					end
				RX_CHECK_START:
					if (!rx_s[28-:6])
						if (!rx) begin
							rx_s[28-:6] = 4;
							rx_s[22-:4] = 8;
							rx_state = RX_READ_BITS;
						end
						else
							rx_state = RX_ERROR;
				RX_READ_BITS:
					if (!rx_s[28-:6]) begin
						rx_s[18-:8] = {rx, rx_s[18:12]};
						rx_s[28-:6] = 4;
						rx_s[22-:4] = rx_s[22-:4] - 1;
						rx_state = (rx_s[22-:4] ? RX_READ_BITS : RX_CHECK_STOP);
					end
				RX_CHECK_STOP:
					if (!rx_s[28-:6])
						rx_state = (rx ? RX_RECEIVED : RX_ERROR);
				RX_DELAY_RESTART: rx_state = (rx_s[28-:6] ? RX_DELAY_RESTART : RX_IDLE);
				RX_ERROR: begin
					rx_s[28-:6] = 8;
					rx_state = RX_DELAY_RESTART;
				end
				RX_RECEIVED: rx_state = RX_IDLE;
			endcase
			case (tx_state)
				TX_IDLE: begin
					tx_s[28-:6] = 0;
					if (transmit) begin
						tx_s[18-:8] = tx_byte;
						tx_s[10-:11] = CLOCK_DIVIDE;
						tx_s[28-:6] = 4;
						tx_out = 0;
						tx_s[22-:4] = 8;
						tx_state = TX_SENDING;
					end
				end
				TX_SENDING:
					if (!tx_s[28-:6])
						if (tx_s[22-:4]) begin
							tx_s[22-:4] = tx_s[22-:4] - 1;
							tx_out = tx_s[11];
							tx_s[18-:8] = {1'b0, tx_s[18:12]};
							tx_s[28-:6] = 4;
							tx_state = TX_SENDING;
						end
						else begin
							tx_out = 1;
							tx_s[28-:6] = 8;
							tx_state = TX_DELAY_RESTART;
						end
				TX_DELAY_RESTART: tx_state = (tx_s[28-:6] == 0 ? TX_BYTE_SENT : TX_DELAY_RESTART);
				TX_BYTE_SENT: tx_state = TX_STOP;
				TX_STOP: tx_state = TX_IDLE;
			endcase
		end
endmodule
