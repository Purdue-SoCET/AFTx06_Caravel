module statusLogic (
	i2c_status,
	tx_full,
	tx_empty,
	rx_full,
	rx_empty,
	rx_w_ena,
	tx_r_ena,
	mid_tx_empty,
	mid_rx_full,
	next_status
);
	input wire [4:0] i2c_status;
	input wire tx_full;
	input wire tx_empty;
	input wire rx_full;
	input wire rx_empty;
	input wire rx_w_ena;
	input wire tx_r_ena;
	input wire mid_tx_empty;
	input wire mid_rx_full;
	output reg [12:0] next_status;
	reg rx_overflow;
	reg tx_underflow;
	always @(*)
		if (rx_full & rx_w_ena)
			rx_overflow = 1'b1;
		else
			rx_overflow = 1'b0;
	always @(*)
		if (tx_empty & tx_r_ena)
			tx_underflow = 1'b1;
		else
			tx_underflow = 1'b0;
	always @(*) next_status = {tx_underflow, tx_full, tx_empty, rx_overflow, rx_full, rx_empty, mid_tx_empty, mid_rx_full, i2c_status};
endmodule
