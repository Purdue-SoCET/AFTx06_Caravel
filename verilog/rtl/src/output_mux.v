module output_mux (
	drive_select,
	tx_SDA,
	tx_SCL,
	rx_SDA,
	rx_SCL,
	start_stop_SDA,
	start_stop_SCL,
	SDA_out,
	SCL_out
);
	input wire [1:0] drive_select;
	input wire tx_SDA;
	input wire tx_SCL;
	input wire rx_SDA;
	input wire rx_SCL;
	input wire start_stop_SDA;
	input wire start_stop_SCL;
	output reg SDA_out;
	output reg SCL_out;
	localparam [1:0] DS_RECEIVE = 2;
	localparam [1:0] DS_START_STOP = 1;
	localparam [1:0] DS_TRANSMIT = 3;
	always @(*)
		case (drive_select)
			DS_START_STOP: begin
				SDA_out = start_stop_SDA;
				SCL_out = start_stop_SCL;
			end
			DS_RECEIVE: begin
				SDA_out = rx_SDA;
				SCL_out = rx_SCL;
			end
			DS_TRANSMIT: begin
				SDA_out = tx_SDA;
				SCL_out = tx_SCL;
			end
			default: begin
				SDA_out = 1;
				SCL_out = 1;
			end
		endcase
endmodule
