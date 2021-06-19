module shift_register (
	clk,
	n_rst,
	bus_address,
	tx_data,
	shift_input_select,
	data_direction,
	shift_direction,
	shift_strobe,
	shift_in,
	shift_load,
	shift_out,
	data_out
);
	input wire clk;
	input wire n_rst;
	input wire [9:0] bus_address;
	input wire [7:0] tx_data;
	input wire [1:0] shift_input_select;
	input wire data_direction;
	input wire shift_direction;
	input wire shift_strobe;
	input wire shift_in;
	input wire shift_load;
	output wire shift_out;
	output reg [7:0] data_out;
	reg [7:0] load_value;
	wire read_write;
	reg [7:0] next_data;
	localparam [0:0] RX = 0;
	assign read_write = data_direction == RX;
	assign shift_out = data_out[7];
	localparam [1:0] SS_10_BIT_ADDRESS_BYTE_1 = 0;
	localparam [1:0] SS_10_BIT_ADDRESS_BYTE_2 = 1;
	localparam [1:0] SS_7_BIT_ADDRESS = 2;
	localparam [1:0] SS_TX_FIFO = 3;
	always @(*)
		case (shift_input_select)
			SS_10_BIT_ADDRESS_BYTE_1: load_value = {5'b11110, bus_address[9:8], read_write};
			SS_10_BIT_ADDRESS_BYTE_2: load_value = bus_address[7:0];
			SS_7_BIT_ADDRESS: load_value = {bus_address[6:0], read_write};
			SS_TX_FIFO: load_value = tx_data;
		endcase
	localparam [0:0] TX = 1;
	always @(*)
		if (shift_load)
			next_data = load_value;
		else if (shift_strobe && (shift_direction == RX))
			next_data = {data_out[6:0], shift_in};
		else if (shift_strobe && (shift_direction == TX))
			next_data = {data_out[6:0], 1'b1};
		else
			next_data = data_out;
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			data_out <= 8'b11111111;
		else
			data_out <= next_data;
endmodule
