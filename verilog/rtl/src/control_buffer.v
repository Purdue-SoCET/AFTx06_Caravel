module control_buffer (
	clk,
	n_rst,
	u_bus_address,
	u_data_direction,
	u_address_mode,
	u_clock_div,
	u_stretch_enabled,
	load_buffer,
	bus_address,
	data_direction,
	address_mode,
	stretch_enabled,
	clock_div
);
	input wire clk;
	input wire n_rst;
	input wire [9:0] u_bus_address;
	input wire u_data_direction;
	input wire u_address_mode;
	input wire [31:0] u_clock_div;
	input wire u_stretch_enabled;
	input wire load_buffer;
	output reg [9:0] bus_address;
	output reg data_direction;
	output reg address_mode;
	output reg stretch_enabled;
	output reg [31:0] clock_div;
	localparam [0:0] ADDR_7_BIT = 0;
	localparam [0:0] RX = 0;
	always @(posedge clk or negedge n_rst)
		if (!n_rst) begin
			bus_address <= 10'b0000000000;
			data_direction <= RX;
			address_mode <= ADDR_7_BIT;
			stretch_enabled <= 1;
			clock_div <= 300;
		end
		else if (load_buffer) begin
			bus_address <= u_bus_address;
			data_direction <= u_data_direction;
			address_mode <= u_address_mode;
			stretch_enabled <= u_stretch_enabled;
			clock_div <= u_clock_div;
		end
		else begin
			bus_address <= bus_address;
			data_direction <= data_direction;
			address_mode <= address_mode;
			stretch_enabled <= stretch_enabled;
			clock_div <= clock_div;
		end
endmodule
