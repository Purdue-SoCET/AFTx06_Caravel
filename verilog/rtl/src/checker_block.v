module checker_block (
	clk,
	n_rst,
	SDA_sync,
	SCL_sync,
	rx_data,
	bus_address,
	address_mode,
	rw_store,
	rw_mode,
	address_match,
	start,
	stop
);
	input wire clk;
	input wire n_rst;
	input wire SDA_sync;
	input wire SCL_sync;
	input wire [7:0] rx_data;
	input wire [9:0] bus_address;
	input wire address_mode;
	input wire rw_store;
	output reg rw_mode;
	output reg [1:0] address_match;
	output reg start;
	output reg stop;
	reg scl_mid1;
	reg scl_mid2;
	reg sda_mid1;
	reg sda_mid2;
	reg [1:0] temp_address_match;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			scl_mid1 <= 1'b0;
			scl_mid2 <= 1'b0;
			sda_mid1 <= 1'b0;
			sda_mid2 <= 1'b0;
			rw_mode <= 1'b0;
			address_match <= 1'b0;
			start <= 1'b0;
			stop <= 1'b0;
		end
		else begin
			scl_mid1 <= SCL_sync;
			scl_mid2 <= scl_mid1;
			sda_mid1 <= SDA_sync;
			sda_mid2 <= sda_mid1;
			rw_mode <= (rw_store && rx_data[0]) || (!rw_store && rw_mode);
			address_match <= temp_address_match;
			start <= (((scl_mid1 && scl_mid2) && !sda_mid1) && sda_mid2 ? 1'b1 : 1'b0);
			stop <= (((scl_mid1 && scl_mid2) && sda_mid1) && !sda_mid2 ? 1'b1 : 1'b0);
		end
	always @(*)
		if (address_mode == 1'b0) begin
			temp_address_match[1] = (rx_data[7:1] == bus_address[6:0] ? 1'b1 : 1'b0);
			temp_address_match[0] = 0;
		end
		else begin
			temp_address_match[1] = (rx_data[2:1] == bus_address[9:8] ? 1'b1 : 1'b0);
			temp_address_match[0] = (rx_data[7:0] == bus_address[7:0] ? 1'b1 : 1'b0);
		end
endmodule
