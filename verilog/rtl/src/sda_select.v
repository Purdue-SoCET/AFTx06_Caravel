module sda_select (
	sda_mode,
	tx_out,
	SDA_out_slave
);
	input wire [1:0] sda_mode;
	input wire tx_out;
	output reg SDA_out_slave;
	always @(*)
		if (sda_mode == 2'b00)
			SDA_out_slave = 1'b1;
		else if (sda_mode == 2'b01)
			SDA_out_slave = 1'b0;
		else if (sda_mode == 2'b10)
			SDA_out_slave = 1'b1;
		else if (sda_mode == 2'b11)
			SDA_out_slave = tx_out;
endmodule
