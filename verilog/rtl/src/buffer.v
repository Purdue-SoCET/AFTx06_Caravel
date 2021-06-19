/*
File name:   buffer.vhd
Created:     3/11/2013
Author:      Chuan Yean Tan
Version:	1.0 
Description: 	loading rx bits
		
*/
module buffer(		input clk, n_Rst, 
			input shift_strobe,
			input packet_done,
			input rx,
			output reg [7:0] UDATA_IN
	     );

reg [7:0] next_data_reg,data_reg;
reg [7:0] next_data_rcv,data_rcv;
reg [7:0] next_udata_in;

always @ (posedge clk)
begin 
	if (n_Rst == 1'b0) begin
		data_reg 	<= 8'd0;
		UDATA_IN	<= 8'd0;
	end else begin
		data_reg <= next_data_reg;
		data_rcv <= next_data_rcv;
		UDATA_IN <= next_udata_in;
	end
end 

always @ (shift_strobe,packet_done,rx,data_reg,data_rcv,UDATA_IN)
begin
	if (shift_strobe == 1'b1) begin 
		next_data_reg <= {rx, data_reg[7:1]};
	end else begin 
		next_data_reg <= data_reg;
	end

	if (packet_done == 1'b1) begin
		next_data_rcv <= data_reg;
	end else begin
		next_data_rcv <= data_rcv; 
	end

	if (packet_done == 1'b1) begin 
		next_udata_in <= data_reg;
	end else begin 
		next_udata_in <= UDATA_IN; 
	end
end	




endmodule
