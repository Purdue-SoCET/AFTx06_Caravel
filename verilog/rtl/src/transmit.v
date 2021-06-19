/*
File name:   transmit.v
Created:     2/28/2013
Author:      Chuan Yean Tan
Version:     1.0 
Description: Transmit data out via shift register and counter

Specs: 	-Baud rate @ 115200 bits per second
	-System clock @ 50MHz
	-Offset of 286 cycles for each bit
        -FPGA clock @27MHz
        -offset of 234 cycles
*/
module transmit(	input clk, n_Rst, 
			input [7:0] data_in, 		//data packet to be transmitted
			input start_bit,		//sends start bit
			input enable_count, 		//enable tx
			input tx_enable,		//enable signal
			output reg start_send,		//indicates that start bit sent
			output reg byte_send, 		//indicates that packet is succesfully sent
			output reg data_out 		//data bit that is shifted out
		);
   parameter cyclewaits = 434;
   
reg next_shift, shift; 					//indicates to shift a data out
reg [7:0] data_to_send, next_data_to_send;		//register to be transmitted
reg [3:0] next_num_bits, num_bits;			//count number of bits 
reg [9:0] next_tim_count, tim_count;			//count number of clock cycles
reg next_data_out;
reg next_start_send;

//REGISTERS
always @ (posedge clk, negedge n_Rst) begin
	if (n_Rst == 1'b0) begin 
		tim_count 	<= 9'd0;
		num_bits	<= 4'd0;
		data_to_send	<= 8'b11111111;
		shift		<= 1'b0;
		data_out	<= 1'b1;
	end else begin
		data_to_send 	<= next_data_to_send;
		data_out	<= next_data_out;
		tim_count 	<= next_tim_count;
		num_bits	<= next_num_bits;
		shift 		<= next_shift;
	end 
end //end always count clock cycles

//OUTPUT LOGIC
always @ (start_bit, num_bits, shift, tim_count,tx_enable, data_in, data_to_send, data_out) begin 
if(tx_enable == 1'b1)begin 
	if (start_bit == 1'b1) begin 
		next_data_to_send <= data_in;
		next_data_out <= 1'b0;
	end else begin
		next_data_out <= data_to_send[0];
		if(shift == 1'b1) begin 
			next_data_to_send <= {1'b1, data_to_send[7:1]};	
		end else begin 
			next_data_to_send <= data_to_send;
		end
	end
end else begin 
	next_data_to_send <= data_to_send;
	next_data_out	  <= data_out;
	next_tim_count 	  <= tim_count;
	next_num_bits	  <= num_bits;
end

//NEXT NUM BITS
if (num_bits == 4'd9 & shift == 1'b1 & tx_enable == 1'b1) begin 
	next_num_bits <= 4'd0;
	byte_send <= 1'b1;
end else if (shift == 1'b1 & tx_enable == 1'b1) begin
	next_num_bits <= num_bits + 4'b1;  
	byte_send <= 1'b0;
end else begin 
	next_num_bits <= num_bits;
	byte_send <= 1'b0;
end 

//NEXT TIME COUNT
//286
if (tim_count == cyclewaits & tx_enable == 1'b1) begin
	next_tim_count <= 9'd0;
	if (start_bit == 1'b1) begin
		start_send <= 1'b1; 
	end else begin 
		start_send <= 1'b0;
	end 
end else if (tx_enable == 1'b1) begin
	next_tim_count 	<= tim_count + 8'd1; 
	start_send <= 1'b0;
end else begin
	next_tim_count <= tim_count;
	start_send <= 1'b0;
end

//NEXT SHIFT
//285
//233
if (tim_count == (cyclewaits-1)) begin
	next_shift <= 1'b1;
end else begin 
	next_shift <= 1'b0;
end

end 

endmodule

