/*
File name:   	timer.vhd
Created:     	2/28/2013
Author:      	Chuan Yean Tan
Version:	1.0 
Description: 	calculate offsets
		UART specifications: 115200 baud rate, system clock at 33MHz
                However the FPGA is running on 27MHz
                and the test bench is running at 50Mhz
*/
module timer(		input clk, n_Rst, 
			input enable_timer,
			output wire shift_strobe,
			output wire packet_done);

reg [3:0] count, nextcount;
reg [8:0] tim_count, next_tim_count;
reg trig;
   
   parameter cyclewaits = 434; //has to equal to clock frequency / baudrate
   
//ctrReg
always @ (posedge clk, negedge n_Rst)
begin 
	if (n_Rst == 1'b0) begin
		tim_count 	<= 9'd0;
		count 		<= 4'd0; 
	end else begin
		tim_count 	<= next_tim_count;
		count 		<= nextcount;
	end
end 

//ctrtime
always @ (enable_timer,count,tim_count)
begin

	//default value 
	next_tim_count = tim_count;
	nextcount = count;
	
	//first loop takes care of the start bit
	//second loop till end clocks when it is at the center
	//assert trigger

	//old values: 286 & 143
	//FPGA values : 234 & 117

	if ((count == 4'd0) && (tim_count == (cyclewaits>>1)))begin 
		trig = 1'b1;
	end else if ((count > 4'd0) && (tim_count == cyclewaits))begin
		trig = 1'b1; 
	end else begin
		trig = 1'b0;
	end

	//increment the timing counter
	if ((count == 4'd0) && (tim_count < (cyclewaits>>1)) && (enable_timer == 1'b1))begin
		next_tim_count = tim_count + 1;
	end else if ((count != 4'd0) && (tim_count < cyclewaits) && (enable_timer == 1'b1)) begin
		next_tim_count = tim_count + 1;
	end else if ((count == 4'd0) && (tim_count == (cyclewaits>>1)))begin 
		next_tim_count = 4'd0;
	end else if ((count > 4'd0) && (tim_count == cyclewaits)) begin
		next_tim_count = 4'd0;
	end else begin 
		next_tim_count = tim_count;
	end

	//bit counter
	if ((count < 4'd10) && (trig == 1'b1))begin
		nextcount = count + 1;
	end else if (count == 4'd10)begin 
		nextcount = 4'd0;
	end else begin
		nextcount = count; 
	end

end	
	assign shift_strobe = ((count <= 4'd8) &&(trig == 1'b1) && (count != 4'd0)) ? 1'b1 : 1'b0;
	assign packet_done = (count == 4'd10) ? 1'b1: 1'b0; 


endmodule
