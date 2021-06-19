/*
Reference UART design : 
1. http://www.quicklogic.com/assets/pdf/data_sheets/QL_UART_PSB_DS_RevC.pdf
2. http://www.ece301.com/fpga-projects/52-uart-txd.html
3. http://www.engr.siu.edu/~haibo/ece428/notes/ece428_uart.pdf
4. http://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter

--FILE DETAILS--
File name:   	tcu.v
Created:     	2/28/2013
Author:      	Chuan Yean Tan
Version:		1.0 
Description: 	transmitter control unit
*/ 

module tcu(		input clk, n_Rst, 
			input byte_send,		//data is sent
			input start_trans, 		//signal from the debugger to start transmitting
			input start_send,
			output reg enable_count,//counts the number of bits that is sent out
			output reg enable_start,
			output reg tx_enable,
			output reg start_bit,	//send start bit
			output reg tx_state		//indicating whether it is IDLE(0) or BUSY(1) 
				);

				
//TOP LEVEL STATES
parameter [2:0]
		IDLE		= 	3'd0,
		START_SEND 	= 	3'd1,
		START_BIT	= 	3'd2,
		SEND_BIT	= 	3'd3, 
		STOP	 	= 	3'd4;
		
reg [2:0] state;
reg [2:0] nextState;
		
//next state logic
always @ (posedge clk, negedge n_Rst) begin 

	if (n_Rst == 1'd0) begin 
		state <= IDLE; 
	end else begin 
		state <= nextState;
	end
end 

//state machines transition
always @ (state, start_trans, byte_send, start_send) begin
	case (state)
	
	IDLE: begin 
		if (start_trans == 1'b1) begin 
			nextState <= START_SEND; 
		end else begin
			nextState <= IDLE;
		end
	end  
	
	START_SEND: begin
		if (start_send == 1'b1)begin 
			nextState <= SEND_BIT;
		end else begin
			nextState <= START_SEND;
		end
	end
	
	SEND_BIT: begin 
		if (byte_send == 1'b1) begin
			nextState <= STOP;
		end else begin
			nextState <= SEND_BIT;
		end
	end

	STOP: begin 
			nextState <= IDLE;
	end 

	default: begin
		nextState <= IDLE;
	end
	endcase
end 

//state machine output
always @ (state, start_trans, byte_send)
begin 

	case (state) 
	
	IDLE: begin
		enable_start 	= 1'b0;
		enable_count 	= 1'b0;
		tx_state 	= 1'b0;
		tx_enable 	= 1'b0;
	end

	START_SEND: begin
		enable_start 	= 1'b1;
		enable_count 	= 1'b0;
		tx_state 	= 1'b1;
		tx_enable	= 1'b1;
	end
	
	SEND_BIT: begin 
		enable_start	= 1'b0;
		enable_count 	= 1'b1;
		tx_state 	= 1'b1;
		tx_enable 	= 1'b1;
	end

	STOP: begin 
		enable_start 	= 1'b0;
		enable_count 	= 1'b0;
		tx_state 	= 1'b0;
		tx_enable 	= 1'b1;
	end
	
	default: begin
		enable_start 	= 1'b0;
		enable_count 	= 1'b0;
		tx_state 	= 1'b0;
		tx_enable 	= 1'b0;
	end
	endcase
end
	
endmodule
