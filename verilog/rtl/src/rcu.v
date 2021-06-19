/*
File name:   rcu.v
Created:     2/28/2013
Author:      Chuan Yean Tan
Version:	1.0 
Description: receive control unit of UART

Extracted from ECE337

*/

module rcu(			input clk, n_Rst, 
				input start_detected, 
				input framing_error,
				input packet_done,
				input start_rcv,
				output reg sbc_clear, 
				output reg sbc_enable, 
				output reg enable_timer,
				output reg data_ready,
				output reg load_buffer,
				output reg byte_rcv);

//TOP LEVEL STATES
parameter [2:0]
		IDLE		= 	3'd0,
		CLR_ERR 	= 	3'd1,
		LOAD_DATA 	= 	3'd2,
		CHECK 		= 	3'd3,
		TEMP 		= 	3'd4,
		LOAD_BUF 	= 	3'd5,
		DAT_READY	=	3'd6;

reg [4:0] state;
reg [4:0] nextState;
		
//next state logic
always @ (posedge clk, negedge n_Rst) begin 

	if (n_Rst == 1'd0) begin 
		state <= IDLE; 
	end else begin 
		state <= nextState;
	end
end

//state machines transition
always @ (state, start_detected,packet_done,framing_error,start_rcv)begin
	case (state)
	
	IDLE: begin 
		if (start_detected == 1'b1 && start_rcv == 1'b1) begin 
			nextState <= CLR_ERR; 
		end else begin
			nextState <= IDLE;
		end
	end  
	
	CLR_ERR: begin 
		nextState <= LOAD_DATA;
	end

	LOAD_DATA: begin 
		if (packet_done == 1'b1) begin 
			nextState <= CHECK;
		end else begin 
			nextState <= LOAD_DATA;
		end 
	end 
	
	CHECK: begin
		nextState <= TEMP;
	end
	
	TEMP: begin 
		if (framing_error == 1'b1) begin 
			nextState <= IDLE;
		end else begin 
			nextState <= LOAD_BUF;
		end	
	end

	LOAD_BUF: begin 
		nextState <= DAT_READY;
	end

	DAT_READY: begin
		nextState <= IDLE;
	end
	
	default: begin
		nextState <= IDLE;
	end
	endcase
end 

//state machine output
always @ (state, start_detected,packet_done,framing_error)
begin 

	case (state) 
	
	IDLE: begin
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	CLR_ERR: begin 
		sbc_clear 	<= 1'b1;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	LOAD_DATA: begin 
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b1;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	CHECK: begin 
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b1;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	TEMP: begin 
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	LOAD_BUF: begin 
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b1;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end

	DAT_READY: begin 
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b1;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b1;
		byte_rcv	<= 1'b1;
	end
	
	default: begin
		sbc_clear 	<= 1'b0;
		sbc_enable 	<= 1'b0;
		load_buffer 	<= 1'b0;
		enable_timer 	<= 1'b0;
		data_ready 	<= 1'b0;
		byte_rcv	<= 1'b0;
	end
	endcase
end
	
endmodule
