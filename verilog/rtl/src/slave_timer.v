module slave_timer (
	clk,
	n_rst,
	start,
	stop,
	rising_edge,
	falling_edge,
	byte_received,
	ack_prep,
	ack_check,
	ack_done
);
	input wire clk;
	input wire n_rst;
	input wire start;
	input wire stop;
	input wire rising_edge;
	input wire falling_edge;
	output reg byte_received;
	output reg ack_prep;
	output reg ack_check;
	output reg ack_done;
	reg [3:0] state;
	reg [3:0] next_state;
	reg temp_byte_received;
	reg temp_ack_prep;
	reg temp_ack_check;
	reg temp_ack_done;
	localparam [3:0] IDLE = 0;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			state <= IDLE;
			byte_received <= 1'b0;
			ack_prep <= 1'b0;
			ack_check <= 1'b0;
			ack_done <= 1'b0;
		end
		else begin
			state <= next_state;
			byte_received <= temp_byte_received;
			ack_prep <= temp_ack_prep;
			ack_check <= temp_ack_check;
			ack_done <= temp_ack_done;
		end
	localparam [3:0] CHECK = 11;
	localparam [3:0] DONE = 12;
	localparam [3:0] PREP = 10;
	localparam [3:0] READ_1 = 2;
	localparam [3:0] READ_2 = 3;
	localparam [3:0] READ_3 = 4;
	localparam [3:0] READ_4 = 5;
	localparam [3:0] READ_5 = 6;
	localparam [3:0] READ_6 = 7;
	localparam [3:0] READ_7 = 8;
	localparam [3:0] READ_8 = 9;
	localparam [3:0] START = 1;
	always @(state or start or stop or rising_edge or falling_edge) begin
		next_state = state;
		case (state)
			IDLE:
				if (start == 1'b1)
					next_state = START;
			START:
				if (rising_edge == 1'b1)
					next_state = READ_1;
			READ_1:
				if (start == 1'b1)
					next_state = START;
				else if (rising_edge == 1'b1)
					next_state = READ_2;
			READ_2:
				if (start == 1'b1)
					next_state = START;
				else if (rising_edge == 1'b1)
					next_state = READ_3;
			READ_3:
				if (rising_edge == 1'b1)
					next_state = READ_4;
			READ_4:
				if (rising_edge == 1'b1)
					next_state = READ_5;
			READ_5:
				if (rising_edge == 1'b1)
					next_state = READ_6;
			READ_6:
				if (rising_edge == 1'b1)
					next_state = READ_7;
			READ_7:
				if (rising_edge == 1'b1)
					next_state = READ_8;
			READ_8:
				if (falling_edge == 1'b1)
					next_state = PREP;
			PREP:
				if (rising_edge == 1'b1)
					next_state = CHECK;
			CHECK:
				if (falling_edge == 1'b1)
					next_state = DONE;
			DONE:
				if (stop == 1'b1)
					next_state = IDLE;
				else if (start == 1'b1)
					next_state = START;
				else if (rising_edge == 1'b1)
					next_state = READ_1;
		endcase
	end
	always @(state) begin
		temp_byte_received = 1'b0;
		temp_ack_prep = 1'b0;
		temp_ack_check = 1'b0;
		temp_ack_done = 1'b0;
		case (state)
			PREP: begin
				temp_byte_received = 1'b1;
				temp_ack_prep = 1'b1;
			end
			CHECK: begin
				temp_byte_received = 1'b1;
				temp_ack_check = 1'b1;
			end
			DONE: temp_ack_done = 1'b1;
			default: begin
				temp_byte_received = 1'b0;
				temp_ack_prep = 1'b0;
				temp_ack_check = 1'b0;
				temp_ack_done = 1'b0;
			end
		endcase
	end
endmodule
