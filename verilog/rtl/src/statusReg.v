module statusReg (
	data_in,
	clk,
	n_rst,
	clear,
	data_out
);
	input reg [12:0] data_in;
	input wire clk;
	input wire n_rst;
	input wire clear;
	output reg [12:0] data_out;
	reg [12:0] next_data_out;
	reg [1:0] next_state;
	reg [1:0] curr_state;
	localparam [1:0] IDLE = 0;
	always @(posedge clk or negedge n_rst)
		if (0 == n_rst) begin
			data_out <= {13 {1'sb0}};
			curr_state <= IDLE;
		end
		else begin
			data_out <= next_data_out;
			curr_state <= next_state;
		end
	localparam [1:0] EXIT = 3;
	localparam [1:0] WAIT1 = 1;
	localparam [1:0] WAIT2 = 2;
	always @(*) begin
		next_data_out = data_in;
		next_state = curr_state;
		case (curr_state)
			IDLE: begin
				next_data_out[0] = data_in[0] | data_out[0];
				next_data_out[2] = data_in[2] | data_out[2];
				next_data_out[3] = data_in[3] | data_out[3];
				next_data_out[5] = data_in[5] | data_out[5];
				next_data_out[6] = data_in[6] | data_out[6];
				next_data_out[9] = data_in[9] | data_out[9];
				next_data_out[12] = data_in[12] | data_out[12];
				if (clear)
					next_state = WAIT1;
			end
			WAIT1: begin
				next_data_out[0] = data_out[0];
				next_data_out[3:2] = data_out[3:2];
				next_data_out[6:5] = data_out[6:5];
				next_data_out[9] = data_out[9];
				next_data_out[12] = data_out[12];
				next_state = WAIT2;
			end
			WAIT2: begin
				next_data_out[0] = data_out[0];
				next_data_out[3:2] = data_out[3:2];
				next_data_out[6:5] = data_out[6:5];
				next_data_out[9] = data_out[9];
				next_data_out[12] = data_out[12];
				next_state = EXIT;
			end
			EXIT: begin
				next_state = IDLE;
				next_data_out[0] = 1'b0;
				next_data_out[3:2] = 2'b00;
				next_data_out[6:5] = 2'b00;
				next_data_out[9] = 1'b0;
				next_data_out[12] = 1'b0;
			end
		endcase
	end
endmodule
