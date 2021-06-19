module master_timer (
	clk,
	n_rst,
	timer_active,
	clock_div,
	direction,
	should_nack,
	SDA_sync,
	SCL_sync,
	SDA_out,
	SCL_out,
	shift_strobe,
	byte_complete,
	ack_gen,
	ack,
	abort
);
	input wire clk;
	input wire n_rst;
	input wire timer_active;
	input wire [31:0] clock_div;
	input wire direction;
	input wire should_nack;
	input wire SDA_sync;
	input wire SCL_sync;
	input wire SDA_out;
	output reg SCL_out;
	output reg shift_strobe;
	output reg byte_complete;
	output reg ack_gen;
	output reg ack;
	output reg abort;
	localparam STANDARD_MODE_LOW = 300;
	localparam STANDARD_MODE_HIGH = 300;
	localparam FAST_MODE_LOW = 90;
	localparam FAST_MODE_HIGH = 60;
	localparam FAST_PLUS_LOW = 36;
	localparam FAST_PLUS_HIGH = 24;
	reg [4:0] next_state;
	reg [4:0] state;
	wire [3:0] current_bit;
	reg chk_ack;
	reg timer_reset;
	reg inc_bit;
	wire half_cycle;
	reg shift_strobe_next;
	flex_counter_master #(.NUM_CNT_BITS(4)) bit_counter(
		.clk(clk),
		.n_rst(n_rst),
		.clear(!timer_active),
		.count_enable(inc_bit),
		.rollover_val(4'd15),
		.count_out(current_bit)
	);
	flex_counter_master #(.NUM_CNT_BITS(32)) cycle_counter(
		.clk(clk),
		.n_rst(n_rst),
		.clear(timer_reset),
		.count_enable(timer_active),
		.rollover_val(clock_div),
		.rollover_flag(half_cycle)
	);
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			ack <= 1;
		else if (chk_ack)
			ack <= SDA_sync;
		else
			ack <= ack;
	localparam [4:0] ABORT = 10;
	localparam [4:0] ACK_HIGH = 8;
	localparam [4:0] ACK_R = 7;
	localparam [4:0] ACK_T_HIGH = 13;
	localparam [4:0] ACK_T_LOW = 11;
	localparam [4:0] ACK_T_SYNC = 12;
	localparam [4:0] CHK_BIT = 2;
	localparam [4:0] CLK_HIGH = 9;
	localparam [4:0] CLK_LOW = 4;
	localparam [4:0] CLK_SYNC = 5;
	localparam [4:0] DONE = 18;
	localparam [4:0] IDLE = 0;
	localparam [4:0] INC_BIT = 1;
	localparam [4:0] NACK_T_HIGH = 16;
	localparam [4:0] NACK_T_LOW = 14;
	localparam [4:0] NACK_T_SYNC = 15;
	localparam [0:0] RX = 0;
	localparam [4:0] R_SHIFT = 6;
	localparam [0:0] TX = 1;
	localparam [4:0] T_SHIFT = 3;
	localparam [4:0] WAIT = 17;
	always @(*) begin
		next_state = state;
		case (state)
			IDLE: next_state = INC_BIT;
			INC_BIT: next_state = CHK_BIT;
			CHK_BIT:
				if (current_bit == 10)
					next_state = WAIT;
				else if ((current_bit == 9) && (direction == RX))
					next_state = (should_nack ? NACK_T_LOW : ACK_T_LOW);
				else if ((current_bit == 1) || (direction == RX))
					next_state = CLK_LOW;
				else
					next_state = T_SHIFT;
			T_SHIFT: next_state = CLK_LOW;
			CLK_LOW: next_state = (half_cycle ? CLK_SYNC : CLK_LOW);
			CLK_SYNC:
				if (SCL_sync)
					if (direction == RX)
						next_state = R_SHIFT;
					else if (current_bit == 9)
						next_state = ACK_R;
					else
						next_state = CLK_HIGH;
			R_SHIFT: next_state = CLK_HIGH;
			ACK_R: next_state = ACK_HIGH;
			CLK_HIGH:
				if (((direction == TX) && SDA_out) && !SDA_sync)
					next_state = ABORT;
				else if (half_cycle | !SCL_sync)
					next_state = INC_BIT;
			ACK_HIGH:
				if (half_cycle | !SCL_sync)
					next_state = INC_BIT;
			ACK_T_LOW: next_state = (half_cycle ? ACK_T_SYNC : ACK_T_LOW);
			ACK_T_SYNC: next_state = (SCL_sync ? ACK_T_HIGH : ACK_T_SYNC);
			ACK_T_HIGH:
				if (!SCL_sync | half_cycle)
					next_state = WAIT;
			NACK_T_LOW: next_state = (half_cycle ? NACK_T_SYNC : NACK_T_LOW);
			NACK_T_SYNC: next_state = (SCL_sync ? NACK_T_HIGH : NACK_T_SYNC);
			NACK_T_HIGH:
				if (!SCL_sync | half_cycle)
					next_state = WAIT;
			WAIT: next_state = (half_cycle ? DONE : WAIT);
		endcase
		if (!timer_active)
			next_state = IDLE;
	end
	always @(posedge clk or negedge n_rst)
		if (!n_rst) begin
			state <= IDLE;
			shift_strobe <= 0;
		end
		else begin
			state <= next_state;
			shift_strobe <= shift_strobe_next;
		end
	always @(*) begin
		abort = 0;
		shift_strobe_next = 0;
		SCL_out = 0;
		byte_complete = 0;
		chk_ack = 0;
		inc_bit = 0;
		ack_gen = 0;
		timer_reset = 1;
		case (state)
			IDLE:
				;
			INC_BIT: inc_bit = 1;
			CHK_BIT:
				;
			T_SHIFT: shift_strobe_next = 1;
			CLK_LOW: timer_reset = 0;
			R_SHIFT: begin
				shift_strobe_next = 1;
				SCL_out = 1;
			end
			ACK_R: begin
				chk_ack = 1;
				SCL_out = 1;
			end
			CLK_SYNC: SCL_out = 1;
			CLK_HIGH: begin
				SCL_out = 1;
				timer_reset = 0;
			end
			ACK_HIGH: begin
				SCL_out = 1;
				timer_reset = 0;
			end
			ABORT: begin
				SCL_out = 1;
				abort = 1;
			end
			ACK_T_LOW: begin
				ack_gen = 1;
				timer_reset = 0;
			end
			ACK_T_SYNC: begin
				ack_gen = 1;
				SCL_out = 1;
			end
			ACK_T_HIGH: begin
				ack_gen = 1;
				SCL_out = 1;
				timer_reset = 0;
			end
			NACK_T_LOW: timer_reset = 0;
			NACK_T_SYNC: SCL_out = 1;
			NACK_T_HIGH: begin
				SCL_out = 1;
				timer_reset = 0;
			end
			WAIT: timer_reset = 0;
			DONE: byte_complete = 1;
		endcase
	end
endmodule
