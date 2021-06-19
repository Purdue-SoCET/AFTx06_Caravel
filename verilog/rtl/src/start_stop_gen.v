module start_stop_gen (
	clk,
	n_rst,
	bus_busy,
	start,
	stop,
	clock_div,
	SDA,
	SCL,
	done
);
	input clk;
	input n_rst;
	input bus_busy;
	input start;
	input stop;
	input wire [31:0] clock_div;
	output reg SDA;
	output reg SCL;
	output reg done;
	localparam COUNTER_BITS = 9;
	wire enabled;
	wire wait_expired;
	reg [4:0] state;
	reg [4:0] next_state;
	assign enabled = start | stop;
	flex_counter_master #(.NUM_CNT_BITS(32)) counter(
		.clk(clk),
		.n_rst(n_rst),
		.rollover_val(clock_div),
		.rollover_flag(wait_expired),
		.count_enable(enabled),
		.clear(!enabled)
	);
	localparam [4:0] IDLE_BUSY = 1;
	localparam [4:0] IDLE_NOT_BUSY = 0;
	localparam [4:0] START1 = 2;
	localparam [4:0] START2 = 3;
	localparam [4:0] START3 = 4;
	localparam [4:0] START_DONE = 5;
	localparam [4:0] STOP1 = 6;
	localparam [4:0] STOP2 = 7;
	localparam [4:0] STOP3 = 8;
	localparam [4:0] STOP_DONE = 9;
	always @(*) begin
		next_state = state;
		case (state)
			IDLE_NOT_BUSY:
				if (start)
					next_state = START1;
				else if (stop)
					next_state = STOP1;
				else if (bus_busy)
					next_state = IDLE_BUSY;
			IDLE_BUSY:
				if (start)
					next_state = START1;
				else if (stop)
					next_state = STOP1;
				else if (!bus_busy)
					next_state = IDLE_NOT_BUSY;
			START1: next_state = (wait_expired ? START2 : START1);
			START2: next_state = (wait_expired ? START3 : START2);
			START3: next_state = (wait_expired ? START_DONE : START3);
			STOP1: next_state = (wait_expired ? STOP2 : STOP1);
			STOP2: next_state = (wait_expired ? STOP3 : STOP2);
			STOP3: next_state = (wait_expired ? STOP_DONE : STOP3);
		endcase
		if (!enabled & bus_busy)
			next_state = IDLE_BUSY;
		else if (!enabled & !bus_busy)
			next_state = IDLE_NOT_BUSY;
	end
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			state <= IDLE_NOT_BUSY;
		else
			state <= next_state;
	always @(*) begin
		SDA = 1;
		SCL = 1;
		done = 0;
		case (state)
			IDLE_NOT_BUSY: begin
				SDA = 1;
				SCL = 1;
				done = 0;
			end
			IDLE_BUSY: begin
				SDA = 1;
				SCL = 0;
				done = 0;
			end
			START1: begin
				SDA = 1;
				SCL = 1;
				done = 0;
			end
			START2: begin
				SDA = 0;
				SCL = 1;
				done = 0;
			end
			START3: begin
				SDA = 0;
				SCL = 0;
				done = 0;
			end
			START_DONE: begin
				SDA = 0;
				SCL = 0;
				done = 1;
			end
			STOP1: begin
				SDA = 0;
				SCL = 0;
				done = 0;
			end
			STOP2: begin
				SDA = 0;
				SCL = 1;
				done = 0;
			end
			STOP3: begin
				SDA = 1;
				SCL = 1;
				done = 0;
			end
			STOP_DONE: begin
				SDA = 1;
				SCL = 1;
				done = 1;
			end
		endcase
	end
endmodule
