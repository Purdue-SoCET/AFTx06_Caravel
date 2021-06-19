module busy_tester (
	clk,
	n_rst,
	SDA_sync,
	SCL_sync,
	bus_busy
);
	input clk;
	input n_rst;
	input SDA_sync;
	input SCL_sync;
	output bus_busy;
	reg [2:0] state;
	reg [2:0] nextState;
	localparam [2:0] ALMOST_BUSY = 3'b001;
	localparam [2:0] ALMOST_BUSY2 = 3'b010;
	localparam [2:0] ALMOST_IDLE = 3'b101;
	localparam [2:0] ALMOST_IDLE2 = 3'b110;
	localparam [2:0] BUSY = 3'b100;
	localparam [2:0] IDLE = 3'b000;
	always @(*) begin
		nextState = state;
		case (state)
			IDLE: nextState = (SCL_sync & SDA_sync ? ALMOST_BUSY : IDLE);
			ALMOST_BUSY:
				if (SCL_sync & SDA_sync)
					nextState = ALMOST_BUSY;
				else if (SCL_sync & !SDA_sync)
					nextState = ALMOST_BUSY2;
				else
					nextState = IDLE;
			ALMOST_BUSY2: nextState = (SCL_sync & !SDA_sync ? BUSY : IDLE);
			BUSY: nextState = (SCL_sync & !SDA_sync ? ALMOST_IDLE : BUSY);
			ALMOST_IDLE:
				if (SCL_sync & !SDA_sync)
					nextState = ALMOST_IDLE;
				else if (SCL_sync & SDA_sync)
					nextState = ALMOST_IDLE2;
				else
					nextState = BUSY;
			ALMOST_IDLE2: nextState = (SCL_sync & SDA_sync ? IDLE : BUSY);
		endcase
	end
	always @(posedge clk or negedge n_rst)
		if (!n_rst)
			state <= IDLE;
		else
			state <= nextState;
	assign bus_busy = state[2];
endmodule
