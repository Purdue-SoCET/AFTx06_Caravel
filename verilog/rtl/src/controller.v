module controller (
	CLK,
	nRST,
	mode,
	EN,
	op_complete,
	w_done,
	SS_IN,
	SS_OUT,
	shifter_en,
	shifter_load,
	counter_en,
	shifter_rst,
	counter_rst,
	TX_REN,
	RX_WEN
);
	input wire CLK;
	input wire nRST;
	input wire mode;
	input wire EN;
	input wire op_complete;
	input wire w_done;
	input wire SS_IN;
	output wire SS_OUT;
	output reg shifter_en;
	output reg shifter_load;
	output reg counter_en;
	output reg shifter_rst;
	output reg counter_rst;
	output reg TX_REN;
	output reg RX_WEN;
	localparam MASTER = 1'b1;
	localparam SLAVE = 1'b0;
	reg [31:0] state;
	reg [31:0] state_n;
	localparam [31:0] IDLE = 0;
	always @(posedge CLK or negedge nRST)
		if (!nRST)
			state <= IDLE;
		else
			state <= state_n;
	localparam [31:0] MASTER_BUFFER = 5;
	localparam [31:0] MASTER_TXN = 4;
	localparam [31:0] SLAVE_BUFFER = 3;
	localparam [31:0] SLAVE_TXN = 2;
	localparam [31:0] SLAVE_WAIT = 1;
	always @(*) begin : NEXT_STATE
		state_n = state;
		casez (state)
			IDLE:
				if (EN && (mode == MASTER))
					state_n = MASTER_TXN;
				else if (EN && (mode == SLAVE))
					state_n = SLAVE_WAIT;
			SLAVE_WAIT:
				if (~SS_IN)
					state_n = SLAVE_TXN;
			SLAVE_TXN:
				if (w_done)
					state_n = SLAVE_BUFFER;
				else if (SS_IN)
					state_n = SLAVE_BUFFER;
			SLAVE_BUFFER:
				if (SS_IN)
					state_n = IDLE;
				else
					state_n = SLAVE_TXN;
			MASTER_TXN:
				if (w_done)
					state_n = MASTER_BUFFER;
				else if (op_complete)
					state_n = IDLE;
			MASTER_BUFFER:
				if (op_complete)
					state_n = IDLE;
				else
					state_n = MASTER_TXN;
		endcase
	end
	assign SS_OUT = !((state == MASTER_TXN) || (state == MASTER_BUFFER));
	always @(*) begin : OUTPUT_LOGIC
		shifter_en = 1'b0;
		shifter_load = 1'b0;
		counter_en = 1'b0;
		shifter_rst = 1'b0;
		counter_rst = 1'b0;
		TX_REN = 1'b0;
		RX_WEN = 1'b0;
		casez (state)
			IDLE:
				if (EN) begin
					TX_REN = 1;
					shifter_load = 1;
				end
				else begin
					shifter_rst = 1;
					counter_rst = 1;
				end
			SLAVE_TXN, MASTER_TXN: begin
				shifter_en = 1;
				counter_en = 1;
			end
			SLAVE_BUFFER, MASTER_BUFFER: begin
				TX_REN = 1;
				RX_WEN = 1;
				shifter_load = 1;
				shifter_en = 0;
				counter_en = 0;
			end
		endcase
	end
endmodule
