module interrupt_control (
	CLK,
	nRST,
	transfer_complete,
	modf,
	rx_water,
	tx_water,
	rx_full,
	tx_empty,
	interrupt_enable,
	clear,
	interrupts
);
	input wire CLK;
	input wire nRST;
	input wire transfer_complete;
	input wire modf;
	input wire rx_water;
	input wire tx_water;
	input wire rx_full;
	input wire tx_empty;
	input wire [5:0] interrupt_enable;
	input wire [5:0] clear;
	output wire [5:0] interrupts;
	reg [5:0] interrupt_e;
	reg [5:0] interrupt_flags;
	reg [5:0] interrupt_n;
	wire [5:0] input_group;
	assign input_group = {transfer_complete, rx_water, tx_water, rx_full, tx_empty, modf};
	assign interrupts = interrupt_flags;
	always @(posedge CLK or negedge nRST)
		if (nRST == 0) begin
			interrupt_flags <= {6 {1'sb0}};
			interrupt_e <= {6 {1'sb0}};
		end
		else begin
			interrupt_flags <= interrupt_n;
			interrupt_e <= input_group;
		end
	always @(*) begin
		interrupt_n = interrupts;
		begin : sv2v_autoblock_20
			reg signed [31:0] i;
			for (i = 0; i < 6; i = i + 1)
				if (clear[i])
					interrupt_n[i] = 1'b0;
				else
					interrupt_n[i] = interrupt_flags[i] | (~interrupt_e[i] & input_group[i]);
		end
		interrupt_n = interrupt_n & interrupt_enable;
	end
endmodule
