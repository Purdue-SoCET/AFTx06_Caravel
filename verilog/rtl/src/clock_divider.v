module clock_divider (
	HCLK,
	n_RST,
	PRE,
	tim_clk
);
	input wire HCLK;
	input wire n_RST;
	input wire [2:0] PRE;
	output reg tim_clk;
	reg clkdiv2;
	reg clkdiv4;
	reg clkdiv8;
	reg clkdiv16;
	reg clkdiv32;
	reg clkdiv64;
	reg clkdiv128;
	always @(posedge HCLK or negedge n_RST)
		if (!n_RST)
			clkdiv2 <= 0;
		else
			clkdiv2 <= ~clkdiv2;
	always @(posedge clkdiv2 or negedge n_RST)
		if (!n_RST)
			clkdiv4 <= 0;
		else
			clkdiv4 <= ~clkdiv4;
	always @(posedge clkdiv4 or negedge n_RST)
		if (!n_RST)
			clkdiv8 <= 0;
		else
			clkdiv8 <= ~clkdiv8;
	always @(posedge clkdiv8 or negedge n_RST)
		if (!n_RST)
			clkdiv16 <= 0;
		else
			clkdiv16 <= ~clkdiv16;
	always @(posedge clkdiv16 or negedge n_RST)
		if (!n_RST)
			clkdiv32 <= 0;
		else
			clkdiv32 <= ~clkdiv32;
	always @(posedge clkdiv32 or negedge n_RST)
		if (!n_RST)
			clkdiv64 <= 0;
		else
			clkdiv64 <= ~clkdiv64;
	always @(posedge clkdiv64 or negedge n_RST)
		if (!n_RST)
			clkdiv128 <= 0;
		else
			clkdiv128 <= ~clkdiv128;
	always @(*)
		case (PRE)
			3'b000: tim_clk = HCLK;
			3'b001: tim_clk = clkdiv2;
			3'b010: tim_clk = clkdiv4;
			3'b011: tim_clk = clkdiv8;
			3'b100: tim_clk = clkdiv16;
			3'b101: tim_clk = clkdiv32;
			3'b110: tim_clk = clkdiv64;
			3'b111: tim_clk = clkdiv128;
		endcase
endmodule
