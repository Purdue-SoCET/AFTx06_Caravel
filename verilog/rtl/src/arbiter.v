module arbiter (
	HCLK,
	HRESETn,
	HTRANS,
	HMASTLOCK,
	HREADY,
	ARB_SEL,
	ARB_SEL_PREV,
	MASTER_SEL,
	MASTER_SEL_PREV
);
	parameter ARBITRATION = "HIGH";
	parameter MM = 2;
	input HCLK;
	input HRESETn;
	input [(MM * 2) - 1:0] HTRANS;
	input [MM - 1:0] HMASTLOCK;
	input [MM - 1:0] HREADY;
	output reg [MM - 1:0] ARB_SEL;
	output reg [MM - 1:0] ARB_SEL_PREV;
	output reg [$clog2(MM) - 1:0] MASTER_SEL;
	output reg [$clog2(MM) - 1:0] MASTER_SEL_PREV;
	reg [MM - 1:0] ARB_SEL_PREV_n;
	reg [$clog2(MM) - 1:0] MASTER_SEL_PREV_n;
	always @(posedge HCLK or negedge HRESETn)
		if (!HRESETn) begin
			ARB_SEL_PREV <= 'b1;
			MASTER_SEL_PREV <= {$clog2(MM) {1'sb0}};
		end
		else begin
			ARB_SEL_PREV <= ARB_SEL_PREV_n;
			MASTER_SEL_PREV <= MASTER_SEL_PREV_n;
		end
	always @(*) begin
		ARB_SEL_PREV_n = ARB_SEL_PREV;
		MASTER_SEL_PREV_n = MASTER_SEL_PREV;
		if (HREADY[MASTER_SEL_PREV]) begin
			ARB_SEL_PREV_n = ARB_SEL;
			MASTER_SEL_PREV_n = MASTER_SEL;
		end
	end
	localparam ahbl_bus_mux_defines_BUSY = 2'b01;
	localparam ahbl_bus_mux_defines_SEQ = 2'b11;
	always @(*) begin
		ARB_SEL = ARB_SEL_PREV;
		MASTER_SEL = MASTER_SEL_PREV;
		if (((HMASTLOCK[MASTER_SEL_PREV] == 'b1) || (HTRANS[MASTER_SEL_PREV * 2+:2] == ahbl_bus_mux_defines_BUSY)) || (HTRANS[MASTER_SEL_PREV * 2+:2] == ahbl_bus_mux_defines_SEQ)) begin
			ARB_SEL = ARB_SEL_PREV;
			MASTER_SEL = MASTER_SEL_PREV;
		end
		else begin
			MASTER_SEL = {$clog2(MM) {1'sb0}};
			begin : sv2v_autoblock_19
				reg signed [31:0] i;
				for (i = 0; i < MM; i = i + 1)
					if (HTRANS[(i * 2) + 1] != 1'b0)
						MASTER_SEL = i;
			end
			ARB_SEL = 'b1 << MASTER_SEL;
		end
	end
endmodule
