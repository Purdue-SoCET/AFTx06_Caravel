module aphase_cache (
	HCLK,
	HRESETn,
	ARB_SEL,
	ARB_SEL_PREV,
	HREADY_in,
	HREADY_out,
	upstream_in,
	downstream_out
);
	input HCLK;
	input HRESETn;
	input ARB_SEL;
	input ARB_SEL_PREV;
	input HREADY_in;
	output reg HREADY_out;
	input wire [45:0] upstream_in;
	output wire [45:0] downstream_out;
	reg valid;
	reg valid_n;
	reg [45:0] cache;
	reg [45:0] cache_n;
	always @(posedge HCLK or negedge HRESETn)
		if (!HRESETn) begin
			valid <= 1'b0;
			cache <= {46 {1'sb0}};
		end
		else begin
			valid <= valid_n;
			cache <= cache_n;
		end
	localparam ahbl_bus_mux_defines_IDLE = 2'b00;
	always @(*) begin : APHASE
		if (ARB_SEL && HREADY_in) begin
			valid_n = 'b0;
			cache_n = {46 {1'sb0}};
		end
		else if ((upstream_in[2-:2] != ahbl_bus_mux_defines_IDLE) && !valid) begin
			valid_n = 'b1;
			cache_n = upstream_in;
		end
		else begin
			cache_n = cache;
			valid_n = valid;
		end
	end
	always @(*) begin : SELECT
		if (!ARB_SEL_PREV && valid)
			HREADY_out = 1'b0;
		else if (ARB_SEL_PREV)
			HREADY_out = HREADY_in;
		else
			HREADY_out = 1'b1;
	end
	assign downstream_out = (valid ? cache : upstream_in);
endmodule
