module rdata_arbiter (
	rdata1,
	rdata2,
	rdata3,
	ren,
	addr_valid1,
	addr_valid2,
	addr_valid3,
	rdata
);
	input wire [31:0] rdata1;
	input wire [31:0] rdata2;
	input wire [31:0] rdata3;
	input wire ren;
	input wire addr_valid1;
	input wire addr_valid2;
	input wire addr_valid3;
	output reg [31:0] rdata;
	always @(*)
		if (addr_valid1 & ren)
			rdata = rdata1;
		else if (addr_valid2 & ren)
			rdata = rdata2;
		else if (addr_valid3 & ren)
			rdata = rdata3;
		else
			rdata = 'bz;
endmodule
