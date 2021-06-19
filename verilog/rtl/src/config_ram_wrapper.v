module config_ram_wrapper (
	CLK,
	nRST,
	wdata,
	addr,
	byte_en,
	wen,
	ren,
	rdata,
	busy
);
	parameter N_BYTES = 4;
	parameter DEPTH = 256;
	parameter LAT = 0;
	parameter ADDR_BITS = $clog2(DEPTH);
	parameter N_BITS = N_BYTES * 8;
	input wire CLK;
	input wire nRST;
	input wire [N_BITS - 1:0] wdata;
	input wire [ADDR_BITS - 1:0] addr;
	input wire [N_BYTES - 1:0] byte_en;
	input wire wen;
	input wire ren;
	output wire [N_BITS - 1:0] rdata;
	output wire busy;
	ram_sim_model #(
		.LAT(0),
		.ENDIANNESS("little"),
		.N_BYTES(N_BYTES),
		.DEPTH(DEPTH)
	) v_lat_ram(
		.CLK(CLK),
		.nRST(nRST),
		.wdata_in(wdata),
		.addr_in(addr),
		.byte_en_in(byte_en),
		.wen_in(wen),
		.ren_in(ren),
		.rdata_out(rdata),
		.busy_out(busy)
	);
endmodule
