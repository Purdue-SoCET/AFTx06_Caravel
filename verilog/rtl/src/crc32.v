module crc32 (
	CLK,
	nRST,
	in_byte,
	reset,
	new_byte,
	done,
	result
);
	input wire CLK;
	input wire nRST;
	input wire [7:0] in_byte;
	input wire reset;
	input wire new_byte;
	output wire done;
	localparam rv32i_types_pkg_WORD_SIZE = 32;
	output wire [31:0] result;
	parameter POLY = 32'h04c11db7;
	parameter POLY_REV = 32'hedb88320;
	parameter POLY_REV_REC = 32'h82608edb;
	wire [31:0] next_crc;
	reg [31:0] curr_crc;
	wire [31:0] mask;
	wire update;
	reg [3:0] count;
	assign result = ~curr_crc;
	assign done = (count[3] & ~new_byte) | reset;
	assign update = ~count[3];
	assign mask = {32 {curr_crc[0]}};
	assign next_crc = (curr_crc >> 1) ^ (POLY_REV & mask);
	always @(posedge CLK or negedge nRST)
		if (~nRST)
			curr_crc <= {32 {1'sb1}};
		else if (reset)
			curr_crc <= {32 {1'sb1}};
		else if (new_byte)
			curr_crc <= curr_crc ^ {24'h000000, in_byte};
		else if (update)
			curr_crc <= next_crc;
	always @(posedge CLK or negedge nRST)
		if (~nRST)
			count <= 4'b1000;
		else if (new_byte)
			count <= 4'b0000;
		else if (update)
			count <= count + 1;
endmodule
