module dmem_extender (
	dmem_in,
	load_type,
	byte_en,
	ext_out
);
	localparam rv32i_types_pkg_WORD_SIZE = 32;
	input wire [31:0] dmem_in;
	localparam rv32i_types_pkg_LD_W = 3;
	input wire [2:0] load_type;
	input wire [3:0] byte_en;
	output reg [31:0] ext_out;
	localparam [2:0] rv32i_types_pkg_LB = 3'b000;
	localparam [2:0] rv32i_types_pkg_LBU = 3'b100;
	localparam [2:0] rv32i_types_pkg_LH = 3'b001;
	localparam [2:0] rv32i_types_pkg_LHU = 3'b101;
	localparam [2:0] rv32i_types_pkg_LW = 3'b010;
	always @(*)
		casez (load_type)
			rv32i_types_pkg_LB:
				casez (byte_en)
					4'b0001: ext_out = $signed(dmem_in[7:0]);
					4'b0010: ext_out = $signed(dmem_in[15:8]);
					4'b0100: ext_out = $signed(dmem_in[23:16]);
					4'b1000: ext_out = $signed(dmem_in[31:24]);
					default: ext_out = {32 {1'sb0}};
				endcase
			rv32i_types_pkg_LBU:
				casez (byte_en)
					4'b0001: ext_out = dmem_in[7:0];
					4'b0010: ext_out = dmem_in[15:8];
					4'b0100: ext_out = dmem_in[23:16];
					4'b1000: ext_out = dmem_in[31:24];
					default: ext_out = {32 {1'sb0}};
				endcase
			rv32i_types_pkg_LH:
				casez (byte_en)
					4'b0011: ext_out = $signed(dmem_in[15:0]);
					4'b1100: ext_out = $signed(dmem_in[31:16]);
					default: ext_out = {32 {1'sb0}};
				endcase
			rv32i_types_pkg_LHU:
				casez (byte_en)
					4'b0011: ext_out = dmem_in[15:0];
					4'b1100: ext_out = dmem_in[31:16];
					default: ext_out = {32 {1'sb0}};
				endcase
			rv32i_types_pkg_LW: ext_out = dmem_in;
			default: ext_out = {32 {1'sb0}};
		endcase
endmodule
