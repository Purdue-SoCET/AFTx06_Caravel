module SOC_RAM (
	clk,
	n_rst,
	bot_active,
	w_data,
	addr,
	w_en,
	byte_en,
	r_data
);
	parameter integer ADDRBIT = 16;
	parameter integer DATABIT = 32;
	parameter integer BOTTOMADDR = 0;
	parameter integer TOPADDR = 65535;
	input wire clk;
	input wire n_rst;
	input wire bot_active;
	input wire [DATABIT - 1:0] w_data;
	input wire [ADDRBIT - 1:0] addr;
	input wire w_en;
	input wire [3:0] byte_en;
	output reg [DATABIT - 1:0] r_data;
	reg [DATABIT - 1:0] RAM [TOPADDR:BOTTOMADDR];
	reg [ADDRBIT - 1:0] address_reg;
	reg [DATABIT - 1:0] w_data_reg;
	reg w_en_reg;
	reg [3:0] byte_en_reg;
	reg bot_active_reg;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0)
			RAM[0] <= 32'b00000000000000000000000000000000;
		else begin
			if (w_en_reg && bot_active_reg) begin
				RAM[address_reg][7:0] <= (byte_en_reg[0] ? w_data_reg[7:0] : RAM[address_reg][7:0]);
				RAM[address_reg][15:8] <= (byte_en_reg[1] ? w_data_reg[15:8] : RAM[address_reg][15:8]);
				RAM[address_reg][23:16] <= (byte_en_reg[2] ? w_data_reg[23:16] : RAM[address_reg][23:16]);
				RAM[address_reg][31:24] <= (byte_en_reg[3] ? w_data_reg[31:24] : RAM[address_reg][31:24]);
			end
			address_reg <= addr;
			w_data_reg <= w_data;
			w_en_reg <= w_en;
			byte_en_reg <= byte_en;
			bot_active_reg <= bot_active;
		end
	wire [DATABIT:1] sv2v_tmp_D6081;
	assign sv2v_tmp_D6081 = (bot_active_reg ? RAM[address_reg] : 32'hdeadbeef);
	always @(*) r_data = sv2v_tmp_D6081;
endmodule
