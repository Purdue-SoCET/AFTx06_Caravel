module counters (
	clk,
	n_rst,
	clear,
	count_enable,
	count_clk,
	bytelen,
	OPC,
	w_done
);
	input wire clk;
	input wire n_rst;
	input wire clear;
	input wire count_enable;
	input wire count_clk;
	input wire [31:0] bytelen;
	output wire OPC;
	output wire w_done;
	wire byte_cnt_en;
	localparam spi_type_pkg_WORD_W = 32;
	wire [31:0] bytes_trs;
	wire [2:0] word_rollover_val;
	assign w_done = byte_cnt_en;
	flex_counter_spi bitcnt(
		.clk(clk),
		.n_rst(n_rst),
		.clear(w_done),
		.count_enable(count_enable & count_clk),
		.rollover_val(4'd8),
		.rollover_flag(byte_cnt_en)
	);
	flex_counter_spi #(.NUM_CNT_BITS(32)) bytecnt(
		.clk(clk),
		.n_rst(n_rst),
		.clear(OPC),
		.count_enable(byte_cnt_en),
		.rollover_val(bytelen),
		.count_out(bytes_trs),
		.rollover_flag(OPC)
	);
endmodule
