module apbSlave (
	pclk,
	n_rst,
	pdata,
	paddr,
	penable,
	psel,
	pwrite,
	rx_data,
	rx_w_ena,
	i2c_status,
	scl,
	tx_r_ena,
	prdata,
	i2c_interrupt,
	tx_data,
	rx_almost_full,
	rx_full,
	control_out,
	address,
	clk_out,
	tx_empty
);
	input wire pclk;
	input wire n_rst;
	input wire [31:0] pdata;
	input wire [31:0] paddr;
	input wire penable;
	input wire psel;
	input wire pwrite;
	input wire [7:0] rx_data;
	input wire rx_w_ena;
	input wire [5:0] i2c_status;
	input wire scl;
	input wire tx_r_ena;
	output reg [31:0] prdata;
	output reg i2c_interrupt;
	output reg [7:0] tx_data;
	output reg rx_almost_full;
	output reg rx_full;
	output wire [10:0] control_out;
	output reg [9:0] address;
	output reg [31:0] clk_out;
	output reg tx_empty;
	reg [11:0] control;
	assign control_out = control[10:0];
	reg tx_w_ena;
	reg cr_w_ena;
	reg cr_r_ena;
	reg addr_ena;
	reg [9:0] addr_data;
	reg [1:0] rx_r_ena;
	reg [12:0] next_status;
	reg clk_div_ena;
	reg [7:0] i2c_data;
	reg tx_full;
	reg tx_almst_full;
	reg rx_empty;
	reg mid_tx_empty;
	reg mid_rx_full;
	reg [12:0] status;
	wire status_clear;
	wire [1:1] sv2v_tmp_IX10_tx_w_ena;
	always @(*) tx_w_ena = sv2v_tmp_IX10_tx_w_ena;
	wire [1:1] sv2v_tmp_IX10_cr_w_ena;
	always @(*) cr_w_ena = sv2v_tmp_IX10_cr_w_ena;
	wire [1:1] sv2v_tmp_IX10_cr_r_ena;
	always @(*) cr_r_ena = sv2v_tmp_IX10_cr_r_ena;
	wire [1:1] sv2v_tmp_IX10_addr_ena;
	always @(*) addr_ena = sv2v_tmp_IX10_addr_ena;
	wire [2:1] sv2v_tmp_IX10_rx_r_ena;
	always @(*) rx_r_ena = sv2v_tmp_IX10_rx_r_ena;
	wire [1:1] sv2v_tmp_IX10_clk_div_ena;
	always @(*) clk_div_ena = sv2v_tmp_IX10_clk_div_ena;
	address_decoder IX10(
		.pclk(pclk),
		.n_rst(n_rst),
		.paddr(paddr - 32'h80040000),
		.penable(penable),
		.psel(psel),
		.pwrite(pwrite),
		.tx_w_ena(sv2v_tmp_IX10_tx_w_ena),
		.cr_w_ena(sv2v_tmp_IX10_cr_w_ena),
		.cr_r_ena(sv2v_tmp_IX10_cr_r_ena),
		.addr_ena(sv2v_tmp_IX10_addr_ena),
		.rx_r_ena(sv2v_tmp_IX10_rx_r_ena),
		.clk_div_ena(sv2v_tmp_IX10_clk_div_ena),
		.status_clear(status_clear)
	);
	wire clear_tx;
	wire transaction_begin;
	assign clear_tx = i2c_status[0] | control[11];
	assign transaction_begin = control[9];
	wire tx_almost_full;
	wire [8:1] sv2v_tmp_IX1_r_data;
	always @(*) tx_data = sv2v_tmp_IX1_r_data;
	wire [1:1] sv2v_tmp_IX1_full;
	always @(*) tx_full = sv2v_tmp_IX1_full;
	wire [1:1] sv2v_tmp_IX1_empty;
	always @(*) tx_empty = sv2v_tmp_IX1_empty;
	apbfifo IX1(
		.w_data(pdata[7:0]),
		.w_enable(tx_w_ena),
		.r_enable(tx_r_ena),
		.r_clk(scl),
		.w_clk(pclk),
		.n_rst(n_rst),
		.clear(clear_tx),
		.store_r_ptr(transaction_begin),
		.revert_r_ptr(i2c_status[3]),
		.r_data(sv2v_tmp_IX1_r_data),
		.full(sv2v_tmp_IX1_full),
		.empty(sv2v_tmp_IX1_empty),
		.almost_full(tx_almost_full)
	);
	wire [12:1] sv2v_tmp_IX2_data_out;
	always @(*) control = sv2v_tmp_IX2_data_out;
	controlReg #(.MAX_BIT(11)) IX2(
		.data_in({pdata[11:0]}),
		.clk(pclk),
		.shift_enable(cr_w_ena),
		.n_rst(n_rst),
		.clear_begin_trans(i2c_status[5]),
		.data_out(sv2v_tmp_IX2_data_out)
	);
	wire [10:1] sv2v_tmp_IX8_data_out;
	always @(*) address = sv2v_tmp_IX8_data_out;
	flexPTP #(.NUM_BITS(9)) IX8(
		.data_in(pdata[9:0]),
		.clk(pclk),
		.shift_enable(addr_ena),
		.n_rst(n_rst),
		.data_out(sv2v_tmp_IX8_data_out)
	);
	wire [8:1] sv2v_tmp_IX3_r_data;
	always @(*) i2c_data = sv2v_tmp_IX3_r_data;
	wire [1:1] sv2v_tmp_IX3_full;
	always @(*) rx_full = sv2v_tmp_IX3_full;
	wire [1:1] sv2v_tmp_IX3_empty;
	always @(*) rx_empty = sv2v_tmp_IX3_empty;
	wire [1:1] sv2v_tmp_IX3_almost_full;
	always @(*) rx_almost_full = sv2v_tmp_IX3_almost_full;
	apbfifo IX3(
		.w_data(rx_data),
		.w_enable(rx_w_ena),
		.r_enable(rx_r_ena[0]),
		.r_clk(pclk),
		.w_clk(scl),
		.n_rst(n_rst),
		.clear(1'b0),
		.store_r_ptr(1'b0),
		.revert_r_ptr(1'b0),
		.r_data(sv2v_tmp_IX3_r_data),
		.full(sv2v_tmp_IX3_full),
		.empty(sv2v_tmp_IX3_empty),
		.almost_full(sv2v_tmp_IX3_almost_full)
	);
	wire [13:1] sv2v_tmp_IX4_next_status;
	always @(*) next_status = sv2v_tmp_IX4_next_status;
	statusLogic IX4(
		.i2c_status(i2c_status[4:0]),
		.tx_full(tx_full),
		.tx_empty(tx_empty),
		.rx_full(rx_full),
		.rx_empty(rx_empty),
		.rx_w_ena(rx_w_ena),
		.tx_r_ena(tx_r_ena),
		.mid_tx_empty(mid_tx_empty),
		.mid_rx_full(mid_rx_full),
		.next_status(sv2v_tmp_IX4_next_status)
	);
	wire [13:1] sv2v_tmp_IX5_data_out;
	always @(*) status = sv2v_tmp_IX5_data_out;
	statusReg IX5(
		.data_in(next_status),
		.clk(pclk),
		.n_rst(n_rst),
		.clear(status_clear),
		.data_out(sv2v_tmp_IX5_data_out)
	);
	wire [32:1] sv2v_tmp_IX9_data_out;
	always @(*) clk_out = sv2v_tmp_IX9_data_out;
	flexPTP #(.NUM_BITS(31)) IX9(
		.data_in(pdata),
		.clk(pclk),
		.shift_enable(clk_div_ena),
		.n_rst(n_rst),
		.data_out(sv2v_tmp_IX9_data_out)
	);
	always @(*)
		if (rx_r_ena[1])
			prdata = {24'd0, i2c_data};
		else if (cr_r_ena)
			prdata = {21'd0, control};
		else
			prdata = {19'd0, status};
	wire [1:1] sv2v_tmp_IX7_interrupt;
	always @(*) i2c_interrupt = sv2v_tmp_IX7_interrupt;
	interruptLogic IX7(
		.clk(pclk),
		.n_rst(n_rst),
		.status(status),
		.interrupt(sv2v_tmp_IX7_interrupt)
	);
	always @(*)
		if (status[4] & (status[6] | (!status[10] & next_status[10])))
			mid_tx_empty = 1'b1;
		else
			mid_tx_empty = 1'b0;
	always @(*)
		if (status[4] & (status[5] | (!status[8] & next_status[8])))
			mid_rx_full = 1'b1;
		else
			mid_rx_full = 1'b0;
endmodule
