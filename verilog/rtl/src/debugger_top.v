module debugger_top (
	clk,
	rst_n,
	rx,
	M0_RST,
	tx,
	HREADY,
	HRDATA,
	HWRITE,
	HSIZE,
	HBURST,
	HTRANS,
	HWDATA,
	HADDR
);
	input wire clk;
	input wire rst_n;
	input wire rx;
	output wire M0_RST;
	output wire tx;
	input wire HREADY;
	input wire [31:0] HRDATA;
	output wire HWRITE;
	output wire [2:0] HSIZE;
	output wire [2:0] HBURST;
	output wire [1:0] HTRANS;
	output wire [31:0] HWDATA;
	output wire [31:0] HADDR;
	wire byte_send;
	wire [7:0] UDATA_OUT;
	wire [7:0] UDATA_IN;
	wire byte_rcv;
	wire start_trans;
	wire is_receiving;
	wire is_transmitting;
	wire recv_error;
	debugger debugger(
		.clk(clk),
		.rst_n(rst_n),
		.data_in(UDATA_IN),
		.byte_rcv(byte_rcv),
		.byte_send(byte_send),
		.start_trans(start_trans),
		.data_send(UDATA_OUT),
		.M0_RST(M0_RST),
		.HREADY(HREADY),
		.HRDATA(HRDATA),
		.HWRITE(HWRITE),
		.HSIZE(HSIZE),
		.HBURST(HBURST),
		.HTRANS(HTRANS),
		.HADDR(HADDR),
		.HWDATA(HWDATA)
	);
	uart uart(
		.clk(clk),
		.n_rst(rst_n),
		.rx(rx),
		.transmit(start_trans),
		.tx_byte(UDATA_OUT),
		.tx(tx),
		.received(byte_rcv),
		.rx_byte(UDATA_IN),
		.is_receiving(is_receiving),
		.is_transmitting(is_transmitting),
		.recv_error(recv_error),
		.sent(byte_send)
	);
endmodule
