module pwm (
	paddr,
	pwdata,
	psel,
	penable,
	pwrite,
	prdata,
	clk,
	n_rst,
	pwm_out
);
	parameter NUM_CHANNELS = 2;
	input wire [31:0] paddr;
	input wire [31:0] pwdata;
	input wire psel;
	input wire penable;
	input wire pwrite;
	output wire [31:0] prdata;
	input wire clk;
	input wire n_rst;
	output wire [NUM_CHANNELS - 1:0] pwm_out;
	localparam NUM_REG_PER_CHAN = 3;
	localparam NUM_REGS = NUM_REG_PER_CHAN * NUM_CHANNELS;
	localparam PERIOD_IND = 0;
	localparam DUTY_IND = 1;
	localparam COUNT_IND = 2;
	wire [NUM_REGS - 1:0] wen;
	wire [NUM_REGS - 1:0] ren;
	wire [31:0] w_data;
	genvar i;
	APB_SlaveInterface_general #(.NUM_REGS(NUM_REGS)) apbs(
		.clk(clk),
		.n_rst(n_rst),
		.PADDR(paddr),
		.PWDATA(pwdata),
		.PENABLE(penable),
		.PWRITE(pwrite),
		.PRDATA(prdata),
		.PSEL(psel),
		.read_data(0),
		.w_enable(wen),
		.r_enable(ren),
		.w_data(w_data)
	);
	generate
		for (i = 0; i < NUM_CHANNELS; i = i + 1) pwmchannel channel(
			.clk(clk),
			.n_rst(n_rst),
			.pwm_out(pwm_out[i]),
			.cont_wen(wen[(i * NUM_REG_PER_CHAN) + COUNT_IND]),
			.period_wen(wen[(i * NUM_REG_PER_CHAN) + PERIOD_IND]),
			.duty_wen(wen[(i * NUM_REG_PER_CHAN) + DUTY_IND]),
			.duty_in(w_data),
			.period_in(w_data),
			.control_in(w_data[2:0])
		);
	endgenerate
endmodule
