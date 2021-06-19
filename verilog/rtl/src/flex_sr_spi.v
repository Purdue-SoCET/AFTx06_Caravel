module flex_sr_spi (
	clk,
	n_rst,
	shift_enable,
	shift_clk,
	shift_clear,
	shift_msb,
	load_enable,
	mode,
	serial_in,
	parallel_in,
	parallel_out,
	serial_out
);
	parameter NUM_BITS = 4;
	input wire clk;
	input wire n_rst;
	input wire shift_enable;
	input wire shift_clk;
	input wire shift_clear;
	input wire shift_msb;
	input wire load_enable;
	input wire mode;
	input wire serial_in;
	input wire [NUM_BITS - 1:0] parallel_in;
	output wire [NUM_BITS - 1:0] parallel_out;
	output wire serial_out;
	reg [NUM_BITS - 1:0] nstate;
	reg [NUM_BITS - 1:0] cstate;
	assign parallel_out = cstate;
	assign serial_out = (shift_msb == 1 ? cstate[NUM_BITS - 1] : cstate[0]);
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			cstate <= {NUM_BITS {1'sb0}};
		else
			cstate <= nstate;
	always @(*) begin
		nstate = cstate;
		if (load_enable)
			nstate = parallel_in;
		else if (shift_clear)
			nstate = {NUM_BITS {1'sb0}};
		else if (shift_enable & shift_clk) begin
			if (shift_msb) begin
				if (mode)
					nstate = {cstate[NUM_BITS - 2:0], 1'b0};
				else
					nstate = {cstate[NUM_BITS - 2:0], serial_in};
			end
			else if (mode)
				nstate = {1'b0, cstate[NUM_BITS - 1:1]};
			else
				nstate = {serial_in, cstate[NUM_BITS - 1:1]};
		end
		else
			nstate = cstate;
	end
endmodule
