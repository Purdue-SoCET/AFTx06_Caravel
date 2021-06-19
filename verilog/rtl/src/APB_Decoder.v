module APB_Decoder (
	PADDR,
	psel_en,
	PRData_in,
	PRDATA_PSlave,
	PSEL_slave
);
	parameter NUM_SLAVES = 2;
	input wire [31:0] PADDR;
	input wire psel_en;
	input wire [(NUM_SLAVES * 32) - 1:0] PRData_in;
	output wire [31:0] PRDATA_PSlave;
	output wire [NUM_SLAVES - 1:0] PSEL_slave;
	reg [NUM_SLAVES - 1:0] psel_slave_reg;
	wire signed [31:0] i;
	always @(*) begin
		psel_slave_reg = {NUM_SLAVES {1'sb0}};
		if (psel_en)
			psel_slave_reg = 1 << PADDR[23:16];
	end
	assign PSEL_slave = psel_slave_reg;
	assign PRDATA_PSlave = PRData_in[PADDR[23:16] * 32+:32];
endmodule
