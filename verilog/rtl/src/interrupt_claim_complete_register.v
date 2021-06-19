module interrupt_claim_complete_register (
	clk,
	n_rst,
	interrupt_processing,
	claim_complete_addr,
	active_interrupt_ID,
	active_interrupt,
	interrupt_claimed,
	interrupt_request_pulse,
	addr,
	wen,
	ren,
	rdata,
	wdata,
	addr_valid
);
	parameter N_interrupts = 32;
	input clk;
	input n_rst;
	input interrupt_processing;
	input reg [31:0] claim_complete_addr;
	input reg [31:0] active_interrupt_ID;
	input reg [N_interrupts - 1:0] active_interrupt;
	output reg interrupt_claimed;
	output wire interrupt_request_pulse;
	input wire [31:0] addr;
	input wire wen;
	input wire ren;
	output reg [31:0] rdata;
	input wire [31:0] wdata;
	output wire addr_valid;
	reg [31:0] claim_complete_reg;
	reg [31:0] claim_complete_reg_n;
	wire interrupt_claim;
	reg interrupt_claim_prev;
	wire interrupt_request;
	reg interrupt_request_prev;
	assign addr_valid = (addr >= claim_complete_addr) && (addr < (claim_complete_addr + 4));
	assign interrupt_request = (wen & addr_valid) & (active_interrupt_ID != {32 {1'sb0}});
	always @(posedge clk or negedge n_rst)
		if (n_rst == 1'b0) begin
			claim_complete_reg <= {32 {1'sb0}};
			interrupt_claim_prev <= 1'b0;
			interrupt_request_prev <= 1'b0;
		end
		else begin
			claim_complete_reg <= claim_complete_reg_n;
			interrupt_claim_prev <= interrupt_claim;
			interrupt_request_prev <= interrupt_request;
		end
	always @(*)
		if (addr_valid && wen)
			claim_complete_reg_n = wdata;
		else if (interrupt_request_pulse)
			claim_complete_reg_n = active_interrupt_ID;
		else
			claim_complete_reg_n = claim_complete_reg;
	assign interrupt_request_pulse = interrupt_request_prev | interrupt_processing;
	assign interrupt_claim = addr_valid && ren;
	wire [1:1] sv2v_tmp_8AB22;
	assign sv2v_tmp_8AB22 = interrupt_claim && ~interrupt_claim_prev;
	always @(*) interrupt_claimed = sv2v_tmp_8AB22;
	always @(*)
		if (addr_valid)
			rdata = claim_complete_reg;
		else
			rdata = 32'b00000000000000000000000000000000;
endmodule
