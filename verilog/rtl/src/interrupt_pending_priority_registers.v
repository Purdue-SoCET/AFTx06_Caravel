module interrupt_pending_priority_registers (
	clk,
	n_rst,
	interrupt_requests_masked,
	pending_interrupts,
	interrupt_priority_regs,
	active_interrupt,
	interrupt_claimed,
	priority_addr,
	pending_addr,
	enable_addr,
	addr,
	wen,
	rdata,
	wdata,
	addr_valid
);
	parameter N_interrupts = 32;
	input wire clk;
	input wire n_rst;
	input wire [N_interrupts - 1:0] interrupt_requests_masked;
	output wire [N_interrupts - 1:0] pending_interrupts;
	output reg [(N_interrupts * 32) - 1:0] interrupt_priority_regs;
	input wire [N_interrupts - 1:0] active_interrupt;
	input wire interrupt_claimed;
	input wire [31:0] priority_addr;
	input wire [31:0] pending_addr;
	input wire [31:0] enable_addr;
	input wire [31:0] addr;
	input wire wen;
	output reg [31:0] rdata;
	input wire [31:0] wdata;
	output wire addr_valid;
	reg [(((N_interrupts - 1) >> 5) >= 0 ? ((((N_interrupts - 1) >> 5) + 1) * 32) - 1 : ((1 - ((N_interrupts - 1) >> 5)) * 32) + ((((N_interrupts - 1) >> 5) * 32) - 1)):(((N_interrupts - 1) >> 5) >= 0 ? 0 : ((N_interrupts - 1) >> 5) * 32)] interrupt_pending_regs;
	reg [(((N_interrupts - 1) >> 5) >= 0 ? ((((N_interrupts - 1) >> 5) + 1) * 32) - 1 : ((1 - ((N_interrupts - 1) >> 5)) * 32) + ((((N_interrupts - 1) >> 5) * 32) - 1)):(((N_interrupts - 1) >> 5) >= 0 ? 0 : ((N_interrupts - 1) >> 5) * 32)] interrupt_pending_regs_n;
	reg [(N_interrupts * 32) - 1:0] interrupt_priority_regs_n;
	wire [31:0] addr_shifted_pending;
	wire [31:0] addr_shifted_priority;
	wire addr_valid_interrupt_pending;
	wire addr_valid_priority;
	assign addr_shifted_pending = addr - pending_addr;
	assign addr_shifted_priority = addr - priority_addr;
	assign addr_valid_interrupt_pending = (addr >= pending_addr) && (addr < enable_addr);
	assign addr_valid_priority = (addr >= priority_addr) && (addr < pending_addr);
	assign addr_valid = addr_valid_interrupt_pending || addr_valid_priority;
	genvar i;
	generate
		for (i = 0; i < (((N_interrupts - 1) >> 5) >= 0 ? ((N_interrupts - 1) >> 5) + 1 : 1 - ((N_interrupts - 1) >> 5)); i = i + 1) if (i == ((((N_interrupts - 1) >> 5) >= 0 ? ((N_interrupts - 1) >> 5) + 1 : 1 - ((N_interrupts - 1) >> 5)) - 1)) begin
			assign pending_interrupts[N_interrupts - 1:32 * i] = interrupt_pending_regs[(((N_interrupts - 1) >> 5) >= 0 ? i : ((N_interrupts - 1) >> 5) - i) * 32+:32];
		end
		else assign pending_interrupts[(32 * i) + 31:32 * i] = interrupt_pending_regs[(((N_interrupts - 1) >> 5) >= 0 ? i : ((N_interrupts - 1) >> 5) - i) * 32+:32];
	endgenerate
	always @(*) begin
		interrupt_pending_regs_n = interrupt_pending_regs | interrupt_requests_masked;
		if (interrupt_claimed == 1'b1)
			interrupt_pending_regs_n = (interrupt_pending_regs | interrupt_requests_masked) & ~active_interrupt;
	end
	always @(posedge clk or negedge n_rst)
		if (n_rst == 'b0) begin
			interrupt_pending_regs <= 'b0;
			interrupt_priority_regs <= 'b0;
		end
		else begin
			interrupt_pending_regs <= interrupt_pending_regs_n;
			interrupt_priority_regs <= interrupt_priority_regs_n;
		end
	always @(*) begin
		interrupt_priority_regs_n = interrupt_priority_regs;
		if (addr_valid_priority && wen)
			interrupt_priority_regs_n[(addr_shifted_priority >> 2) * 32+:32] = {29'b00000000000000000000000000000, wdata[2:0]};
	end
	always @(*)
		if (addr_valid_priority)
			rdata = interrupt_priority_regs[(addr_shifted_priority >> 2) * 32+:32];
		else if (addr_valid_interrupt_pending)
			rdata = interrupt_pending_regs[(((N_interrupts - 1) >> 5) >= 0 ? addr_shifted_pending >> 2 : ((N_interrupts - 1) >> 5) - (addr_shifted_pending >> 2)) * 32+:32];
		else
			rdata = 32'b00000000000000000000000000000000;
endmodule
