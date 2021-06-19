module interrupt_enable_registers (
	n_rst,
	clk,
	enable_addr,
	reserved_addr,
	priority_threshold_addr,
	claim_complete_addr,
	interrupt_priority_regs,
	interrupt_masks,
	addr,
	wen,
	rdata,
	wdata,
	addr_valid
);
	parameter N_interrupts = 32;
	input wire n_rst;
	input wire clk;
	input wire [31:0] enable_addr;
	input wire [31:0] reserved_addr;
	input wire [31:0] priority_threshold_addr;
	input wire [31:0] claim_complete_addr;
	input wire [(N_interrupts * 32) - 1:0] interrupt_priority_regs;
	output reg [N_interrupts - 1:0] interrupt_masks;
	input wire [31:0] addr;
	input wire wen;
	output reg [31:0] rdata;
	input wire [31:0] wdata;
	output wire addr_valid;
	reg [((N_interrupts >> 5) >= 0 ? (((N_interrupts >> 5) + 1) * 32) - 1 : ((1 - (N_interrupts >> 5)) * 32) + (((N_interrupts >> 5) * 32) - 1)):((N_interrupts >> 5) >= 0 ? 0 : (N_interrupts >> 5) * 32)] interrupt_enable_regs;
	reg [((N_interrupts >> 5) >= 0 ? (((N_interrupts >> 5) + 1) * 32) - 1 : ((1 - (N_interrupts >> 5)) * 32) + (((N_interrupts >> 5) * 32) - 1)):((N_interrupts >> 5) >= 0 ? 0 : (N_interrupts >> 5) * 32)] interrupt_enable_regs_n;
	reg [31:0] interrupt_priority_thresh_regs;
	reg [31:0] interrupt_priority_thresh_regs_n;
	wire [31:0] addr_shifted_enable;
	reg [N_interrupts - 1:0] interrupt_priority_below_mask_val;
	wire addr_valid_enable;
	wire addr_valid_priority_thresh;
	assign addr_shifted_enable = addr - enable_addr;
	assign addr_valid_enable = (addr >= enable_addr) && (addr < reserved_addr);
	assign addr_valid_priority_thresh = (addr >= priority_threshold_addr) && (addr < claim_complete_addr);
	assign addr_valid = addr_valid_enable || addr_valid_priority_thresh;
	always @(*) begin : sv2v_autoblock_21
		integer n;
		for (n = 0; n < N_interrupts; n = n + 1)
			if ((interrupt_priority_regs[n * 32+:32] <= interrupt_priority_thresh_regs) | ~interrupt_enable_regs[(((N_interrupts >> 5) >= 0 ? (n + 1) >> 5 : (N_interrupts >> 5) - ((n + 1) >> 5)) * 32) + ((n + 1) & 32'd31)])
				interrupt_masks[n] = 1'b1;
			else
				interrupt_masks[n] = 1'b0;
	end
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0) begin
			interrupt_enable_regs <= {(((N_interrupts >> 5) >= 0 ? (((N_interrupts >> 5) + 1) * 32) - 1 : ((1 - (N_interrupts >> 5)) * 32) + (((N_interrupts >> 5) * 32) - 1)) >= ((N_interrupts >> 5) >= 0 ? 0 : (N_interrupts >> 5) * 32) ? (((N_interrupts >> 5) >= 0 ? (((N_interrupts >> 5) + 1) * 32) - 1 : ((1 - (N_interrupts >> 5)) * 32) + (((N_interrupts >> 5) * 32) - 1)) - ((N_interrupts >> 5) >= 0 ? 0 : (N_interrupts >> 5) * 32)) + 1 : (((N_interrupts >> 5) >= 0 ? 0 : (N_interrupts >> 5) * 32) - ((N_interrupts >> 5) >= 0 ? (((N_interrupts >> 5) + 1) * 32) - 1 : ((1 - (N_interrupts >> 5)) * 32) + (((N_interrupts >> 5) * 32) - 1))) + 1) {1'sb0}};
			interrupt_priority_thresh_regs <= {32 {1'sb0}};
		end
		else begin
			interrupt_enable_regs <= interrupt_enable_regs_n;
			interrupt_priority_thresh_regs <= interrupt_priority_thresh_regs_n;
		end
	always @(*)
		if (addr_valid_enable)
			rdata = interrupt_enable_regs[((N_interrupts >> 5) >= 0 ? addr_shifted_enable >> 2 : (N_interrupts >> 5) - (addr_shifted_enable >> 2)) * 32+:32];
		else if (addr_valid_priority_thresh)
			rdata = interrupt_priority_thresh_regs;
		else
			rdata = 'b0;
	always @(*) begin
		interrupt_enable_regs_n = interrupt_enable_regs;
		if (addr_valid_enable & wen)
			interrupt_enable_regs_n[((N_interrupts >> 5) >= 0 ? addr_shifted_enable >> 2 : (N_interrupts >> 5) - (addr_shifted_enable >> 2)) * 32+:32] = wdata;
	end
	always @(*)
		if (addr_valid_priority_thresh & wen)
			interrupt_priority_thresh_regs_n = wdata;
		else
			interrupt_priority_thresh_regs_n = interrupt_priority_thresh_regs;
endmodule
