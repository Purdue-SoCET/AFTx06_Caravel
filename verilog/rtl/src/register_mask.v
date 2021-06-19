module register_mask (
	interrupt_requests,
	interrupt_masks,
	interrupt_requests_masked
);
	parameter N_interrupts = 32;
	input wire [N_interrupts - 1:0] interrupt_requests;
	input wire [N_interrupts - 1:0] interrupt_masks;
	output wire [N_interrupts - 1:0] interrupt_requests_masked;
	assign interrupt_requests_masked = interrupt_requests & ~interrupt_masks;
endmodule
