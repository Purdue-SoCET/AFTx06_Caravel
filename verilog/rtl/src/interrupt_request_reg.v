module interrupt_request_reg (
	clk,
	n_rst,
	interrupt_requests_in,
	interrupt_requests
);
	parameter N_interrupts = 32;
	input clk;
	input n_rst;
	input wire [N_interrupts - 1:0] interrupt_requests_in;
	output reg [N_interrupts - 1:0] interrupt_requests;
	reg [N_interrupts - 1:0] interrupt_requests_in_prev;
	reg [N_interrupts - 1:0] interrupt_requests_next;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 'b0) begin
			interrupt_requests_in_prev <= 'b0;
			interrupt_requests <= 'b0;
		end
		else begin
			interrupt_requests <= interrupt_requests_next;
			interrupt_requests_in_prev <= interrupt_requests_in;
		end
	always @(*) interrupt_requests_next = interrupt_requests_in & ~interrupt_requests_in_prev;
endmodule
