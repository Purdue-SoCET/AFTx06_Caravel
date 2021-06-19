module clock_generator (
	CLK,
	nRST,
	divider,
	enable,
	polarity,
	SCK
);
	parameter NUMBITS = 32;
	input wire CLK;
	input wire nRST;
	input wire [NUMBITS - 1:0] divider;
	input wire enable;
	input wire polarity;
	output reg SCK;
	reg [31:0] count;
	reg [31:0] count_nxt;
	wire SCK_next;
	always @(posedge CLK or negedge nRST)
		if (!nRST) begin
			count <= {32 {1'sb0}};
			SCK <= 1'b0;
		end
		else if (enable) begin
			count <= count_nxt;
			SCK <= SCK_next;
		end
		else begin
			count <= {32 {1'sb0}};
			SCK <= polarity;
		end
	always @(*)
		if (count == divider)
			count_nxt = 0;
		else
			count_nxt = count + 1;
	assign SCK_next = (count == divider ? ~SCK : SCK);
endmodule
