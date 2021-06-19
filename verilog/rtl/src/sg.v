module sg (
	CLK,
	nRST,
	enable,
	polarity,
	tb_sck
);
	input wire CLK;
	input wire nRST;
	input wire enable;
	input wire polarity;
	output reg tb_sck;
	reg [3:0] cnt;
	always @(posedge CLK or negedge nRST)
		if (!nRST)
			tb_sck <= polarity;
		else begin
			if (enable)
				cnt <= cnt + 1;
			else
				cnt <= {4 {1'sb0}};
			if (enable && (cnt == 4'hf))
				tb_sck <= ~tb_sck;
			else if (enable)
				tb_sck <= tb_sck;
			else
				tb_sck <= polarity;
		end
endmodule
