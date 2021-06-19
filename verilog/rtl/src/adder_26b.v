module adder_26b (
	frac1,
	frac2,
	sum,
	ovf
);
	input [26:0] frac1;
	input [26:0] frac2;
	output reg [26:0] sum;
	output reg ovf;
	always @(*) begin
		sum = frac1 + frac2;
		ovf = 0;
		if (((frac1[26] == 1) && (frac2[26] == 1)) && (sum[26] == 0)) begin
			ovf = 1;
			sum[26] = 1;
		end
		if (((frac1[26] == 0) && (frac2[26] == 0)) && (sum[26] == 1)) begin
			ovf = 1;
			sum[26] = 0;
		end
	end
endmodule
