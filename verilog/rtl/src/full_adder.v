module full_adder (
	x,
	y,
	cin,
	cout,
	sum
);
	input wire x;
	input wire y;
	input wire cin;
	output wire cout;
	output wire sum;
	assign sum = (x ^ y) ^ cin;
	assign cout = ((x & y) | (x & cin)) | (y & cin);
endmodule
