module radix4_divider (
	CLK,
	nRST,
	start,
	is_signed,
	dividend,
	divisor,
	quotient,
	remainder,
	finished
);
	parameter NUM_BITS = 32;
	input wire CLK;
	input wire nRST;
	input wire start;
	input wire is_signed;
	input wire [NUM_BITS - 1:0] dividend;
	input wire [NUM_BITS - 1:0] divisor;
	output wire [NUM_BITS - 1:0] quotient;
	output wire [NUM_BITS - 1:0] remainder;
	output reg finished;
	reg [NUM_BITS - 1:0] next_remainder;
	reg [NUM_BITS - 1:0] next_quotient;
	reg [NUM_BITS - 1:0] shifted_remainder;
	reg [NUM_BITS - 1:0] shifted_quotient;
	reg [NUM_BITS - 1:0] temp_quotient;
	reg [NUM_BITS - 1:0] temp_remainder;
	reg [NUM_BITS:0] Result1;
	reg [NUM_BITS:0] Result2;
	reg [NUM_BITS:0] Result3;
	wire [NUM_BITS - 1:0] DivisorX2;
	wire [NUM_BITS - 1:0] DivisorX3;
	reg [4:0] count;
	reg [4:0] next_count;
	wire [NUM_BITS - 1:0] usign_divisor;
	wire [NUM_BITS - 1:0] usign_dividend;
	wire adjustment_possible;
	wire adjust_quotient;
	wire adjust_remainder;
	wire div_done;
	assign usign_divisor = (is_signed & divisor[NUM_BITS - 1] ? ~divisor + 1 : divisor);
	assign usign_dividend = (is_signed & dividend[NUM_BITS - 1] ? ~dividend + 1 : dividend);
	assign adjustment_possible = is_signed && (divisor[NUM_BITS - 1] ^ dividend[NUM_BITS - 1]);
	assign adjust_quotient = adjustment_possible && ~quotient[NUM_BITS - 1];
	assign adjust_remainder = is_signed && dividend[NUM_BITS - 1];
	assign div_done = count == 0;
	assign quotient = temp_quotient;
	assign remainder = temp_remainder;
	always @(posedge CLK or negedge nRST)
		if (nRST == 0)
			finished <= 1'b0;
		else if (start)
			finished <= 1'b0;
		else if (div_done)
			finished <= 1'b1;
	assign DivisorX2 = usign_divisor << 1;
	assign DivisorX3 = (usign_divisor << 1) + usign_divisor;
	always @(posedge CLK or negedge nRST)
		if (nRST == 0) begin
			count <= 5'd16;
			temp_quotient <= {NUM_BITS {1'sb0}};
			temp_remainder <= {NUM_BITS {1'sb0}};
		end
		else if (start) begin
			temp_quotient <= usign_dividend;
			temp_remainder <= {NUM_BITS {1'sb0}};
			count <= 5'd16;
		end
		else if (count >= 0) begin
			temp_quotient <= next_quotient;
			temp_remainder <= next_remainder;
			count <= next_count;
		end
	always @(*) begin
		next_quotient = temp_quotient;
		next_remainder = temp_remainder;
		next_count = count;
		if (count != 0) begin
			next_count = count - 1;
			shifted_remainder = (temp_remainder << 2) | temp_quotient[NUM_BITS - 1:NUM_BITS - 2];
			shifted_quotient = temp_quotient << 2;
			Result1 = shifted_remainder - usign_divisor;
			Result2 = shifted_remainder - DivisorX2;
			Result3 = shifted_remainder - DivisorX3;
			if (Result1[NUM_BITS - 1] | Result1[NUM_BITS]) begin
				next_remainder = shifted_remainder;
				next_quotient = shifted_quotient | 0;
				if ((count == 1) && adjust_quotient)
					next_quotient = ~next_quotient + 1;
				if ((count == 1) && adjust_remainder)
					next_remainder = ~next_remainder + 1;
			end
			else if (Result2[NUM_BITS - 1] | Result2[NUM_BITS]) begin
				next_remainder = Result1;
				next_quotient = shifted_quotient | 1;
				if ((count == 1) && adjust_quotient)
					next_quotient = ~next_quotient + 1;
				if ((count == 1) && adjust_remainder)
					next_remainder = ~next_remainder + 1;
			end
			else if (Result3[NUM_BITS - 1] | Result3[NUM_BITS]) begin
				next_remainder = Result2;
				next_quotient = shifted_quotient | 2;
				if ((count == 1) && adjust_quotient)
					next_quotient = ~next_quotient + 1;
				if ((count == 1) && adjust_remainder)
					next_remainder = ~next_remainder + 1;
			end
			else begin
				next_remainder = Result3;
				next_quotient = shifted_quotient | 3;
				if ((count == 1) && adjust_quotient)
					next_quotient = ~next_quotient + 1;
				if ((count == 1) && adjust_remainder)
					next_remainder = ~next_remainder + 1;
			end
		end
	end
endmodule
