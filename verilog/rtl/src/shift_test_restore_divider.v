module shift_test_restore_divider (
	CLK,
	nRST,
	divisor,
	dividend,
	is_signed,
	start,
	remainder,
	quotient,
	finished
);
	parameter N = 32;
	input wire CLK;
	input wire nRST;
	input wire [N - 1:0] divisor;
	input wire [N - 1:0] dividend;
	input wire is_signed;
	input wire start;
	output wire [N - 1:0] remainder;
	output wire [N - 1:0] quotient;
	output reg finished;
	localparam COUNTER_BITS = $clog2(N) + 1;
	localparam U_Q = N - 1;
	localparam U_R = (2 * N) - 1;
	reg [(2 * N) + 1:0] result;
	assign {remainder, quotient} = result[(2 * N) - 1:0];
	reg test_phase;
	reg [COUNTER_BITS - 1:0] counter;
	wire [N - 1:0] usign_divisor;
	wire [N - 1:0] usign_dividend;
	wire adjustment_possible;
	wire adjust_quotient;
	wire adjust_remainder;
	wire div_done;
	assign usign_divisor = (is_signed & divisor[N - 1] ? ~divisor + 1 : divisor);
	assign usign_dividend = (is_signed & dividend[N - 1] ? ~dividend + 1 : dividend);
	assign adjustment_possible = is_signed && (divisor[N - 1] ^ dividend[N - 1]);
	assign adjust_quotient = adjustment_possible && ~quotient[N - 1];
	assign adjust_remainder = is_signed && dividend[N - 1];
	assign div_done = counter == 0;
	always @(posedge CLK or negedge nRST)
		if (~nRST) begin
			result <= {(((2 * N) + 1) >= 0 ? (2 * N) + 2 : 1 - ((2 * N) + 1)) {1'sb0}};
			counter <= N;
			test_phase <= 1'b0;
		end
		else if (start) begin
			result <= {{N - 1 {1'b0}}, usign_dividend, 1'b0};
			counter <= N;
			test_phase <= 1'b0;
		end
		else if (counter > 0) begin
			if (~test_phase)
				result[U_R + 1-:N + 1] <= result[U_R + 1-:N + 1] - usign_divisor;
			else begin
				counter <= counter - 1;
				if (result[U_R + 1])
					result <= {result[U_R + 1-:N + 1] + usign_divisor, result[U_Q:0]} << 1;
				else
					result <= {result[U_R - 1:0], 1'b1};
			end
			test_phase <= ~test_phase;
		end
		else if (~finished) begin
			if (adjust_quotient)
				result[U_Q:0] <= ~result[U_Q:0] + 1;
			if (adjust_remainder)
				result[U_R-:N] <= ~result[U_R + 1-:N] + 1;
			else
				result[U_R-:N] <= result[U_R + 1-:N];
		end
	always @(posedge CLK or negedge nRST)
		if (~nRST)
			finished <= 1'b0;
		else if (start)
			finished <= 1'b0;
		else if (div_done)
			finished <= 1'b1;
endmodule
