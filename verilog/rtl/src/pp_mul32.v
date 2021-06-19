module pp_mul32 (
	CLK,
	nRST,
	multiplicand,
	multiplier,
	is_signed,
	start,
	finished,
	product
);
	input wire CLK;
	input wire nRST;
	input wire [31:0] multiplicand;
	input wire [31:0] multiplier;
	input wire [1:0] is_signed;
	input wire start;
	output wire finished;
	output reg [63:0] product;
	reg [31:0] multiplicand_reg;
	reg [31:0] multiplier_reg;
	wire [63:0] result;
	wire [63:0] result2;
	wire [63:0] temp_product;
	wire [63:0] temp_product2;
	wire [31:0] multiplicand_mod;
	wire [31:0] multiplier_mod;
	wire adjust_product;
	reg [63:0] partial_product [15:0];
	reg [63:0] pp0;
	reg [63:0] pp1;
	reg [63:0] pp2;
	reg [63:0] pp3;
	reg [63:0] pp4;
	reg [63:0] pp5;
	reg [63:0] pp6;
	reg [63:0] pp7;
	reg [63:0] pp8;
	reg [63:0] pp9;
	reg [63:0] pp10;
	reg [63:0] pp11;
	reg [63:0] pp12;
	reg [63:0] pp13;
	reg [63:0] pp14;
	reg [63:0] pp15;
	wire [32:0] mul_plus2;
	wire [32:0] mul_minus2;
	wire [32:0] mul_minus1;
	reg [63:0] pp [15:0];
	wire [32:0] modified_in;
	wire [63:0] sum0;
	wire [63:0] sum1;
	wire [63:0] sum2;
	wire [63:0] sum3;
	wire [63:0] sum4;
	wire [63:0] sum5;
	wire [63:0] sum6;
	wire [63:0] sum7;
	wire [63:0] sum8;
	wire [63:0] sum9;
	wire [63:0] sum10;
	wire [63:0] sum11;
	wire [63:0] sum12;
	wire [63:0] sum13;
	wire [63:0] cout0;
	wire [63:0] cout1;
	wire [63:0] cout2;
	wire [63:0] cout3;
	wire [63:0] cout4;
	wire [63:0] cout5;
	wire [63:0] cout6;
	wire [63:0] cout7;
	wire [63:0] cout8;
	wire [63:0] cout9;
	wire [63:0] cout10;
	wire [63:0] cout11;
	wire [63:0] cout12;
	wire [63:0] cout13;
	wire [1:0] count;
	wire mult_complete;
	reg [63:0] sum5_pip;
	reg [63:0] cout5_pip;
	reg [63:0] sum6_pip;
	reg [63:0] cout6_pip;
	reg [63:0] sum7_pip;
	reg [63:0] cout7_pip;
	reg [1:0] is_signed_reg;
	wire done;
	reg count_ena;
	integer i;
	integer j;
	always @(posedge CLK or negedge nRST)
		if (nRST == 0) begin
			multiplicand_reg <= {32 {1'sb0}};
			multiplier_reg <= {32 {1'sb0}};
			is_signed_reg <= {2 {1'sb0}};
		end
		else if (start) begin
			multiplicand_reg <= multiplicand;
			multiplier_reg <= multiplier;
			is_signed_reg <= is_signed;
		end
	assign multiplicand_mod = (is_signed_reg[1] && multiplicand_reg[31] ? ~multiplicand_reg + 1 : multiplicand_reg);
	assign multiplier_mod = (is_signed_reg[0] && multiplier_reg[31] ? ~multiplier_reg + 1 : multiplier_reg);
	assign adjust_product = (is_signed_reg[0] & multiplier_reg[31]) ^ (is_signed_reg[1] & multiplicand_reg[31]);
	assign mul_plus2 = multiplicand_mod + multiplicand_mod;
	assign mul_minus2 = ~mul_plus2 + 1;
	assign mul_minus1 = ~multiplicand_mod + 1;
	assign modified_in = {multiplier_mod, 1'b0};
	always @(*)
		for (i = 0; i < 32; i = i + 2)
			case ({modified_in[i + 2], modified_in[i + 1], modified_in[i]})
				3'b000: pp[i / 2] = {64 {1'sb0}};
				3'b001: pp[i / 2] = {32'd0, multiplicand_mod};
				3'b010: pp[i / 2] = {32'd0, multiplicand_mod};
				3'b011: pp[i / 2] = {31'd0, mul_plus2};
				3'b100:
					if (mul_minus2 == 0)
						pp[i / 2] = {64 {1'sb0}};
					else
						pp[i / 2] = {{31 {1'b1}}, mul_minus2};
				3'b101:
					if (mul_minus1 == 0)
						pp[i / 2] = {64 {1'sb0}};
					else
						pp[i / 2] = {{31 {1'b1}}, mul_minus1};
				3'b110:
					if (mul_minus1 == 0)
						pp[i / 2] = {64 {1'sb0}};
					else
						pp[i / 2] = {{31 {1'b1}}, mul_minus1};
				3'b111: pp[i / 2] = {64 {1'sb0}};
			endcase
	always @(*)
		for (j = 0; j < 16; j = j + 1)
			partial_product[j] = pp[j] << (2 * j);
	always @(posedge CLK or negedge nRST)
		if (nRST == 0) begin
			pp0 <= {64 {1'sb0}};
			pp1 <= {64 {1'sb0}};
			pp2 <= {64 {1'sb0}};
			pp3 <= {64 {1'sb0}};
			pp4 <= {64 {1'sb0}};
			pp5 <= {64 {1'sb0}};
			pp6 <= {64 {1'sb0}};
			pp7 <= {64 {1'sb0}};
			pp8 <= {64 {1'sb0}};
			pp9 <= {64 {1'sb0}};
			pp10 <= {64 {1'sb0}};
			pp11 <= {64 {1'sb0}};
			pp12 <= {64 {1'sb0}};
			pp13 <= {64 {1'sb0}};
			pp14 <= {64 {1'sb0}};
			pp15 <= {64 {1'sb0}};
		end
		else begin
			pp0 <= partial_product[0];
			pp1 <= partial_product[1];
			pp2 <= partial_product[2];
			pp3 <= partial_product[3];
			pp4 <= partial_product[4];
			pp5 <= partial_product[5];
			pp6 <= partial_product[6];
			pp7 <= partial_product[7];
			pp8 <= partial_product[8];
			pp9 <= partial_product[9];
			pp10 <= partial_product[10];
			pp11 <= partial_product[11];
			pp12 <= partial_product[12];
			pp13 <= partial_product[13];
			pp14 <= partial_product[14];
			pp15 <= partial_product[15];
		end
	carry_save_adder #(.BIT_WIDTH(64)) CSA0(
		.x(pp0),
		.y(pp1),
		.z(pp2),
		.cout(cout0),
		.sum(sum0)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA1(
		.x(pp3),
		.y(pp4),
		.z(pp5),
		.cout(cout1),
		.sum(sum1)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA2(
		.x(pp6),
		.y(pp7),
		.z(pp8),
		.cout(cout2),
		.sum(sum2)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA3(
		.x(pp9),
		.y(pp10),
		.z(pp11),
		.cout(cout3),
		.sum(sum3)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA4(
		.x(pp12),
		.y(pp13),
		.z(pp14),
		.cout(cout4),
		.sum(sum4)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA5(
		.x(cout0),
		.y(sum0),
		.z(cout1),
		.cout(cout5),
		.sum(sum5)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA6(
		.x(sum1),
		.y(cout2),
		.z(sum2),
		.cout(cout6),
		.sum(sum6)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA7(
		.x(cout3),
		.y(sum3),
		.z(cout4),
		.cout(cout7),
		.sum(sum7)
	);
	always @(posedge CLK or negedge nRST)
		if (nRST == 0) begin
			cout5_pip <= {64 {1'sb0}};
			sum5_pip <= {64 {1'sb0}};
			cout6_pip <= {64 {1'sb0}};
			sum6_pip <= {64 {1'sb0}};
			cout7_pip <= {64 {1'sb0}};
			sum7_pip <= {64 {1'sb0}};
		end
		else begin
			cout5_pip <= cout5;
			sum5_pip <= sum5;
			cout6_pip <= cout6;
			sum6_pip <= sum6;
			cout7_pip <= cout7;
			sum7_pip <= sum7;
		end
	carry_save_adder #(.BIT_WIDTH(64)) CSA8(
		.x(cout5),
		.y(sum5),
		.z(cout6),
		.cout(cout8),
		.sum(sum8)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA9(
		.x(sum6),
		.y(cout7),
		.z(sum7),
		.cout(cout9),
		.sum(sum9)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA10(
		.x(cout8),
		.y(sum8),
		.z(cout9),
		.cout(cout10),
		.sum(sum10)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA11(
		.x(sum9),
		.y(pp15),
		.z(sum4),
		.cout(cout11),
		.sum(sum11)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA12(
		.x(cout10),
		.y(sum10),
		.z(cout11),
		.cout(cout12),
		.sum(sum12)
	);
	carry_save_adder #(.BIT_WIDTH(64)) CSA13(
		.x(cout12),
		.y(sum12),
		.z(sum11),
		.cout(cout13),
		.sum(sum13)
	);
	flex_counter_mul #(.NUM_CNT_BITS(2)) FC(
		.clk(CLK),
		.n_rst(nRST),
		.clear(start),
		.count_enable(count_ena),
		.rollover_val(2'd2),
		.count_out(count),
		.rollover_flag(finished)
	);
	assign temp_product = cout13 + sum13;
	assign temp_product2 = ((is_signed_reg[0] == 0) && multiplier_reg[31] ? temp_product + ({{33 {multiplicand_mod[31]}}, multiplicand_mod} << 32) : temp_product);
	assign result = (adjust_product ? ~temp_product2 + 1 : temp_product2);
	assign mult_complete = (count == 2'd1) | (count == 2'd2);
	assign result2 = (mult_complete ? result : {64 {1'sb0}});
	always @(posedge CLK or negedge nRST)
		if (nRST == 0)
			product <= {64 {1'sb0}};
		else
			product <= result2;
	reg state;
	reg next_state;
	localparam [0:0] IDLE = 0;
	always @(posedge CLK or negedge nRST)
		if (nRST == 0)
			state <= IDLE;
		else
			state <= next_state;
	localparam [0:0] START = 1;
	always @(*) begin
		next_state = state;
		if ((state == IDLE) && start)
			next_state = START;
		else if ((state == START) && finished)
			next_state = IDLE;
		else
			next_state = state;
	end
	always @(*) begin
		count_ena = 0;
		case (state)
			IDLE: count_ena = 0;
			START: count_ena = ~finished;
		endcase
	end
endmodule
