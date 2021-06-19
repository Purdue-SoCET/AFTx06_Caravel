module left_shift (
	fraction,
	result,
	shifted_amount
);
	input [25:0] fraction;
	output reg [25:0] result;
	output reg [7:0] shifted_amount;
	always @(*) begin
		result = fraction;
		shifted_amount = 0;
		casez (fraction)
			26'b01zzzzzzzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[24:0], 1'd0};
				shifted_amount = 1;
			end
			26'b001zzzzzzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[23:0], 2'd0};
				shifted_amount = 2;
			end
			26'b0001zzzzzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[22:0], 3'd0};
				shifted_amount = 3;
			end
			26'b00001zzzzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[21:0], 4'd0};
				shifted_amount = 4;
			end
			26'b000001zzzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[20:0], 5'd0};
				shifted_amount = 5;
			end
			26'b0000001zzzzzzzzzzzzzzzzzzz: begin
				result = {fraction[19:0], 6'd0};
				shifted_amount = 6;
			end
			26'b00000001zzzzzzzzzzzzzzzzzz: begin
				result = {fraction[18:0], 7'd0};
				shifted_amount = 7;
			end
			26'b000000001zzzzzzzzzzzzzzzzz: begin
				result = {fraction[17:0], 8'd0};
				shifted_amount = 8;
			end
			26'b0000000001zzzzzzzzzzzzzzzz: begin
				result = {fraction[16:0], 9'd0};
				shifted_amount = 9;
			end
			26'b00000000001zzzzzzzzzzzzzzz: begin
				result = {fraction[15:0], 10'd0};
				shifted_amount = 10;
			end
			26'b000000000001zzzzzzzzzzzzzz: begin
				result = {fraction[14:0], 11'd0};
				shifted_amount = 11;
			end
			26'b0000000000001zzzzzzzzzzzzz: begin
				result = {fraction[13:0], 12'd0};
				shifted_amount = 12;
			end
			26'b00000000000001zzzzzzzzzzzz: begin
				result = {fraction[12:0], 13'd0};
				shifted_amount = 13;
			end
			26'b000000000000001zzzzzzzzzzz: begin
				result = {fraction[11:0], 14'd0};
				shifted_amount = 14;
			end
			26'b0000000000000001zzzzzzzzzz: begin
				result = {fraction[10:0], 15'd0};
				shifted_amount = 15;
			end
			26'b00000000000000001zzzzzzzzz: begin
				result = {fraction[9:0], 16'd0};
				shifted_amount = 16;
			end
			26'b000000000000000001zzzzzzzz: begin
				result = {fraction[8:0], 17'd0};
				shifted_amount = 17;
			end
			26'b0000000000000000001zzzzzzz: begin
				result = {fraction[7:0], 18'd0};
				shifted_amount = 18;
			end
			26'b00000000000000000001zzzzzz: begin
				result = {fraction[6:0], 19'd0};
				shifted_amount = 19;
			end
			26'b000000000000000000001zzzzz: begin
				result = {fraction[5:0], 20'd0};
				shifted_amount = 20;
			end
			26'b0000000000000000000001zzzz: begin
				result = {fraction[4:0], 21'd0};
				shifted_amount = 21;
			end
			26'b00000000000000000000001zzz: begin
				result = {fraction[3:0], 22'd0};
				shifted_amount = 22;
			end
			26'b000000000000000000000001zz: begin
				result = {fraction[2:0], 23'd0};
				shifted_amount = 23;
			end
			26'b0000000000000000000000001z: begin
				result = {fraction[1:0], 24'd0};
				shifted_amount = 24;
			end
			26'b00000000000000000000000001: begin
				result = {fraction[0], 25'd0};
				shifted_amount = 25;
			end
		endcase
	end
endmodule
