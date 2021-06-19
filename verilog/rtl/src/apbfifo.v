module apbfifo (
	w_data,
	w_enable,
	r_enable,
	r_clk,
	w_clk,
	n_rst,
	clear,
	store_r_ptr,
	revert_r_ptr,
	r_data,
	full,
	empty,
	almost_full
);
	input reg [7:0] w_data;
	input wire w_enable;
	input wire r_enable;
	input wire r_clk;
	input wire w_clk;
	input wire n_rst;
	input wire clear;
	input wire store_r_ptr;
	input wire revert_r_ptr;
	output reg [7:0] r_data;
	output reg full;
	output reg empty;
	output reg almost_full;
	reg w_ena;
	reg r_ena;
	reg [3:0] w_ptr;
	reg [3:0] r_ptr;
	reg [7:0] ss;
	reg [3:0] next_ptr;
	reg write;
	reg [2:0] temp_ptr;
	wire [7:0] data0;
	wire [7:0] data1;
	wire [7:0] data2;
	wire [7:0] data3;
	wire [7:0] data4;
	wire [7:0] data5;
	wire [7:0] data6;
	wire [7:0] data7;
	always @(*)
		if (w_enable) begin
			if (full)
				w_ena = 1'b0;
			else
				w_ena = 1'b1;
		end
		else
			w_ena = 1'b0;
	always @(*) r_ena = r_enable & ~empty;
	wire [4:1] sv2v_tmp_IX_count_out;
	always @(*) w_ptr = sv2v_tmp_IX_count_out;
	flex_counter_reg IX(
		.clk(w_clk),
		.n_rst(n_rst),
		.clear((w_ena & (w_ptr == 4'b1111)) | clear),
		.save_count(1'b0),
		.revert_count(1'b0),
		.count_enable(w_ena),
		.count_out(sv2v_tmp_IX_count_out),
		.rollover_val(4'd15)
	);
	wire [4:1] sv2v_tmp_IX1_count_out;
	always @(*) r_ptr = sv2v_tmp_IX1_count_out;
	flex_counter_reg IX1(
		.clk(r_clk),
		.n_rst(n_rst),
		.clear((r_ena & (r_ptr == 4'b1111)) | clear),
		.save_count(store_r_ptr),
		.revert_count(revert_r_ptr),
		.count_enable(r_ena),
		.count_out(sv2v_tmp_IX1_count_out),
		.rollover_val(4'd15)
	);
	always @(*)
		if (w_ptr == r_ptr)
			empty = 1'b1;
		else
			empty = 1'b0;
	always @(*)
		if ((w_ptr[2:0] == r_ptr[2:0]) & (w_ptr[3] != r_ptr[3]))
			full = 1'b1;
		else
			full = 1'b0;
	always @(*) begin
		temp_ptr = r_ptr[2:0] - 1'b1;
		if (temp_ptr == w_ptr[2:0])
			almost_full = 1'b1;
		else
			almost_full = 1'b0;
	end
	always @(posedge w_clk or negedge n_rst)
		if (0 == n_rst)
			next_ptr <= {4 {1'sb0}};
		else
			next_ptr <= w_ptr;
	always @(*)
		if (w_ptr == next_ptr)
			write = 1'b0;
		else
			write = 1'b1;
	always @(*)
		if (write == 1)
			case (w_ptr[2:0])
				3'd0: ss = 8'b10000000;
				3'd1: ss = 8'b00000001;
				3'd2: ss = 8'b00000010;
				3'd3: ss = 8'b00000100;
				3'd4: ss = 8'b00001000;
				3'd5: ss = 8'b00010000;
				3'd6: ss = 8'b00100000;
				3'd7: ss = 8'b01000000;
				default: ss = {8 {1'sb0}};
			endcase
		else
			ss = {8 {1'sb0}};
	p2p IX3(
		.data_in(w_data),
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[0]),
		.data_out(data0)
	);
	p2p IX4(
		.data_in(w_data),
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[1]),
		.data_out(data1)
	);
	p2p IX5(
		.data_in(w_data),
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[2]),
		.data_out(data2)
	);
	p2p IX6(
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[3]),
		.data_in(w_data),
		.data_out(data3)
	);
	p2p IX7(
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[4]),
		.data_in(w_data),
		.data_out(data4)
	);
	p2p IX8(
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[5]),
		.data_in(w_data),
		.data_out(data5)
	);
	p2p IX9(
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[6]),
		.data_in(w_data),
		.data_out(data6)
	);
	p2p IX10(
		.clk(w_clk),
		.n_rst(n_rst),
		.shift_enable(ss[7]),
		.data_in(w_data),
		.data_out(data7)
	);
	always @(*)
		case (r_ptr[2:0])
			3'd0: r_data = data0;
			3'd1: r_data = data1;
			3'd2: r_data = data2;
			3'd3: r_data = data3;
			3'd4: r_data = data4;
			3'd5: r_data = data5;
			3'd6: r_data = data6;
			3'd7: r_data = data7;
			default: r_data = 8'd0;
		endcase
endmodule
