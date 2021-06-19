module address_decoder (
	pclk,
	n_rst,
	paddr,
	penable,
	psel,
	pwrite,
	tx_w_ena,
	cr_w_ena,
	cr_r_ena,
	addr_ena,
	rx_r_ena,
	clk_div_ena,
	status_clear
);
	input wire pclk;
	input wire n_rst;
	input wire [31:0] paddr;
	input wire penable;
	input wire psel;
	input wire pwrite;
	output reg tx_w_ena;
	output reg cr_w_ena;
	output reg cr_r_ena;
	output reg addr_ena;
	output reg [1:0] rx_r_ena;
	output reg clk_div_ena;
	output reg status_clear;
	reg [3:0] curr_state;
	reg [3:0] next_state;
	localparam [3:0] IDLE = 0;
	always @(posedge pclk or negedge n_rst)
		if (n_rst == 0)
			curr_state <= IDLE;
		else
			curr_state <= next_state;
	localparam [3:0] ADDR = 8;
	localparam [3:0] CLK_DIV = 11;
	localparam [3:0] CONTROL_R = 7;
	localparam [3:0] CONTROL_W = 6;
	localparam [3:0] EIDLE = 4;
	localparam [3:0] RX = 9;
	localparam [3:0] RX_IDLE = 3;
	localparam [3:0] RX_WAIT = 2;
	localparam [3:0] STATUS = 10;
	localparam [3:0] TX = 5;
	localparam [3:0] WAIT = 1;
	always @(*)
		case (curr_state)
			IDLE: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				status_clear = 1'b0;
				if (psel) begin
					if (paddr[5:0] == 6'd0)
						next_state = TX;
					else if (paddr[5:0] == 6'd4) begin
						if (pwrite)
							next_state = CONTROL_W;
						else
							next_state = CONTROL_R;
					end
					else if (paddr[5:0] == 6'd8)
						next_state = ADDR;
					else if (paddr[5:0] == 6'd12)
						next_state = RX;
					else if (paddr[5:0] == 6'd16)
						next_state = STATUS;
					else if (paddr[5:0] == 6'd20)
						next_state = CLK_DIV;
					else
						next_state = EIDLE;
				end
				else
					next_state = curr_state;
			end
			TX: begin
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				tx_w_ena = 1'b1;
				status_clear = 1'b0;
				next_state = IDLE;
			end
			CONTROL_W: begin
				tx_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				cr_w_ena = 1'b1;
				cr_r_ena = 1'b0;
				status_clear = 1'b0;
				next_state = IDLE;
			end
			CONTROL_R: begin
				tx_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				cr_r_ena = 1'b1;
				cr_w_ena = 1'b0;
				status_clear = 1'b0;
				next_state = IDLE;
			end
			ADDR: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				addr_ena = 1'b1;
				status_clear = 1'b0;
				next_state = IDLE;
			end
			RX: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				clk_div_ena = 1'b0;
				rx_r_ena = 2'b11;
				status_clear = 1'b0;
				next_state = RX_IDLE;
			end
			STATUS: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				clk_div_ena = 1'b0;
				rx_r_ena = 2'b00;
				status_clear = 1'b1;
				next_state = IDLE;
			end
			CLK_DIV: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b1;
				status_clear = 1'b0;
				next_state = IDLE;
			end
			WAIT: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				status_clear = 1'b0;
				if (penable == 0)
					next_state = IDLE;
				else
					next_state = curr_state;
			end
			RX_WAIT: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				clk_div_ena = 1'b0;
				rx_r_ena = 2'b10;
				status_clear = 1'b0;
				if (penable == 0)
					next_state = RX_IDLE;
				else
					next_state = curr_state;
			end
			RX_IDLE: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b10;
				clk_div_ena = 1'b0;
				status_clear = 1'b0;
				if (psel) begin
					if (paddr[5:0] == 6'd0)
						next_state = TX;
					else if (paddr[5:0] == 6'd4) begin
						if (pwrite)
							next_state = CONTROL_W;
						else
							next_state = CONTROL_R;
					end
					else if (paddr[5:0] == 6'd8)
						next_state = ADDR;
					else if (paddr[5:0] == 6'd12)
						next_state = RX;
					else if (paddr[5:0] == 6'd16)
						next_state = STATUS;
					else if (paddr[5:0] == 6'd20)
						next_state = CLK_DIV;
					else
						next_state = EIDLE;
				end
				else
					next_state = curr_state;
			end
			EIDLE: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				rx_r_ena = 2'b00;
				clk_div_ena = 1'b0;
				status_clear = 1'b0;
				if (penable == 0)
					next_state = IDLE;
				else
					next_state = EIDLE;
			end
			default: begin
				tx_w_ena = 1'b0;
				cr_r_ena = 1'b0;
				cr_w_ena = 1'b0;
				addr_ena = 1'b0;
				clk_div_ena = 1'b0;
				rx_r_ena = 2'b00;
				status_clear = 1'b0;
				next_state = IDLE;
			end
		endcase
endmodule
