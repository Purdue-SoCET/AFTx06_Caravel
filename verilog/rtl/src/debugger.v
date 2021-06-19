module debugger (
	clk,
	rst_n,
	data_in,
	byte_rcv,
	byte_send,
	HREADY,
	HRDATA,
	start_trans,
	data_send,
	M0_RST,
	HWRITE,
	HSIZE,
	HBURST,
	HTRANS,
	HADDR,
	HWDATA
);
	input clk;
	input rst_n;
	input wire [7:0] data_in;
	input wire byte_rcv;
	input wire byte_send;
	input wire HREADY;
	input wire [31:0] HRDATA;
	output reg start_trans;
	output reg [7:0] data_send;
	output M0_RST;
	output reg HWRITE;
	output reg [2:0] HSIZE;
	output reg [2:0] HBURST;
	output reg [1:0] HTRANS;
	output reg [31:0] HADDR;
	output reg [31:0] HWDATA;
	parameter SIZE = 3;
	reg [3:0] state;
	reg [1:0] set_count_state;
	reg [1:0] set_addr_state;
	reg [2:0] read32_state;
	reg [2:0] write32_state;
	reg [1:0] rcv_x_byte_state;
	reg [1:0] send_x_byte_state;
	reg alive_state;
	reg core_rst_state;
	reg core_norm_state;
	reg [7:0] cmd_reg;
	reg [7:0] crc_reg;
	reg [3:0] rcv_x_byte_ctr;
	reg [31:0] rcv_x_byte_reg;
	reg [31:0] send_x_byte_ctr;
	reg [31:0] send_x_byte_reg;
	reg [31:0] count_reg;
	reg [31:0] read_ctr;
	reg [31:0] write_ctr;
	reg [31:0] addr_reg;
	reg core_rst_reg;
	function [7:0] next_crc8;
		input [7:0] data;
		input [7:0] crc;
		reg [7:0] d;
		reg [7:0] c;
		reg [7:0] newcrc;
		begin
			d = data;
			c = crc;
			newcrc[0] = ((((((d[5] ^ d[4]) ^ d[2]) ^ d[0]) ^ c[0]) ^ c[2]) ^ c[4]) ^ c[5];
			newcrc[1] = ((((((d[6] ^ d[5]) ^ d[3]) ^ d[1]) ^ c[1]) ^ c[3]) ^ c[5]) ^ c[6];
			newcrc[2] = ((((((d[7] ^ d[6]) ^ d[5]) ^ d[0]) ^ c[0]) ^ c[5]) ^ c[6]) ^ c[7];
			newcrc[3] = ((((((((((((d[7] ^ d[6]) ^ d[5]) ^ d[4]) ^ d[2]) ^ d[1]) ^ d[0]) ^ c[0]) ^ c[1]) ^ c[2]) ^ c[4]) ^ c[5]) ^ c[6]) ^ c[7];
			newcrc[4] = ((((((((((d[7] ^ d[6]) ^ d[5]) ^ d[3]) ^ d[2]) ^ d[1]) ^ c[1]) ^ c[2]) ^ c[3]) ^ c[5]) ^ c[6]) ^ c[7];
			newcrc[5] = ((((((((d[7] ^ d[6]) ^ d[4]) ^ d[3]) ^ d[2]) ^ c[2]) ^ c[3]) ^ c[4]) ^ c[6]) ^ c[7];
			newcrc[6] = ((((((d[7] ^ d[3]) ^ d[2]) ^ d[0]) ^ c[0]) ^ c[2]) ^ c[3]) ^ c[7];
			newcrc[7] = ((((d[4] ^ d[3]) ^ d[1]) ^ c[1]) ^ c[3]) ^ c[4];
			next_crc8 = newcrc;
		end
	endfunction
	localparam [1:0] SEND_BYTE = 1;
	localparam [1:0] SEND_X_BYTE_DONE = 3;
	localparam [1:0] START_SEND_X_BYTE = 0;
	localparam [1:0] TRIM_BYTE = 2;
	task send_x_byte;
		case (send_x_byte_state)
			START_SEND_X_BYTE: send_x_byte_state <= SEND_BYTE;
			SEND_BYTE: begin
				start_trans <= 1'b1;
				data_send <= send_x_byte_reg[31:24];
				if (byte_send == 1'b1) begin
					crc_reg <= next_crc8(send_x_byte_reg[31:24], crc_reg);
					send_x_byte_ctr <= send_x_byte_ctr - 1'b1;
					send_x_byte_state <= TRIM_BYTE;
				end
				else
					send_x_byte_state <= SEND_BYTE;
			end
			TRIM_BYTE: begin
				send_x_byte_reg <= {send_x_byte_reg[23:0], 8'b00000000};
				start_trans <= 1'b0;
				if (send_x_byte_ctr == 32'd0)
					send_x_byte_state <= SEND_X_BYTE_DONE;
				else
					send_x_byte_state <= SEND_BYTE;
			end
			SEND_X_BYTE_DONE: begin
				send_x_byte_ctr <= 32'd0;
				send_x_byte_state <= START_SEND_X_BYTE;
			end
		endcase
	endtask
	localparam [0:0] ALIVE_DONE = 1;
	localparam [0:0] ALIVE_START = 0;
	task alive;
		case (alive_state)
			ALIVE_START: begin
				send_x_byte_reg <= 32'hae000000;
				send_x_byte_ctr <= 32'd1;
				send_x_byte;
				if (send_x_byte_state == SEND_X_BYTE_DONE)
					alive_state <= ALIVE_DONE;
				else
					alive_state <= ALIVE_START;
			end
			ALIVE_DONE: alive_state <= ALIVE_START;
		endcase
	endtask
	localparam [0:0] CORE_NORM_DONE = 1;
	localparam [0:0] CORE_NORM_SET = 0;
	task core_norm;
		case (core_norm_state)
			CORE_NORM_SET: begin
				core_rst_reg <= 1'b0;
				core_norm_state <= CORE_NORM_DONE;
			end
			CORE_NORM_DONE: core_norm_state <= CORE_NORM_SET;
		endcase
	endtask
	localparam [0:0] CORE_RST_DONE = 1;
	localparam [0:0] CORE_RST_SET = 0;
	task core_rst;
		case (core_rst_state)
			CORE_RST_SET: begin
				core_rst_reg <= 1'b1;
				core_rst_state <= CORE_RST_DONE;
			end
			CORE_RST_DONE: core_rst_state <= CORE_RST_SET;
		endcase
	endtask
	localparam [1:0] RCV_X_BYTE_DONE = 3;
	localparam [1:0] RECEIVE_BYTE = 2;
	localparam [1:0] START_RCV_X_BYTE = 0;
	localparam [1:0] UPDATE_IDLE = 1;
	task rcv_x_byte;
		case (rcv_x_byte_state)
			START_RCV_X_BYTE: begin
				rcv_x_byte_reg <= 32'd0;
				rcv_x_byte_state <= UPDATE_IDLE;
			end
			UPDATE_IDLE:
				if (rcv_x_byte_ctr == 4'd0)
					rcv_x_byte_state <= RCV_X_BYTE_DONE;
				else
					rcv_x_byte_state <= RECEIVE_BYTE;
			RECEIVE_BYTE:
				if (byte_rcv == 1'd1) begin
					rcv_x_byte_ctr <= rcv_x_byte_ctr - 4'd1;
					rcv_x_byte_reg <= {rcv_x_byte_reg[23:0], data_in};
					crc_reg <= next_crc8(data_in, crc_reg);
					rcv_x_byte_state <= UPDATE_IDLE;
				end
				else
					rcv_x_byte_state <= RECEIVE_BYTE;
			RCV_X_BYTE_DONE: rcv_x_byte_state <= START_RCV_X_BYTE;
		endcase
	endtask
	localparam [2:0] READ32_APHASE = 1;
	localparam [2:0] READ32_APHASE_FINISH = 2;
	localparam [2:0] READ32_DONE = 5;
	localparam [2:0] READ32_DPHASE = 3;
	localparam [2:0] READ32_SEND_DATA = 4;
	localparam [2:0] READ32_START = 0;
	task read32;
		case (read32_state)
			READ32_START: begin
				read32_state <= READ32_APHASE;
				read_ctr <= count_reg;
			end
			READ32_APHASE: begin
				read32_state <= READ32_APHASE_FINISH;
				send_x_byte_state <= START_SEND_X_BYTE;
				HADDR <= addr_reg;
				HTRANS <= 2'b10;
				HSIZE <= 3'b010;
				HBURST <= 3'b000;
				HWRITE <= 1'b0;
			end
			READ32_APHASE_FINISH:
				if (HREADY) begin
					HTRANS <= 2'b00;
					read32_state <= READ32_DPHASE;
				end
				else
					read32_state <= READ32_APHASE_FINISH;
			READ32_DPHASE:
				if (HREADY) begin
					send_x_byte_reg <= HRDATA;
					send_x_byte_ctr <= 32'd4;
					read_ctr <= read_ctr - 1;
					addr_reg <= addr_reg + 4;
					read32_state <= READ32_SEND_DATA;
				end
				else
					read32_state <= READ32_DPHASE;
			READ32_SEND_DATA: begin
				send_x_byte;
				if (send_x_byte_state == SEND_X_BYTE_DONE) begin
					if (read_ctr != 0)
						read32_state <= READ32_APHASE;
					else
						read32_state <= READ32_DONE;
				end
				else
					read32_state <= READ32_SEND_DATA;
			end
			READ32_DONE: read32_state <= READ32_START;
			default: read32_state <= READ32_START;
		endcase
	endtask
	localparam [1:0] ADDR_DONE = 3;
	localparam [1:0] ADDR_RCV_BYTE = 1;
	localparam [1:0] START_SET_ADDR = 0;
	task set_addr;
		case (set_addr_state)
			START_SET_ADDR: begin
				rcv_x_byte_ctr <= 4'd4;
				addr_reg <= 32'd0;
				set_addr_state <= ADDR_RCV_BYTE;
				rcv_x_byte_state <= START_RCV_X_BYTE;
			end
			ADDR_RCV_BYTE: begin
				rcv_x_byte;
				if (rcv_x_byte_state == RCV_X_BYTE_DONE)
					set_addr_state <= ADDR_DONE;
				else
					set_addr_state <= ADDR_RCV_BYTE;
			end
			ADDR_DONE: begin
				addr_reg <= rcv_x_byte_reg;
				set_addr_state <= START_SET_ADDR;
			end
			default: set_addr_state <= START_SET_ADDR;
		endcase
	endtask
	localparam [1:0] COUNT_DONE = 2;
	localparam [1:0] COUNT_RCV_BYTE = 1;
	localparam [1:0] START_SET_COUNT = 0;
	task set_count;
		case (set_count_state)
			START_SET_COUNT: begin
				rcv_x_byte_ctr <= 4'd1;
				count_reg <= 32'd0;
				set_count_state <= COUNT_RCV_BYTE;
			end
			COUNT_RCV_BYTE: begin
				rcv_x_byte;
				if (rcv_x_byte_state == RCV_X_BYTE_DONE) begin
					if (rcv_x_byte_reg <= 32'd0)
						count_reg <= 8'd1;
					else
						count_reg <= rcv_x_byte_reg;
					set_count_state <= COUNT_DONE;
				end
			end
			COUNT_DONE: set_count_state <= START_SET_COUNT;
			default: set_count_state <= START_SET_COUNT;
		endcase
	endtask
	localparam [2:0] WRITE32_ADDR = 1;
	localparam [2:0] WRITE32_CHK_SEND = 3;
	localparam [2:0] WRITE32_DONE = 4;
	localparam [2:0] WRITE32_START = 0;
	localparam [2:0] WRITE32_WRITE_DATA = 2;
	task write32;
		case (write32_state)
			WRITE32_START: begin
				write32_state <= WRITE32_ADDR;
				rcv_x_byte_state <= START_RCV_X_BYTE;
				write_ctr <= count_reg;
				rcv_x_byte_ctr <= 4'd4;
			end
			WRITE32_ADDR: begin
				rcv_x_byte;
				if (rcv_x_byte_state == RCV_X_BYTE_DONE) begin
					HADDR <= addr_reg;
					HSIZE <= 3'b010;
					HBURST <= 3'b000;
					HTRANS <= 2'b10;
					HWRITE <= 1'b1;
					write32_state <= WRITE32_WRITE_DATA;
				end
				else
					write32_state <= WRITE32_ADDR;
			end
			WRITE32_WRITE_DATA: begin
				HWDATA <= rcv_x_byte_reg;
				HTRANS <= 2'b00;
				HWRITE <= 1'b0;
				if (HREADY) begin
					write_ctr <= write_ctr - 1;
					write32_state <= WRITE32_CHK_SEND;
				end
				else
					write32_state <= WRITE32_WRITE_DATA;
			end
			WRITE32_CHK_SEND:
				if (write_ctr == 0)
					write32_state <= WRITE32_DONE;
				else begin
					addr_reg <= addr_reg + 4;
					rcv_x_byte_ctr <= 4'd4;
					write32_state <= WRITE32_ADDR;
				end
			WRITE32_DONE: rcv_x_byte_state <= START_RCV_X_BYTE;
			default: write32_state <= WRITE32_START;
		endcase
	endtask
	localparam [3:0] ALIVE = 6;
	localparam [3:0] CMD_RECD = 1;
	localparam [3:0] CORE_NORM = 8;
	localparam [3:0] CORE_RST = 7;
	localparam [3:0] DONE = 9;
	localparam [3:0] IDLE = 0;
	localparam [3:0] READ32 = 4;
	localparam [3:0] SET_ADDR = 3;
	localparam [3:0] SET_COUNT = 2;
	localparam [3:0] WRITE32 = 5;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			state <= CORE_NORM;
			set_count_state <= START_SET_COUNT;
			set_addr_state <= START_SET_ADDR;
			read32_state <= READ32_START;
			write32_state <= WRITE32_START;
			rcv_x_byte_state <= START_RCV_X_BYTE;
			send_x_byte_state <= START_SEND_X_BYTE;
			alive_state <= ALIVE_START;
			core_rst_state <= CORE_RST_SET;
			core_norm_state <= CORE_NORM_SET;
			cmd_reg <= 8'd0;
			count_reg <= 32'd0;
			addr_reg <= 32'd0;
			rcv_x_byte_ctr <= 4'b0000;
			rcv_x_byte_reg <= 32'd0;
			send_x_byte_ctr <= 32'd0;
			send_x_byte_reg <= 32'd0;
			read_ctr <= 32'd0;
			write_ctr <= 32'd0;
			data_send <= 8'd0;
			start_trans <= 1'b0;
			crc_reg <= 8'd0;
			core_rst_reg <= 1'd1;
			HTRANS <= 2'b00;
			HWRITE <= 1'b0;
			HSIZE <= 3'b000;
			HBURST <= 3'b000;
			HTRANS <= 2'b00;
			HADDR <= 32'd0;
			HWDATA <= 32'd0;
		end
		else
			case (state)
				IDLE: begin
					state <= CMD_RECD;
					rcv_x_byte_ctr <= 4'd1;
					crc_reg <= 8'd0;
				end
				CMD_RECD: begin
					rcv_x_byte;
					if (rcv_x_byte_state == RCV_X_BYTE_DONE)
						if (rcv_x_byte_reg[7] == 1'b1) begin
							cmd_reg <= rcv_x_byte_reg[7:0];
							if (rcv_x_byte_reg[6:0] == SET_COUNT) begin
								state <= SET_COUNT;
								set_count_state <= START_SET_COUNT;
							end
							else if (rcv_x_byte_reg[6:0] == SET_ADDR) begin
								state <= SET_ADDR;
								set_addr_state <= START_SET_ADDR;
							end
							else if (rcv_x_byte_reg[6:0] == READ32) begin
								state <= READ32;
								read32_state <= READ32_START;
							end
							else if (rcv_x_byte_reg[6:0] == WRITE32) begin
								state <= WRITE32;
								write32_state <= WRITE32_START;
							end
							else if (rcv_x_byte_reg[6:0] == ALIVE) begin
								state <= ALIVE;
								alive_state <= ALIVE_START;
							end
							else if (rcv_x_byte_reg[6:0] == CORE_RST) begin
								state <= CORE_RST;
								core_rst_state <= CORE_RST_SET;
							end
							else if (rcv_x_byte_reg[6:0] == CORE_NORM) begin
								state <= CORE_NORM;
								core_norm_state <= CORE_NORM_SET;
							end
							else
								state <= IDLE;
						end
						else
							state <= CMD_RECD;
				end
				SET_COUNT: begin
					set_count;
					if (set_count_state == COUNT_DONE)
						state <= DONE;
				end
				SET_ADDR: begin
					set_addr;
					if (set_addr_state == ADDR_DONE)
						state <= DONE;
				end
				READ32: begin
					read32;
					if (read32_state == READ32_DONE)
						state <= DONE;
				end
				WRITE32: begin
					write32;
					if (write32_state == WRITE32_DONE)
						state <= DONE;
				end
				ALIVE: begin
					alive;
					if (alive_state == ALIVE_DONE)
						state <= DONE;
				end
				CORE_RST: begin
					core_rst;
					if (core_rst_state == CORE_RST_DONE)
						state <= DONE;
				end
				CORE_NORM: begin
					core_norm;
					if (core_norm_state == CORE_NORM_DONE)
						state <= DONE;
				end
				DONE: begin
					send_x_byte_ctr <= 32'd1;
					send_x_byte_reg <= {crc_reg, 24'b000000000000000000000000};
					send_x_byte;
					if (send_x_byte_state == SEND_X_BYTE_DONE) begin
						cmd_reg <= 8'd0;
						state <= IDLE;
					end
				end
				default: state <= IDLE;
			endcase
	assign M0_RST = (core_rst_reg ? 1'b0 : rst_n);
endmodule
