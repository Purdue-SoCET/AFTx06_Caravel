module APB_SlaveInterface_general (
	clk,
	n_rst,
	PADDR,
	PWDATA,
	PENABLE,
	PWRITE,
	PSEL,
	PRDATA,
	pslverr,
	read_data,
	w_enable,
	r_enable,
	w_data
);
	parameter NUM_REGS = 2;
	parameter ADDR_OFFSET = 11'h000;
	input wire clk;
	input wire n_rst;
	input wire [31:0] PADDR;
	input wire [31:0] PWDATA;
	input wire PENABLE;
	input wire PWRITE;
	input wire PSEL;
	output wire [31:0] PRDATA;
	output wire pslverr;
	input wire [(NUM_REGS * 32) - 1:0] read_data;
	output wire [NUM_REGS - 1:0] w_enable;
	output wire [NUM_REGS - 1:0] r_enable;
	output wire [31:0] w_data;
	parameter NUM_REGS_WIDTH = $clog2(NUM_REGS);
	parameter BYTES_PER_WORD = 4;
	reg [31:0] state;
	reg [31:0] nextstate;
	reg [NUM_REGS:0] w_enable_reg;
	reg [NUM_REGS:0] r_enable_reg;
	reg [31:0] prdata_reg;
	reg pslverr_reg;
	reg address_match;
	reg [NUM_REGS - 1:0] address_sel;
	reg [NUM_REGS_WIDTH - 1:0] address_index;
	reg [NUM_REGS - 1:0] i;
	wire [11:0] slave_reg;
	assign w_enable = w_enable_reg;
	assign r_enable = r_enable_reg;
	assign PRDATA = prdata_reg;
	assign pslverr = pslverr_reg;
	assign w_data = PWDATA;
	assign slave_reg = PADDR[11:0];
	always @(*) begin
		address_match = 1'b0;
		address_sel = 0;
		address_index = 0;
		for (i = 0; i < NUM_REGS; i = i + 1)
			if (slave_reg == ((i * BYTES_PER_WORD) + ADDR_OFFSET)) begin
				address_match = 1'b1;
				address_sel = 1 << i;
				address_index = i;
			end
	end
	localparam [31:0] IDLE = 0;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			state <= IDLE;
		else
			state <= nextstate;
	localparam [31:0] ACCESS = 1;
	localparam [31:0] ERROR = 2;
	always @(*)
		case (state)
			IDLE:
				if (PSEL == 1) begin
					if (address_match)
						nextstate = ACCESS;
					else
						nextstate = ERROR;
				end
				else
					nextstate = IDLE;
			ACCESS: nextstate = IDLE;
			ERROR: nextstate = IDLE;
			default: nextstate = IDLE;
		endcase
	always @(*)
		case (state)
			IDLE: begin
				w_enable_reg = 0;
				r_enable_reg = 0;
				prdata_reg = 0;
				pslverr_reg = 1'b0;
			end
			ACCESS:
				if (PWRITE == 1) begin
					w_enable_reg = address_sel;
					r_enable_reg = 0;
					prdata_reg = 0;
					pslverr_reg = 0;
				end
				else begin
					w_enable_reg = 0;
					r_enable_reg = address_sel;
					prdata_reg = read_data[address_index * 32+:32];
					pslverr_reg = 1'b0;
				end
			ERROR: begin
				w_enable_reg = 0;
				r_enable_reg = 0;
				prdata_reg = 32'hbad1bad1;
				pslverr_reg = 1'b1;
			end
			default: begin
				w_enable_reg = 0;
				r_enable_reg = 0;
				prdata_reg = 0;
				pslverr_reg = 0;
			end
		endcase
endmodule
