module fifo (
	CLK,
	nRST,
	WEN,
	REN,
	wdata,
	watermark,
	rdata,
	numdata,
	empty,
	full,
	half
);
	parameter NUM_BITS = 32;
	parameter buffer_size = 10;
	parameter addr_bits = 4;
	parameter WATER_DIR = 0;
	input wire CLK;
	input wire nRST;
	input wire WEN;
	input wire REN;
	input wire [NUM_BITS - 1:0] wdata;
	input wire [4:0] watermark;
	output wire [NUM_BITS - 1:0] rdata;
	output reg [addr_bits - 1:0] numdata;
	output reg empty;
	output reg full;
	output reg half;
	localparam GREATER_EQUAL = 0;
	localparam LESS_EQUAL = 1;
	reg empty_nxt;
	reg full_nxt;
	wire half_nxt;
	reg [addr_bits - 1:0] wr_ptr;
	reg [addr_bits - 1:0] rd_ptr;
	reg [addr_bits - 1:0] wr_ptr_nxt;
	reg [addr_bits - 1:0] rd_ptr_nxt;
	reg [addr_bits - 1:0] numdata_nxt;
	reg [(buffer_size * NUM_BITS) - 1:0] regs;
	assign rdata = (REN && ~empty ? regs[rd_ptr * NUM_BITS+:NUM_BITS] : {NUM_BITS {1'sb0}});
	always @(posedge CLK or negedge nRST)
		if (!nRST) begin
			empty <= 0;
			full <= 0;
			half <= 0;
			rd_ptr <= {addr_bits {1'sb0}};
			wr_ptr <= {addr_bits {1'sb0}};
			numdata <= {addr_bits {1'sb0}};
		end
		else begin
			empty <= empty_nxt;
			full <= full_nxt;
			half <= half_nxt;
			rd_ptr <= rd_ptr_nxt;
			wr_ptr <= wr_ptr_nxt;
			numdata <= numdata_nxt;
		end
	function automatic [NUM_BITS - 1:0] sv2v_cast_56285;
		input reg [NUM_BITS - 1:0] inp;
		sv2v_cast_56285 = inp;
	endfunction
	always @(negedge CLK or negedge nRST)
		if (!nRST)
			regs <= {buffer_size {sv2v_cast_56285(1'sb0)}};
		else if ((WEN == 1) && (full == 0))
			regs[wr_ptr * NUM_BITS+:NUM_BITS] <= wdata;
	generate
		if (WATER_DIR == LESS_EQUAL) begin
			assign half_nxt = numdata <= watermark;
		end
		else assign half_nxt = numdata >= watermark;
	endgenerate
	always @(*) begin
		wr_ptr_nxt = wr_ptr;
		rd_ptr_nxt = rd_ptr;
		empty_nxt = empty;
		full_nxt = full;
		numdata_nxt = numdata;
		if ((WEN == 1) && (REN == 0)) begin
			if (full == 0) begin
				if (wr_ptr < (buffer_size - 1))
					wr_ptr_nxt = wr_ptr + 1;
				else
					wr_ptr_nxt = {addr_bits {1'sb0}};
				numdata_nxt = numdata + 1;
				empty_nxt = 0;
				if (((wr_ptr + 1) == rd_ptr) || ((wr_ptr == (buffer_size - 1)) && (rd_ptr == {addr_bits {1'sb0}})))
					full_nxt = 1;
				else
					full_nxt = 0;
			end
		end
		else if (((WEN == 0) && (REN == 1)) && (numdata != 0)) begin
			if (empty == 0) begin
				if (rd_ptr < (buffer_size - 1))
					rd_ptr_nxt = rd_ptr + 1;
				else
					rd_ptr_nxt = {addr_bits {1'sb0}};
				numdata_nxt = numdata - 1;
				full_nxt = 0;
				if (((rd_ptr + 1) == wr_ptr) || ((rd_ptr == (buffer_size - 1)) && (wr_ptr == {addr_bits {1'sb0}})))
					empty_nxt = 1;
				else
					empty_nxt = 0;
			end
		end
		else if ((WEN == 1) && (REN == 1)) begin
			if (wr_ptr < (buffer_size - 1))
				wr_ptr_nxt = wr_ptr + 1;
			else
				wr_ptr_nxt = {addr_bits {1'sb0}};
			if (rd_ptr < (buffer_size - 1))
				rd_ptr_nxt = rd_ptr + 1;
			else
				rd_ptr_nxt = {addr_bits {1'sb0}};
		end
	end
endmodule
