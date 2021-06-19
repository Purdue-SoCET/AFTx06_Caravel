module ahb_slave (
	HCLK,
	HRESETn,
	HMASTLOCK,
	HWRITE,
	HSEL,
	HREADYIN,
	HADDR,
	HWDATA,
	HTRANS,
	HBURST,
	HSIZE,
	HPROT,
	HRDATA,
	HREADYOUT,
	HRESP,
	burst_cancel,
	slave_wait,
	rdata,
	wdata,
	addr,
	r_prep,
	w_prep,
	wen,
	ren,
	size,
	burst_count,
	burst_type
);
	input wire HCLK;
	input wire HRESETn;
	input wire HMASTLOCK;
	input wire HWRITE;
	input wire HSEL;
	input wire HREADYIN;
	input wire [31:0] HADDR;
	input wire [31:0] HWDATA;
	input wire [1:0] HTRANS;
	input wire [2:0] HBURST;
	input wire [2:0] HSIZE;
	input wire [3:0] HPROT;
	output wire [31:0] HRDATA;
	output wire HREADYOUT;
	output wire HRESP;
	input wire burst_cancel;
	input wire slave_wait;
	input wire [31:0] rdata;
	output wire [31:0] wdata;
	output wire [31:0] addr;
	output wire r_prep;
	output wire w_prep;
	output wire wen;
	output wire ren;
	output wire [2:0] size;
	output wire [4:0] burst_count;
	output wire [2:0] burst_type;
	parameter BASE_ADDRESS = 0;
	parameter NUMBER_ADDRESSES = 1024;
	localparam MAX_ADDRESS = (BASE_ADDRESS + NUMBER_ADDRESSES) - 1;
	localparam TRANS_IDLE = 2'b00;
	localparam TRANS_BUSY = 2'b01;
	localparam TRANS_NONSEQ = 2'b10;
	localparam TRANS_SEQ = 2'b11;
	localparam BURST_SINGLE = 3'b000;
	localparam BURST_INCR = 3'b001;
	localparam BURST_WRAP4 = 3'b010;
	localparam BURST_INCR4 = 3'b011;
	localparam BURST_WRAP8 = 3'b100;
	localparam BURST_INCR8 = 3'b101;
	localparam BURST_WRAP16 = 3'b110;
	localparam BURST_INCR16 = 3'b111;
	localparam RESP_OK = 1'b0;
	localparam RESP_ERROR = 1'b1;
	reg [31:0] current;
	reg [31:0] next;
	wire save_addr;
	reg [4:0] count;
	localparam [31:0] IDLE = 0;
	always @(posedge HCLK or negedge HRESETn)
		if (~HRESETn)
			current <= IDLE;
		else begin
			current <= next;
			if (~HRESETn)
				count <= {5 {1'sb0}};
			else if (HTRANS == TRANS_NONSEQ)
				count <= 1;
			else if (HTRANS == TRANS_SEQ)
				count <= count + 1;
			else
				count <= count;
		end
	localparam [31:0] ERROR_C1 = 3;
	localparam [31:0] ERROR_C2 = 4;
	localparam [31:0] READ = 1;
	localparam [31:0] WRITE = 2;
	always @(*)
		casez (current)
			IDLE:
				if (HSEL && HREADYIN) begin
					if ((HADDR > MAX_ADDRESS) | (HTRANS != TRANS_NONSEQ))
						next = ERROR_C1;
					else if (HWRITE)
						next = WRITE;
					else
						next = READ;
				end
				else
					next = IDLE;
			READ:
				if (burst_cancel)
					next = ERROR_C1;
				else if ((HTRANS == TRANS_BUSY) || slave_wait)
					next = READ;
				else if (HREADYIN && HSEL) begin
					if (HADDR > MAX_ADDRESS)
						next = ERROR_C1;
					else if (~HWRITE && HSEL)
						next = READ;
					else if (((HTRANS == TRANS_NONSEQ) && HWRITE) && HSEL)
						next = WRITE;
					else
						next = ERROR_C1;
				end
				else
					next = IDLE;
			WRITE:
				if (burst_cancel)
					next = ERROR_C1;
				else if ((HTRANS == TRANS_BUSY) || slave_wait)
					next = WRITE;
				else if (HREADYIN && HSEL) begin
					if (HADDR > MAX_ADDRESS)
						next = ERROR_C1;
					else if (HWRITE && HSEL)
						next = WRITE;
					else if (((HTRANS == TRANS_NONSEQ) && HSEL) && ~HWRITE)
						next = READ;
					else
						next = ERROR_C1;
				end
				else
					next = IDLE;
			ERROR_C1: next = ERROR_C2;
			ERROR_C2: next = IDLE;
			default: next = IDLE;
		endcase
	assign HRESP = ((current == ERROR_C1) | (current == ERROR_C2) ? RESP_ERROR : RESP_OK);
	assign HREADYOUT = (current == ERROR_C1 ? 1'b0 : (current == ERROR_C2 ? 1'b1 : ~slave_wait));
	assign HRDATA = rdata;
	assign wdata = HWDATA;
	assign addr = HADDR;
	assign burst_type = HBURST;
	assign wen = (current == WRITE) && (HTRANS != TRANS_BUSY);
	assign w_prep = ((HSEL && (HADDR <= MAX_ADDRESS)) && HWRITE) && (HTRANS != TRANS_BUSY);
	assign ren = (current == READ) && (HTRANS != TRANS_BUSY);
	assign r_prep = ((HSEL && (HADDR <= MAX_ADDRESS)) && ~HWRITE) && (HTRANS != TRANS_BUSY);
	assign burst_count = count;
	assign size = HSIZE;
endmodule
