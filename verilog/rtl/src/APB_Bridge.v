module APB_Bridge (
	clk,
	n_rst,
	HTRANS,
	HWRITE,
	HADDR,
	HWDATA,
	PRDATA,
	HREADY,
	HRESP,
	HRDATA,
	PWDATA,
	PADDR,
	PWRITE,
	PENABLE,
	psel_en
);
	input wire clk;
	input wire n_rst;
	input wire [1:0] HTRANS;
	input wire HWRITE;
	input wire [31:0] HADDR;
	input wire [31:0] HWDATA;
	input wire [31:0] PRDATA;
	output wire HREADY;
	output wire HRESP;
	output wire [31:0] HRDATA;
	output wire [31:0] PWDATA;
	output wire [31:0] PADDR;
	output PWRITE;
	output wire PENABLE;
	output wire psel_en;
	reg psel_en_reg;
	reg penable_reg;
	reg hready_reg;
	reg [2:0] current_state;
	reg [2:0] next_state;
	reg [2:0] pre_state;
	reg iwrite;
	reg [31:0] haddr_reg;
	reg [31:0] wdata_reg;
	reg [31:0] iwdata;
	reg [31:0] addr_reg;
	reg [31:0] rdata_reg;
	reg [31:0] new_haddr;
	wire valid;
	parameter [2:0] ST_IDLE = 3'b000;
	parameter [2:0] ST_READ = 3'b001;
	parameter [2:0] ST_RENABLE = 3'b010;
	parameter [2:0] ST_WWAIT = 3'b011;
	parameter [2:0] ST_WRITE = 3'b100;
	parameter [2:0] ST_WRITEP = 3'b101;
	parameter [2:0] ST_WENABLE = 3'b110;
	parameter [2:0] ST_WENABLEP = 3'b111;
	parameter [1:0] TR_IDLE = 2'b00;
	parameter [1:0] TR_BUSY = 2'b01;
	parameter [1:0] TR_NONSEQ = 2'b10;
	parameter [1:0] TR_SEQ = 2'b11;
	assign valid = (((HTRANS == TR_NONSEQ) | (HTRANS == TR_SEQ)) && (HADDR[31:28] == 4'h8) ? 1 : 0);
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0) begin
			pre_state <= ST_IDLE;
			current_state <= ST_IDLE;
		end
		else begin
			pre_state <= current_state;
			current_state <= next_state;
		end
	always @(*)
		if (n_rst == 0)
			next_state = ST_IDLE;
		else
			case (current_state)
				ST_IDLE:
					if (valid == 1) begin
						if (HWRITE == 1)
							next_state = ST_WWAIT;
						else
							next_state = ST_READ;
					end
					else
						next_state = ST_IDLE;
				ST_READ: next_state = ST_RENABLE;
				ST_RENABLE:
					if (valid == 1) begin
						if (HWRITE == 1)
							next_state = ST_WWAIT;
						else
							next_state = ST_READ;
					end
					else
						next_state = ST_IDLE;
				ST_WWAIT:
					if (valid == 1)
						next_state = ST_WRITEP;
					else
						next_state = ST_WRITE;
				ST_WRITE:
					if (valid == 1)
						next_state = ST_WENABLEP;
					else
						next_state = ST_WENABLE;
				ST_WRITEP: next_state = ST_WENABLEP;
				ST_WENABLE:
					if (valid == 1) begin
						if (HWRITE == 1)
							next_state = ST_WWAIT;
						else
							next_state = ST_READ;
					end
					else
						next_state = ST_IDLE;
				ST_WENABLEP:
					if (valid == 1) begin
						if (HWRITE == 1)
							next_state = ST_WRITEP;
						else
							next_state = ST_READ;
					end
					else
						next_state = ST_IDLE;
				default: next_state = ST_IDLE;
			endcase
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			new_haddr <= 32'h00000000;
		else if (next_state == ST_WWAIT)
			new_haddr <= HADDR;
		else
			new_haddr <= new_haddr;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			haddr_reg <= 32'h00000000;
		else if (current_state == ST_WWAIT)
			haddr_reg <= new_haddr;
		else if ((((next_state == ST_READ) | (next_state == ST_WRITE)) | (next_state == ST_WRITEP)) && (pre_state != ST_WWAIT))
			haddr_reg <= HADDR;
		else
			haddr_reg <= haddr_reg;
	always @(posedge clk or negedge n_rst)
		if (n_rst == 0)
			iwdata <= 32'h00000000;
		else if ((next_state == ST_WRITE) | (next_state == ST_WRITEP))
			iwdata <= HWDATA;
		else
			iwdata <= iwdata;
	assign HRDATA = (current_state == ST_RENABLE ? PRDATA : 32'h00000000);
	always @(*)
		if (n_rst == 0) begin
			psel_en_reg = 1'b0;
			penable_reg = 1'b0;
			hready_reg = 1'b0;
			iwrite = 1'b1;
			addr_reg = 32'h00000000;
			wdata_reg = 32'h00000000;
		end
		else
			case (current_state)
				ST_IDLE: begin
					psel_en_reg = 1'b0;
					penable_reg = 1'b0;
					hready_reg = 1'b1;
					iwrite = 1'b0;
					addr_reg = haddr_reg;
					wdata_reg = iwdata;
				end
				ST_READ: begin
					psel_en_reg = 1'b1;
					penable_reg = 1'b0;
					hready_reg = 1'b0;
					iwrite = 1'b0;
					addr_reg = haddr_reg;
					wdata_reg = iwdata;
				end
				ST_WWAIT: begin
					psel_en_reg = 1'b0;
					penable_reg = 1'b0;
					hready_reg = 1'b0;
					iwrite = 1'b1;
					addr_reg = haddr_reg;
					wdata_reg = iwdata;
				end
				ST_WRITE:
					if (pre_state == ST_WWAIT) begin
						psel_en_reg = 1'b1;
						penable_reg = 1'b0;
						hready_reg = 1'b0;
						iwrite = 1'b1;
						addr_reg = new_haddr;
						wdata_reg = iwdata;
					end
					else begin
						psel_en_reg = 1'b1;
						penable_reg = 1'b0;
						hready_reg = 1'b0;
						iwrite = 1'b1;
						addr_reg = haddr_reg;
						wdata_reg = HWDATA;
					end
				ST_WRITEP:
					if (pre_state == ST_WWAIT) begin
						psel_en_reg = 1'b1;
						penable_reg = 1'b0;
						hready_reg = 1'b0;
						iwrite = 1'b1;
						addr_reg = new_haddr;
						wdata_reg = iwdata;
					end
					else begin
						psel_en_reg = 1'b1;
						penable_reg = 1'b0;
						hready_reg = 1'b0;
						iwrite = 1'b1;
						addr_reg = haddr_reg;
						wdata_reg = HWDATA;
					end
				ST_RENABLE: begin
					psel_en_reg = 1'b1;
					penable_reg = 1'b1;
					hready_reg = 1'b1;
					iwrite = 1'b0;
					addr_reg = haddr_reg;
					wdata_reg = iwdata;
				end
				ST_WENABLE: begin
					psel_en_reg = 1'b1;
					penable_reg = 1'b1;
					hready_reg = 1'b1;
					iwrite = 1'b1;
					addr_reg = haddr_reg;
					wdata_reg = iwdata;
				end
				ST_WENABLEP: begin
					psel_en_reg = 1'b1;
					penable_reg = 1'b1;
					hready_reg = 1'b1;
					iwrite = 1'b1;
					addr_reg = haddr_reg;
					wdata_reg = HWDATA;
				end
				default: begin
					psel_en_reg = 1'b0;
					penable_reg = 1'b0;
					hready_reg = 1'b1;
					iwrite = 1'b1;
					wdata_reg = 32'h00000000;
					addr_reg = 32'h00000000;
				end
			endcase
	assign psel_en = psel_en_reg;
	assign PENABLE = penable_reg;
	assign PADDR = addr_reg;
	assign PWDATA = wdata_reg;
	assign PWRITE = iwrite;
	assign HREADY = hready_reg;
	assign HRESP = 1'b0;
endmodule
