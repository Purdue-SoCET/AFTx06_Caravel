module reset_synchronizer_n (
	clk,
	async_reset,
	sync_reset
);
	input wire clk;
	input wire async_reset;
	output reg sync_reset;
	reg reset_flop;
	always @(posedge clk or negedge async_reset)
		if (!async_reset)
			{sync_reset, reset_flop} <= 2'b00;
		else
			{sync_reset, reset_flop} <= {reset_flop, 1'b1};
endmodule
module top_level_bASIC (
	asyncrst_n,
	clk_sel,
	user_clock2,
	uart_debug_rx,
	uart_debug_tx,
	gpio_bidir_io,
	gpio_output_en_low,
	pwm_w_data_0,
	timer_bidir_0,
	timer_output_en_low,
	SDA_bi,
	SDA_output_en_low,
	SCL_bi,
	SCL_output_en_low,
	SS_bi,
	SS_output_en_low,
	SCK_bi,
	SCK_output_en_low,
	MOSI_bi,
	MOSI_output_en_low,
	MISO_bi,
	MISO_output_en_low,
	wb_clk_i,
	wb_rst_i,
	wbs_stb_i,
	wbs_cyc_i,
	wbs_we_i,
	wbs_sel_i,
	wbs_dat_i,
	wbs_adr_i,
	wbs_ack_o,
	wbs_dat_o,
	la_data_out,
	irq
);
	parameter NUM_GPIO_PINS = 8;
	parameter NUM_PWM_PINS = 1;
	parameter NUM_TIMER_PINS = 1;
	parameter SRAM_DEPTH = 32'h3fffffff;
	parameter NUM_SRAM = 1;
	parameter NUM_AHB_SLAVES = 3;
	parameter NUM_APB_SLAVES = 6;
	input wire asyncrst_n;
	input wire clk_sel;
	input wire user_clock2;
	input wire uart_debug_rx;
	output wire uart_debug_tx;
	inout wire [NUM_GPIO_PINS - 1:0] gpio_bidir_io;
	output wire [NUM_GPIO_PINS - 1:0] gpio_output_en_low;
	output wire [NUM_PWM_PINS - 1:0] pwm_w_data_0;
	inout wire [NUM_TIMER_PINS - 1:0] timer_bidir_0;
	output wire [NUM_TIMER_PINS - 1:0] timer_output_en_low;
	inout wire SDA_bi;
	output wire SDA_output_en_low;
	inout wire SCL_bi;
	output wire SCL_output_en_low;
	inout wire SS_bi;
	output wire SS_output_en_low;
	inout wire SCK_bi;
	output wire SCK_output_en_low;
	inout wire MOSI_bi;
	output wire MOSI_output_en_low;
	inout wire MISO_bi;
	output wire MISO_output_en_low;
	input wire wb_clk_i;
	input wire wb_rst_i;
	input wire wbs_stb_i;
	input wire wbs_cyc_i;
	input wire wbs_we_i;
	input wire [3:0] wbs_sel_i;
	input wire [31:0] wbs_dat_i;
	input wire [31:0] wbs_adr_i;
	output wire wbs_ack_o;
	output wire [31:0] wbs_dat_o;
	output reg [127:0] la_data_out;
	output wire [2:0] irq;
	localparam GPIO0_APB_IDX = 0;
	localparam PWM0_APB_IDX = 1;
	localparam TIMER0_APB_IDX = 2;
	localparam SPI0_APB_IDX = 3;
	localparam I2C0_APB_IDX = 4;
	localparam PWM_PIN_COUNT = NUM_PWM_PINS;
	localparam TIMER_PIN_COUNT = NUM_TIMER_PINS;
	localparam GPIO_PIN_COUNT = NUM_GPIO_PINS;
	localparam N_INTERRUPTS = 32;
	wire clk;
	assign clk = (clk_sel ? user_clock2 : wb_clk_i);
	generate
		if (1) begin : offchip_sramif
			wire [31:0] external_bidir;
			reg nCE;
			reg nOE;
			reg [3:0] nWE;
			wire [3:0] WE;
			wire [31:0] external_rdata;
			wire [31:0] external_wdata;
			reg [18:0] external_addr;
		end
		wire offchip_sramif_external_bidir;
		assign offchip_sramif_external_bidir = offchip_sramif.external_bidir;
	endgenerate
	wire offchip_sramif_external_addr;
	assign offchip_sramif_external_addr = offchip_sramif.external_addr;
	wire offchip_sramif_nWE_out;
	assign offchip_sramif_nWE_out = offchip_sramif.nWE;
	wire offchip_sramif_WE_out;
	assign offchip_sramif_WE_out = ~offchip_sramif.nWE;
	wire offchip_sramif_nOE;
	assign offchip_sramif_nOE = offchip_sramif.nOE;
	reg rst_n;
	wire sync_rst_n;
	wire select_slave;
	generate
		if (1) begin : muxed_ahb_if
			wire [1:0] HTRANS;
			reg [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			reg HREADY;
			wire HREADYOUT;
			wire HSEL;
			wire HMASTLOCK;
			reg [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	generate
		localparam signed [31:0] _param_1E5D4_NUM_PINS = GPIO_PIN_COUNT;
		if (1) begin : gpioIf0
			wire [7:0] gpio_bidir;
			localparam NUM_PINS = _param_1E5D4_NUM_PINS;
			wire [_param_1E5D4_NUM_PINS - 1:0] interrupt;
			reg [_param_1E5D4_NUM_PINS - 1:0] r_data;
			reg [_param_1E5D4_NUM_PINS - 1:0] w_data;
			wire [_param_1E5D4_NUM_PINS - 1:0] en_data;
		end
		assign gpio_bidir_io = gpioIf0.gpio_bidir;
	endgenerate
	generate
		if (1) begin : gpio0apbIf
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		if (1) begin : pwm0If
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		if (1) begin : apb_timer0If
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		localparam signed [31:0] _param_AF95A_NUM_CHANNELS = TIMER_PIN_COUNT;
		if (1) begin : pin_timer0If
			wire timer_bidir;
			localparam NUM_CHANNELS = _param_AF95A_NUM_CHANNELS;
			wire [_param_AF95A_NUM_CHANNELS:0] IRQ;
			reg [_param_AF95A_NUM_CHANNELS - 1:0] r_data;
			reg [_param_AF95A_NUM_CHANNELS - 1:0] w_data;
			wire [_param_AF95A_NUM_CHANNELS - 1:0] output_en;
		end
		assign timer_bidir_0 = pin_timer0If.timer_bidir;
	endgenerate
	generate
		if (1) begin : apb_i2c0If
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		if (1) begin : i2c
			reg SDA;
			reg SCL;
			reg SDA_out;
			reg SCL_out;
			wire interrupt;
		end
	endgenerate
	generate
		if (1) begin : apb_spi0If
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		if (1) begin : spiif
			reg MISO_IN;
			wire MISO_OUT;
			reg MOSI_IN;
			wire MOSI_OUT;
			reg SCK_IN;
			wire SCK_OUT;
			reg SS_IN;
			wire SS_OUT;
			wire mode;
			wire [5:0] interrupts;
		end
	endgenerate
	generate
		localparam signed [31:0] _param_A99B3_NUM_SLAVES = 5;
		if (1) begin : ahb2apbIf
			localparam NUM_SLAVES = _param_A99B3_NUM_SLAVES;
			wire [159:0] PRData_slave;
			wire [4:0] PSEL_slave;
		end
	endgenerate
	generate
		localparam signed [31:0] _param_9891C_N_SRAM = 1;
		if (1) begin : sramIf
			localparam N_SRAM = _param_9891C_N_SRAM;
			wire wen;
			wire [31:0] ram_rData;
			wire [31:0] ram_wData;
			wire [31:0] addr;
			wire [3:0] byte_en;
			wire [0:0] sram_en;
			wire ram_en;
			wire sram_wait;
		end
	endgenerate
	generate
		if (1) begin : blkif
			wire wen;
			reg [31:0] sram_rdata;
			reg [31:0] ram_rdata;
			reg [31:0] rom_rdata;
			wire [31:0] rdata;
			wire [31:0] wdata;
			wire [31:0] addr;
			wire [3:0] byte_en;
			wire sram_wait;
			wire ram_wait;
			wire rom_wait;
			wire mem_wait;
			wire sram_active;
			wire ram_active;
			wire rom_active;
			wire ram_en;
			wire already_got_rom;
			wire already_got_ram;
		end
	endgenerate
	wire [31:0] plicIf_hw_interrupt_requests;
	wire plicIf_interrupt_service_request;
	wire plicIf_interrupt_clear;
	generate
		if (1) begin : interrupt_if
			wire ext_int;
			wire ext_int_clear;
			wire soft_int;
			wire soft_int_clear;
			wire timer_int;
			wire timer_int_clear;
		end
	endgenerate
	generate
		if (1) begin : sramIf_ahbif
			wire [1:0] HTRANS;
			wire [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			wire HREADY;
			wire HREADYOUT;
			reg HSEL;
			wire HMASTLOCK;
			wire [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	generate
		if (1) begin : ahb2apbIf_ahbif
			wire [1:0] HTRANS;
			wire [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			wire HREADY;
			wire HREADYOUT;
			reg HSEL;
			wire HMASTLOCK;
			wire [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	generate
		if (1) begin : wb_ahbif
			wire [1:0] HTRANS;
			wire [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			wire HREADY;
			wire HREADYOUT;
			wire HSEL;
			wire HMASTLOCK;
			wire [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	generate
		if (1) begin : ahb2apbIf_apbif
			wire [31:0] PRDATA;
			wire [31:0] PWDATA;
			wire [31:0] PADDR;
			wire PWRITE;
			wire PENABLE;
			wire PSEL;
		end
	endgenerate
	generate
		if (1) begin : plic_ahb_if
			wire [1:0] HTRANS;
			wire [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			wire HREADY;
			wire HREADYOUT;
			reg HSEL;
			wire HMASTLOCK;
			wire [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	generate
		if (1) begin : clint_ahb_if
			wire [1:0] HTRANS;
			wire [1:0] HRESP;
			wire [2:0] HSIZE;
			wire [31:0] HADDR;
			wire [31:0] HWDATA;
			wire HWRITE;
			wire HREADY;
			wire HREADYOUT;
			reg HSEL;
			wire HMASTLOCK;
			wire [31:0] HRDATA;
			wire [2:0] HBURST;
			wire [3:0] HPROT;
		end
	endgenerate
	wire clintIf_soft_int;
	wire clintIf_clear_soft_int;
	wire clintIf_timer_int;
	wire clintIf_clear_timer_int;
	reg [127:0] la_data_nxt;
	assign interrupt_if.soft_int = clintIf_soft_int;
	assign interrupt_if.soft_int_clear = clintIf_clear_soft_int;
	assign interrupt_if.timer_int = clintIf_timer_int;
	assign interrupt_if.timer_int_clear = clintIf_clear_timer_int;
	assign interrupt_if.ext_int = plicIf_interrupt_service_request;
	assign interrupt_if.ext_int_clear = plicIf_interrupt_clear;
	assign irq[2:0] = {interrupt_if.ext_int, interrupt_if.timer_int, interrupt_if.soft_int};
	localparam [0:0] ADDR = 0;
	localparam [3:0] CLEAR_PREP = 1;
	localparam [3:0] CLEAR_UPDATE = 3;
	localparam [3:0] CLEAR_WB = 2;
	localparam [0:0] DATA = 1;
	localparam [31:0] DATA_INSTR_REQ = 5;
	localparam [31:0] DATA_REQ = 4;
	localparam [31:0] DATA_WAIT = 6;
	localparam [3:0] EVAL = 4;
	localparam [3:0] FETCH = 5;
	localparam [31:0] IDLE = 0;
	localparam [31:0] INSTR_DATA_REQ = 2;
	localparam [31:0] INSTR_REQ = 1;
	localparam [31:0] INSTR_WAIT = 3;
	localparam [3:0] PREFETCH = 6;
	localparam [3:0] PREFETCH_PREP = 7;
	localparam [3:0] PREFETCH_WB = 8;
	localparam [0:0] SASA_COND_OR = 1'b0;
	localparam [3:0] WB = 9;
	localparam [3:0] alu_types_pkg_ALU_ADD = 4'b0011;
	localparam [3:0] alu_types_pkg_ALU_AND = 4'b0101;
	localparam [3:0] alu_types_pkg_ALU_OR = 4'b0110;
	localparam [3:0] alu_types_pkg_ALU_SLL = 4'b0000;
	localparam [3:0] alu_types_pkg_ALU_SLT = 4'b1000;
	localparam [3:0] alu_types_pkg_ALU_SLTU = 4'b1001;
	localparam [3:0] alu_types_pkg_ALU_SRA = 4'b0010;
	localparam [3:0] alu_types_pkg_ALU_SRL = 4'b0001;
	localparam [3:0] alu_types_pkg_ALU_SUB = 4'b0100;
	localparam [3:0] alu_types_pkg_ALU_XOR = 4'b0111;
	localparam [1:0] machine_mode_types_1_11_pkg_BASE_RV32 = 2'h1;
	localparam [30:0] machine_mode_types_1_11_pkg_BREAKPOINT = 31'd3;
	localparam [30:0] machine_mode_types_1_11_pkg_ENV_CALL_M = 31'd11;
	localparam [30:0] machine_mode_types_1_11_pkg_ENV_CALL_S = 31'd9;
	localparam [30:0] machine_mode_types_1_11_pkg_ENV_CALL_U = 31'd8;
	localparam [30:0] machine_mode_types_1_11_pkg_EXT_INT_M = 31'd11;
	localparam [30:0] machine_mode_types_1_11_pkg_EXT_INT_S = 31'd9;
	localparam [30:0] machine_mode_types_1_11_pkg_EXT_INT_U = 31'd8;
	localparam [1:0] machine_mode_types_1_11_pkg_FS_DIRTY = 2'h3;
	localparam [1:0] machine_mode_types_1_11_pkg_FS_OFF = 2'h0;
	localparam [30:0] machine_mode_types_1_11_pkg_ILLEGAL_INSN = 31'd2;
	localparam [30:0] machine_mode_types_1_11_pkg_INSN_ACCESS = 31'd1;
	localparam [30:0] machine_mode_types_1_11_pkg_INSN_MAL = 31'd0;
	localparam [30:0] machine_mode_types_1_11_pkg_INSN_PAGE = 31'd12;
	localparam [30:0] machine_mode_types_1_11_pkg_LOAD_PAGE = 31'd13;
	localparam [30:0] machine_mode_types_1_11_pkg_L_ADDR_MAL = 31'd4;
	localparam [30:0] machine_mode_types_1_11_pkg_L_FAULT = 31'd5;
	localparam [11:0] machine_mode_types_1_11_pkg_MARCHID_ADDR = 12'hf12;
	localparam [11:0] machine_mode_types_1_11_pkg_MCAUSE_ADDR = 12'h342;
	localparam [11:0] machine_mode_types_1_11_pkg_MCYCLEH_ADDR = 12'hb80;
	localparam [11:0] machine_mode_types_1_11_pkg_MCYCLE_ADDR = 12'hb00;
	localparam [11:0] machine_mode_types_1_11_pkg_MEDELEG_ADDR = 12'h302;
	localparam [11:0] machine_mode_types_1_11_pkg_MEPC_ADDR = 12'h341;
	localparam [11:0] machine_mode_types_1_11_pkg_MHARTID_ADDR = 12'hf14;
	localparam [11:0] machine_mode_types_1_11_pkg_MIDELEG_ADDR = 12'h303;
	localparam [11:0] machine_mode_types_1_11_pkg_MIE_ADDR = 12'h304;
	localparam [11:0] machine_mode_types_1_11_pkg_MIMPID_ADDR = 12'hf13;
	localparam [11:0] machine_mode_types_1_11_pkg_MINSTRETH_ADDR = 12'hb82;
	localparam [11:0] machine_mode_types_1_11_pkg_MINSTRET_ADDR = 12'hb02;
	localparam [11:0] machine_mode_types_1_11_pkg_MIP_ADDR = 12'h344;
	localparam [11:0] machine_mode_types_1_11_pkg_MISA_ADDR = 12'h301;
	localparam [11:0] machine_mode_types_1_11_pkg_MSCRATCH_ADDR = 12'h340;
	localparam [11:0] machine_mode_types_1_11_pkg_MSTATUS_ADDR = 12'h300;
	localparam [11:0] machine_mode_types_1_11_pkg_MTVAL_ADDR = 12'h343;
	localparam [11:0] machine_mode_types_1_11_pkg_MTVEC_ADDR = 12'h305;
	localparam [11:0] machine_mode_types_1_11_pkg_MVENDORID_ADDR = 12'hf11;
	localparam [1:0] machine_mode_types_1_11_pkg_M_LEVEL = 2'h3;
	localparam [30:0] machine_mode_types_1_11_pkg_SOFT_INT_M = 31'd3;
	localparam [30:0] machine_mode_types_1_11_pkg_SOFT_INT_S = 31'd1;
	localparam [30:0] machine_mode_types_1_11_pkg_SOFT_INT_U = 31'd0;
	localparam [30:0] machine_mode_types_1_11_pkg_STORE_PAGE = 31'd15;
	localparam [30:0] machine_mode_types_1_11_pkg_S_ADDR_MAL = 31'd6;
	localparam [30:0] machine_mode_types_1_11_pkg_S_FAULT = 31'd7;
	localparam [30:0] machine_mode_types_1_11_pkg_TIMER_INT_M = 31'd7;
	localparam [30:0] machine_mode_types_1_11_pkg_TIMER_INT_S = 31'd5;
	localparam [30:0] machine_mode_types_1_11_pkg_TIMER_INT_U = 31'd4;
	localparam [1:0] machine_mode_types_1_11_pkg_VECTORED = 2'h1;
	localparam [1:0] machine_mode_types_1_11_pkg_XS_ALL_OFF = 2'h0;
	localparam [1:0] machine_mode_types_1_11_pkg_XS_SOME_D = 2'h3;
	localparam [2:0] rv32i_types_pkg_ADDI = 3'b000;
	localparam [2:0] rv32i_types_pkg_ADDSUB = 3'b000;
	localparam [2:0] rv32i_types_pkg_AND = 3'b111;
	localparam [2:0] rv32i_types_pkg_ANDI = 3'b111;
	localparam [6:0] rv32i_types_pkg_AUIPC = 7'b0010111;
	localparam [2:0] rv32i_types_pkg_BEQ = 3'b000;
	localparam [2:0] rv32i_types_pkg_BGE = 3'b101;
	localparam [2:0] rv32i_types_pkg_BGEU = 3'b111;
	localparam [2:0] rv32i_types_pkg_BLT = 3'b100;
	localparam [2:0] rv32i_types_pkg_BLTU = 3'b110;
	localparam [2:0] rv32i_types_pkg_BNE = 3'b001;
	localparam [6:0] rv32i_types_pkg_BRANCH = 7'b1100011;
	localparam [2:0] rv32i_types_pkg_CSRRC = 3'b011;
	localparam [2:0] rv32i_types_pkg_CSRRCI = 3'b111;
	localparam [2:0] rv32i_types_pkg_CSRRS = 3'b010;
	localparam [2:0] rv32i_types_pkg_CSRRSI = 3'b110;
	localparam [2:0] rv32i_types_pkg_CSRRW = 3'b001;
	localparam [2:0] rv32i_types_pkg_CSRRWI = 3'b101;
	localparam [11:0] rv32i_types_pkg_EBREAK = 12'b000000000001;
	localparam [11:0] rv32i_types_pkg_ECALL = 12'b000000000000;
	localparam [2:0] rv32i_types_pkg_FENCEI = 3'b001;
	localparam [6:0] rv32i_types_pkg_IMMED = 7'b0010011;
	localparam [6:0] rv32i_types_pkg_JAL = 7'b1101111;
	localparam [6:0] rv32i_types_pkg_JALR = 7'b1100111;
	localparam [2:0] rv32i_types_pkg_LB = 3'b000;
	localparam [2:0] rv32i_types_pkg_LBU = 3'b100;
	localparam [2:0] rv32i_types_pkg_LH = 3'b001;
	localparam [2:0] rv32i_types_pkg_LHU = 3'b101;
	localparam [6:0] rv32i_types_pkg_LOAD = 7'b0000011;
	localparam [6:0] rv32i_types_pkg_LUI = 7'b0110111;
	localparam [2:0] rv32i_types_pkg_LW = 3'b010;
	localparam [6:0] rv32i_types_pkg_MISCMEM = 7'b0001111;
	localparam [11:0] rv32i_types_pkg_MRET = 12'b001100000010;
	localparam [2:0] rv32i_types_pkg_OR = 3'b110;
	localparam [2:0] rv32i_types_pkg_ORI = 3'b110;
	localparam [2:0] rv32i_types_pkg_PRIV = 3'b000;
	localparam [6:0] rv32i_types_pkg_REGREG = 7'b0110011;
	localparam [2:0] rv32i_types_pkg_SLL = 3'b001;
	localparam [2:0] rv32i_types_pkg_SLLI = 3'b001;
	localparam [2:0] rv32i_types_pkg_SLT = 3'b010;
	localparam [2:0] rv32i_types_pkg_SLTI = 3'b010;
	localparam [2:0] rv32i_types_pkg_SLTIU = 3'b011;
	localparam [2:0] rv32i_types_pkg_SLTU = 3'b011;
	localparam [2:0] rv32i_types_pkg_SR = 3'b101;
	localparam [2:0] rv32i_types_pkg_SRI = 3'b101;
	localparam [6:0] rv32i_types_pkg_STORE = 7'b0100011;
	localparam [6:0] rv32i_types_pkg_SYSTEM = 7'b1110011;
	localparam [2:0] rv32i_types_pkg_XOR = 3'b100;
	localparam [2:0] rv32i_types_pkg_XORI = 3'b100;
	generate
		if (1) begin : corewrap
			wire clk;
			wire rst_n;
			wire wbs_stb_i;
			wire wbs_cyc_i;
			wire wbs_we_i;
			wire [3:0] wbs_sel_i;
			wire [31:0] wbs_dat_i;
			wire [31:0] wbs_adr_i;
			wire wbs_ack_o;
			wire [31:0] wbs_dat_o;
			wire rx;
			wire tx;
			wire TXEV;
			wire LOCKUP;
			wire SYSRESETREQ;
			wire SLEEPING;
			wire M0_RST;
			reg nrst_delayed;
			genvar _arr_9EBF7;
			for (_arr_9EBF7 = 2; _arr_9EBF7 >= 0; _arr_9EBF7 = _arr_9EBF7 - 1) begin : ahb_ifs
				reg [1:0] HTRANS;
				wire [1:0] HRESP;
				reg [2:0] HSIZE;
				reg [31:0] HADDR;
				reg [31:0] HWDATA;
				reg HWRITE;
				wire HREADY;
				wire HREADYOUT;
				wire HSEL;
				reg HMASTLOCK;
				wire [31:0] HRDATA;
				reg [2:0] HBURST;
				reg [3:0] HPROT;
			end
			reg wb_state;
			reg nwb_state;
			always @(posedge clk or negedge rst_n)
				if (~rst_n)
					wb_state <= ADDR;
				else
					wb_state <= nwb_state;
			always @(*) begin
				nwb_state = wb_state;
				if (wb_state == DATA) begin
					if (!ahb_ifs[2].HREADY)
						nwb_state = wb_state;
					else
						nwb_state = ADDR;
				end
				else if (ahb_ifs[2].HREADY)
					nwb_state = DATA;
				else
					nwb_state = ADDR;
			end
			always @(posedge clk or negedge rst_n)
				if (!rst_n)
					nrst_delayed <= 1'b0;
				else
					nrst_delayed <= 1'b1;
			wire [4:1] sv2v_tmp_E6136;
			assign sv2v_tmp_E6136 = {4 {1'sb0}};
			always @(*) ahb_ifs[2].HPROT = sv2v_tmp_E6136;
			wire [1:1] sv2v_tmp_925DC;
			assign sv2v_tmp_925DC = wbs_cyc_i;
			always @(*) ahb_ifs[2].HMASTLOCK = sv2v_tmp_925DC;
			assign wbs_ack_o = (wbs_cyc_i ? ahb_ifs[2].HREADY && rst_n : 1'b0);
			assign wbs_dat_o[31:0] = (wbs_cyc_i ? ahb_ifs[2].HRDATA : {32 {1'sb0}});
			wire [1:1] sv2v_tmp_7E643;
			assign sv2v_tmp_7E643 = (wbs_cyc_i ? wbs_we_i : 1'b0);
			always @(*) ahb_ifs[2].HWRITE = sv2v_tmp_7E643;
			wire [3:1] sv2v_tmp_CB718;
			assign sv2v_tmp_CB718 = (wbs_cyc_i ? wbs_sel_i : {3 {1'sb0}});
			always @(*) ahb_ifs[2].HSIZE = sv2v_tmp_CB718;
			wire [3:1] sv2v_tmp_A4211;
			assign sv2v_tmp_A4211 = {3 {1'sb0}};
			always @(*) ahb_ifs[2].HBURST = sv2v_tmp_A4211;
			wire [2:1] sv2v_tmp_D2673;
			assign sv2v_tmp_D2673 = (wbs_cyc_i ? (wbs_stb_i ? ((wb_state == ADDR) && rst_n ? 2'b10 : 2'b00) : 2'b01) : 2'b00);
			always @(*) ahb_ifs[2].HTRANS = sv2v_tmp_D2673;
			wire [32:1] sv2v_tmp_384EE;
			assign sv2v_tmp_384EE = (wbs_cyc_i ? wbs_adr_i : {32 {1'sb0}});
			always @(*) ahb_ifs[2].HADDR = sv2v_tmp_384EE;
			wire [32:1] sv2v_tmp_29ECD;
			assign sv2v_tmp_29ECD = (wbs_cyc_i ? wbs_dat_i : {32 {1'sb0}});
			always @(*) ahb_ifs[2].HWDATA = sv2v_tmp_29ECD;
			localparam _bbase_91D80_m_in = 0;
			localparam signed [31:0] _param_91D80_MM = 3;
			if (1) begin : BUS_MUX
				localparam ARBITRATION = "HIGH";
				localparam MM = _param_91D80_MM;
				wire HCLK;
				wire HRESETn;
				localparam _mbase_m_in = 0;
				wire [2:0] HREADY;
				wire [2:0] HMASTLOCK;
				wire [5:0] HTRANS;
				wire [2:0] ARB_SEL;
				wire [2:0] ARB_SEL_PREV;
				wire [1:0] MASTER_SEL;
				wire [1:0] MASTER_SEL_PREV;
				wire [95:0] HWDATA_out;
				wire [137:0] downstreams;
				wire [45:0] downstream;
				arbiter #(
					.ARBITRATION(ARBITRATION),
					.MM(MM)
				) ARBITER(
					.HCLK(HCLK),
					.HRESETn(HRESETn),
					.HTRANS(HTRANS),
					.HMASTLOCK(HMASTLOCK),
					.HREADY(HREADY),
					.ARB_SEL(ARB_SEL),
					.MASTER_SEL(MASTER_SEL),
					.ARB_SEL_PREV(ARB_SEL_PREV),
					.MASTER_SEL_PREV(MASTER_SEL_PREV)
				);
				assign downstream = downstreams[MASTER_SEL * 46+:46];
				assign top_level_bASIC.muxed_ahb_if.HADDR = downstream[45-:32];
				assign top_level_bASIC.muxed_ahb_if.HTRANS = downstream[2-:2];
				assign top_level_bASIC.muxed_ahb_if.HSIZE = downstream[5-:3];
				assign top_level_bASIC.muxed_ahb_if.HWRITE = downstream[0];
				assign top_level_bASIC.muxed_ahb_if.HBURST = downstream[13-:3];
				assign top_level_bASIC.muxed_ahb_if.HPROT = downstream[9-:4];
				assign top_level_bASIC.muxed_ahb_if.HMASTLOCK = downstream[10];
				assign top_level_bASIC.muxed_ahb_if.HWDATA = HWDATA_out[MASTER_SEL_PREV * 32+:32];
				genvar i;
				for (i = 0; i < MM; i = i + 1) begin
					wire [45:0] upstream_in;
					assign upstream_in = {top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HADDR, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HBURST, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HMASTLOCK, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HPROT, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HSIZE, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HTRANS, top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HWRITE};
					assign HREADY[i] = top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HREADY;
					assign HMASTLOCK[i] = downstreams[(i * 46) + 10];
					assign HTRANS[i * 2+:2] = downstreams[(i * 46) + 2-:2];
					aphase_cache CACHE(
						.HCLK(HCLK),
						.HRESETn(HRESETn),
						.ARB_SEL(ARB_SEL[i]),
						.ARB_SEL_PREV(ARB_SEL_PREV[i]),
						.HREADY_in(top_level_bASIC.muxed_ahb_if.HREADY),
						.HREADY_out(top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HREADY),
						.upstream_in(upstream_in),
						.downstream_out(downstreams[i * 46+:46])
					);
					assign top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HRESP = top_level_bASIC.muxed_ahb_if.HRESP;
					assign top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HRDATA = top_level_bASIC.muxed_ahb_if.HRDATA;
					assign HWDATA_out[i * 32+:32] = top_level_bASIC.corewrap.ahb_ifs[i + _mbase_m_in].HWDATA;
				end
			end
			assign BUS_MUX.HCLK = clk;
			assign BUS_MUX.HRESETn = rst_n;
			localparam _bbase_11B0E_ahb_master = 0;
			if (1) begin : RISCVBusiness
				wire CLK;
				wire nRST;
				localparam _mbase_ahb_master = _bbase_11B0E_ahb_master;
				if (1) begin : tspp_icache_gen_bus_if
					localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
					wire [31:0] addr;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] wdata;
					wire [31:0] rdata;
					wire ren;
					wire wen;
					wire busy;
					wire [3:0] byte_en;
				end
				if (1) begin : tspp_dcache_gen_bus_if
					localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
					wire [31:0] addr;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					reg [31:0] wdata;
					wire [31:0] rdata;
					wire ren;
					wire wen;
					wire busy;
					wire [3:0] byte_en;
				end
				if (1) begin : icache_mc_if
					localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
					wire [31:0] addr;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] wdata;
					wire [31:0] rdata;
					wire ren;
					wire wen;
					reg busy;
					wire [3:0] byte_en;
				end
				if (1) begin : dcache_mc_if
					localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
					wire [31:0] addr;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] wdata;
					wire [31:0] rdata;
					wire ren;
					wire wen;
					reg busy;
					wire [3:0] byte_en;
				end
				if (1) begin : pipeline_trans_if
					localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
					reg [31:0] addr;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] wdata;
					wire [31:0] rdata;
					reg ren;
					reg wen;
					wire busy;
					reg [3:0] byte_en;
				end
				if (1) begin : rm_if
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] insn;
					wire req_reg_r;
					wire req_reg_w;
					wire [4:0] rsel_s_0;
					wire [4:0] rsel_s_1;
					wire [4:0] rsel_d;
					wire [31:0] rdata_s_0;
					wire [31:0] rdata_s_1;
					wire reg_w;
					wire [31:0] reg_wdata;
					wire req_br_j;
					wire branch_jump;
					wire [31:0] br_j_addr;
					wire [31:0] pc;
					wire req_mem;
					wire [31:0] mem_addr;
					wire [31:0] mem_store;
					wire [31:0] mem_load;
					wire mem_ren;
					wire mem_wen;
					wire mem_busy;
					wire [3:0] mem_byte_en;
					wire execute_stall;
					wire memory_stall;
					wire active_insn;
					wire ex_token;
					wire if_ex_enable;
					wire exception;
					wire [0:0] ex_cause;
				end
				if (1) begin : predict_if
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] current_pc;
					wire [31:0] target_addr;
					wire [31:0] update_addr;
					wire update_predictor;
					wire predict_taken;
					wire prediction;
					wire branch_result;
				end
				if (1) begin : prv_pipe_if
					wire fault_insn;
					wire mal_insn;
					wire illegal_insn;
					wire fault_l;
					wire mal_l;
					wire fault_s;
					wire mal_s;
					wire breakpoint;
					wire env_m;
					wire ret;
					wire timer_int;
					wire soft_int;
					wire ext_int;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] epc;
					wire [31:0] priv_pc;
					wire [31:0] badaddr;
					wire insert_pc;
					wire intr;
					wire pipe_clear;
					wire [127:0] xtvec;
					wire [127:0] xepc_r;
					wire swap;
					wire clr;
					wire set;
					wire invalid_csr;
					wire valid_write;
					wire [11:0] addr;
					wire [31:0] rdata;
					wire [31:0] wdata;
					wire wb_enable;
					wire instr;
					wire ex_rmgmt;
					wire [-1:0] ex_rmgmt_cause;
				end
				if (1) begin : cc_if
					wire icache_clear;
					wire icache_flush;
					wire iclear_done;
					wire iflush_done;
					wire dcache_clear;
					wire dcache_flush;
					wire dclear_done;
					wire dflush_done;
				end
				if (1) begin : sparce_if
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] pc;
					wire [31:0] wb_data;
					wire [31:0] sasa_data;
					wire [31:0] sasa_addr;
					wire wb_en;
					wire sasa_wen;
					wire [4:0] rd;
					wire if_ex_enable;
					wire [31:0] sparce_target;
					wire [31:0] rdata;
					wire skipping;
				end
				if (1) begin : fetch_ex_if
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					reg [128:0] fetch_ex_reg;
					wire [31:0] brj_addr;
				end
				if (1) begin : hazard_if
					wire pc_en;
					wire npc_sel;
					wire i_mem_busy;
					wire d_mem_busy;
					wire dren;
					wire dwen;
					wire iren;
					wire ret;
					wire branch_taken;
					wire prediction;
					wire jump;
					wire branch;
					wire if_ex_stall;
					wire fence_stall;
					wire if_ex_flush;
					wire mispredict;
					wire halt;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] pc;
					wire fault_insn;
					wire mal_insn;
					wire illegal_insn;
					wire fault_l;
					wire mal_l;
					wire fault_s;
					wire mal_s;
					wire breakpoint;
					wire env_m;
					wire [31:0] epc_f;
					wire [31:0] epc_e;
					wire [31:0] badaddr_f;
					wire [31:0] badaddr_e;
					wire [31:0] priv_pc;
					wire insert_priv_pc;
					wire token_ex;
				end
				wire halt;
				wire HREADY;
				assign HREADY = top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_master].HREADY;
				if (1) begin : fetch_stage_i
					wire CLK;
					wire nRST;
					localparam RESET_PC = 32'h00000200;
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					reg [31:0] pc;
					wire [31:0] pc4;
					wire [31:0] npc;
					wire [31:0] instr;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							pc <= RESET_PC;
						else if (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.pc_en)
							pc <= npc;
					assign pc4 = pc + 4;
					assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.current_pc = pc;
					assign npc = (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.insert_priv_pc ? top_level_bASIC.corewrap.RISCVBusiness.hazard_if.priv_pc : (top_level_bASIC.corewrap.RISCVBusiness.sparce_if.skipping ? top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sparce_target : (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.npc_sel ? top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.brj_addr : (top_level_bASIC.corewrap.RISCVBusiness.predict_if.predict_taken ? top_level_bASIC.corewrap.RISCVBusiness.predict_if.target_addr : pc4))));
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.i_mem_busy = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.busy;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr = pc;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.ren = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.iren;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.byte_en = 4'b1111;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wdata = {32 {1'sb0}};
					always @(posedge CLK or negedge nRST)
						if (!nRST)
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg <= {129 {1'sb0}};
						else if (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_flush)
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg <= {129 {1'sb0}};
						else if (!top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall) begin
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[128] <= 1'b1;
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32] <= pc;
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[95-:32] <= pc4;
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[63-:32] <= top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata;
							top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[31-:32] <= top_level_bASIC.corewrap.RISCVBusiness.predict_if.predict_taken;
						end
					wire mal_addr;
					assign mal_addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr[1:0] != 2'b00;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_insn = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_insn = mal_addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.badaddr_f = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.epc_f = pc;
					localparam BUS_ENDIANNESS = "big";
					if (BUS_ENDIANNESS == "big") begin
						assign instr = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata;
					end
					else if (BUS_ENDIANNESS == "little") endian_swapper ltb_endian(
						.word_in(top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata),
						.word_out(instr)
					);
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.pc = pc;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata;
				end
				assign fetch_stage_i.CLK = CLK;
				assign fetch_stage_i.nRST = nRST;
				if (1) begin : execute_stage_i
					wire CLK;
					wire nRST;
					reg halt;
					if (1) begin : cu_if
						wire dwen;
						wire dren;
						wire j_sel;
						wire branch;
						wire jump;
						wire ex_pc_sel;
						wire imm_shamt_sel;
						wire halt;
						reg wen;
						wire ifence;
						reg [3:0] alu_op;
						reg [1:0] alu_a_sel;
						reg [1:0] alu_b_sel;
						reg [2:0] w_sel;
						wire [4:0] shamt;
						wire [11:0] imm_I;
						wire [11:0] imm_S;
						wire [20:0] imm_UJ;
						wire [12:0] imm_SB;
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] instr;
						wire [31:0] imm_U;
						localparam rv32i_types_pkg_LD_W = 3;
						wire [2:0] load_type;
						localparam rv32i_types_pkg_BR_W = 3;
						wire [2:0] branch_type;
						localparam rv32i_types_pkg_OP_W = 7;
						wire [6:0] opcode;
						wire fault_insn;
						reg illegal_insn;
						reg ret_insn;
						reg breakpoint;
						reg ecall_insn;
						reg csr_swap;
						reg csr_set;
						reg csr_clr;
						reg csr_imm;
						wire csr_rw_valid;
						wire [11:0] csr_addr;
						wire [4:0] zimm;
					end
					if (1) begin : rf_if
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						reg [31:0] w_data;
						wire [31:0] rs1_data;
						wire [31:0] rs2_data;
						wire [4:0] rs1;
						wire [4:0] rs2;
						wire [4:0] rd;
						wire wen;
					end
					if (1) begin : alu_if
						wire [3:0] aluop;
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						reg [31:0] port_a;
						reg [31:0] port_b;
						reg [31:0] port_out;
					end
					if (1) begin : jump_if
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						reg [31:0] base;
						reg [31:0] offset;
						wire [31:0] jal_addr;
						wire [31:0] jalr_addr;
					end
					if (1) begin : branch_if
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] rs1_data;
						wire [31:0] rs2_data;
						wire [31:0] pc;
						wire [31:0] branch_addr;
						wire [12:0] imm_sb;
						localparam rv32i_types_pkg_BR_W = 3;
						wire [2:0] branch_type;
						reg branch_taken;
					end
					if (1) begin : cu
						wire [4:0] rmgmt_rsel_s_0;
						wire [4:0] rmgmt_rsel_s_1;
						wire [4:0] rmgmt_rsel_d;
						wire rmgmt_req_reg_r;
						wire rmgmt_req_reg_w;
						localparam rv32i_types_pkg_OP_W = 7;
						wire [31:0] instr_s;
						wire [31:0] instr_i;
						wire [31:0] instr_r;
						wire [31:0] instr_sb;
						wire [31:0] instr_u;
						wire [31:0] instr_uj;
						assign instr_s = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign instr_i = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign instr_r = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign instr_sb = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign instr_u = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign instr_uj = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[6:0];
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs1 = (rmgmt_req_reg_r ? rmgmt_rsel_s_0 : top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[19:15]);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs2 = (rmgmt_req_reg_r ? rmgmt_rsel_s_1 : top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[24:20]);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rd = (rmgmt_req_reg_w ? rmgmt_rsel_d : top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[11:7]);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.shamt = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[24:20];
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_I = instr_i[31-:12];
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_S = {instr_s[31-:7], instr_s[11-:5]};
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_SB = {instr_sb[31], instr_sb[7], instr_sb[30-:6], instr_sb[11-:4], 1'b0};
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_UJ = {instr_uj[31], instr_uj[19-:8], instr_uj[20], instr_uj[30-:10], 1'b0};
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_U = {instr_u[31-:20], 12'b000000000000};
						localparam rv32i_types_pkg_IMM_W = 3;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.imm_shamt_sel = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && ((instr_i[14-:3] == rv32i_types_pkg_SLLI) || (instr_i[14-:3] == rv32i_types_pkg_SRI));
						localparam rv32i_types_pkg_LD_W = 3;
						function automatic [2:0] sv2v_cast_3;
							input reg [2:0] inp;
							sv2v_cast_3 = inp;
						endfunction
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.load_type = sv2v_cast_3(instr_i[14-:3]);
						localparam rv32i_types_pkg_BR_W = 3;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.branch_type = sv2v_cast_3(instr_sb[14-:3]);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.dwen = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_STORE;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.dren = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_LOAD;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ifence = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_MISCMEM) && (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_FENCEI);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.branch = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_BRANCH;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.jump = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_JAL) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_JALR);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ex_pc_sel = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_JAL) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_JALR);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.j_sel = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_JAL;
						always @(*)
							case (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode)
								rv32i_types_pkg_REGREG, rv32i_types_pkg_IMMED, rv32i_types_pkg_LOAD: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_a_sel = 2'd0;
								rv32i_types_pkg_STORE: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_a_sel = 2'd1;
								rv32i_types_pkg_AUIPC: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_a_sel = 2'd2;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_a_sel = 2'd2;
							endcase
						always @(*)
							case (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode)
								rv32i_types_pkg_STORE: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_b_sel = 2'd0;
								rv32i_types_pkg_REGREG: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_b_sel = 2'd1;
								rv32i_types_pkg_IMMED, rv32i_types_pkg_LOAD: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_b_sel = 2'd2;
								rv32i_types_pkg_AUIPC: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_b_sel = 2'd3;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_b_sel = 2'd1;
							endcase
						always @(*)
							case (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode)
								rv32i_types_pkg_LOAD: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd0;
								rv32i_types_pkg_JAL, rv32i_types_pkg_JALR: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd1;
								rv32i_types_pkg_LUI: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd2;
								rv32i_types_pkg_IMMED, rv32i_types_pkg_AUIPC, rv32i_types_pkg_REGREG: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd3;
								rv32i_types_pkg_SYSTEM: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd4;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.w_sel = 3'd0;
							endcase
						always @(*)
							case (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode)
								rv32i_types_pkg_STORE, rv32i_types_pkg_BRANCH: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.wen = 1'b0;
								rv32i_types_pkg_IMMED, rv32i_types_pkg_LUI, rv32i_types_pkg_AUIPC, rv32i_types_pkg_REGREG, rv32i_types_pkg_JAL, rv32i_types_pkg_JALR, rv32i_types_pkg_LOAD: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.wen = 1'b1;
								rv32i_types_pkg_SYSTEM: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.wen = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_rw_valid;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.wen = 1'b0;
							endcase
						wire sr;
						wire aluop_srl;
						wire aluop_sra;
						wire aluop_add;
						wire aluop_sub;
						wire aluop_and;
						wire aluop_or;
						wire aluop_sll;
						wire aluop_xor;
						wire aluop_slt;
						wire aluop_sltu;
						wire add_sub;
						localparam rv32i_types_pkg_REG_W = 3;
						assign sr = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_SRI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_SR));
						assign add_sub = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_ADDSUB);
						assign aluop_sll = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_SLLI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_SLL));
						assign aluop_sra = sr && top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[30];
						assign aluop_srl = sr && ~top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[30];
						assign aluop_add = (((((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_ADDI)) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_AUIPC)) || (add_sub && ~top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[30])) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_LOAD)) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_STORE);
						assign aluop_sub = add_sub && top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[30];
						assign aluop_and = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_ANDI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_AND));
						assign aluop_or = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_ORI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_OR));
						assign aluop_xor = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_XORI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_XOR));
						assign aluop_slt = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_SLTI)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_SLT));
						assign aluop_sltu = ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_IMMED) && (instr_i[14-:3] == rv32i_types_pkg_SLTIU)) || ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_REGREG) && (instr_r[14-:3] == rv32i_types_pkg_SLTU));
						always @(*)
							if (aluop_sll)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SLL;
							else if (aluop_sra)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SRA;
							else if (aluop_srl)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SRL;
							else if (aluop_add)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_ADD;
							else if (aluop_sub)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SUB;
							else if (aluop_and)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_AND;
							else if (aluop_or)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_OR;
							else if (aluop_xor)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_XOR;
							else if (aluop_slt)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SLT;
							else if (aluop_sltu)
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_SLTU;
							else
								top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.alu_op = alu_types_pkg_ALU_ADD;
						localparam INFINITE_LOOP_HALTS = "true";
						if (INFINITE_LOOP_HALTS == "true") begin
							assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.halt = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr == 32'h0000006f;
						end
						else assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.halt = 1'b0;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.fault_insn = 1'b0;
						always @(*)
							case (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode)
								rv32i_types_pkg_REGREG: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.illegal_insn = instr_r[25];
								rv32i_types_pkg_LUI, rv32i_types_pkg_AUIPC, rv32i_types_pkg_JAL, rv32i_types_pkg_JALR, rv32i_types_pkg_BRANCH, rv32i_types_pkg_LOAD, rv32i_types_pkg_STORE, rv32i_types_pkg_IMMED, rv32i_types_pkg_SYSTEM, rv32i_types_pkg_MISCMEM, 7'b0000000: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.illegal_insn = 1'b0;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.illegal_insn = 1'b1;
							endcase
						function automatic [11:0] sv2v_cast_12;
							input reg [11:0] inp;
							sv2v_cast_12 = inp;
						endfunction
						always @(*) begin
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ret_insn = 1'b0;
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.breakpoint = 1'b0;
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ecall_insn = 1'b0;
							if (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_SYSTEM)
								if (sv2v_cast_3(instr_i[14-:3]) == rv32i_types_pkg_PRIV) begin
									if (sv2v_cast_12(instr_i[31-:12]) == rv32i_types_pkg_MRET)
										top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ret_insn = 1'b1;
									if (sv2v_cast_12(instr_i[31-:12]) == rv32i_types_pkg_EBREAK)
										top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.breakpoint = 1'b1;
									if (sv2v_cast_12(instr_i[31-:12]) == rv32i_types_pkg_ECALL)
										top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.ecall_insn = 1'b1;
								end
						end
						always @(*) begin
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_swap = 1'b0;
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_clr = 1'b0;
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_set = 1'b0;
							top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_imm = 1'b0;
							if (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.opcode == rv32i_types_pkg_SYSTEM)
								if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRW)
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_swap = 1'b1;
								else if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRS)
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_set = 1'b1;
								else if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRC)
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_clr = 1'b1;
								else if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRWI) begin
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_swap = 1'b1;
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_imm = 1'b1;
								end
								else if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRSI) begin
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_set = 1'b1;
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_imm = 1'b1;
								end
								else if (sv2v_cast_3(instr_r[14-:3]) == rv32i_types_pkg_CSRRCI) begin
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_clr = 1'b1;
									top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_imm = 1'b1;
								end
						end
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_rw_valid = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_swap | top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_set) | top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_clr;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.csr_addr = sv2v_cast_12(instr_i[31-:12]);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.zimm = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.cu_if.instr[19:15];
					end
					assign cu.rmgmt_rsel_s_0 = top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_s_0;
					assign cu.rmgmt_rsel_s_1 = top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_s_1;
					assign cu.rmgmt_rsel_d = top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_d;
					assign cu.rmgmt_req_reg_r = top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_r;
					assign cu.rmgmt_req_reg_w = top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_w;
					if (1) begin : rf
						wire CLK;
						wire nRST;
						localparam NUM_REGS = 32;
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						reg [1023:0] registers;
						always @(posedge CLK or negedge nRST)
							if (~nRST)
								registers <= {1024 {1'sb0}};
							else if (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.wen && top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rd)
								registers[top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rd * 32+:32] <= top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.w_data;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs1_data = registers[top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs1 * 32+:32];
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs2_data = registers[top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.rf_if.rs2 * 32+:32];
					end
					assign rf.CLK = CLK;
					assign rf.nRST = nRST;
					if (1) begin : alu
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] adder_result;
						wire [31:0] op_a;
						wire [31:0] op_b;
						wire [31:0] twos_comp_b;
						wire carry_out;
						wire sign_r;
						wire sign_a;
						wire sign_b;
						wire [32:0] adder_out;
						wire [32:0] op_a_ext;
						wire [32:0] op_b_ext;
						assign sign_r = adder_out[31];
						assign sign_a = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a[31];
						assign sign_b = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b[31];
						assign op_a = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a;
						assign op_b = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.aluop == alu_types_pkg_ALU_ADD ? top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b : twos_comp_b);
						assign twos_comp_b = ~top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b + 1;
						assign op_a_ext[rv32i_types_pkg_WORD_SIZE] = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.aluop == alu_types_pkg_ALU_SLTU ? 1'b0 : op_a[31]);
						assign op_b_ext[rv32i_types_pkg_WORD_SIZE] = (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.aluop == alu_types_pkg_ALU_SLTU ? 1'b0 : op_b[31]);
						assign op_a_ext[31:0] = op_a;
						assign op_b_ext[31:0] = op_b;
						assign adder_out = op_a_ext + op_b_ext;
						assign adder_result = adder_out[31:0];
						assign carry_out = adder_out[rv32i_types_pkg_WORD_SIZE];
						always @(*)
							casez (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.aluop)
								alu_types_pkg_ALU_SLL: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a << top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b[4:0];
								alu_types_pkg_ALU_SRL: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a >> top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b[4:0];
								alu_types_pkg_ALU_SRA: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = $signed(top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a) >>> top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b[4:0];
								alu_types_pkg_ALU_AND: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a & top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b;
								alu_types_pkg_ALU_OR: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a | top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b;
								alu_types_pkg_ALU_XOR: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_a ^ top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_b;
								alu_types_pkg_ALU_SLT: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = (sign_a & !sign_b ? 1 : (!sign_a & sign_b ? 0 : sign_r));
								alu_types_pkg_ALU_SLTU: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = ~carry_out & |op_b_ext;
								alu_types_pkg_ALU_ADD: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = adder_result;
								alu_types_pkg_ALU_SUB: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = adder_result;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.alu_if.port_out = {32 {1'sb0}};
							endcase
					end
					if (1) begin : jump_calc
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] jump_addr;
						assign jump_addr = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.jump_if.base + top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.jump_if.offset;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.jump_if.jal_addr = jump_addr;
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.jump_if.jalr_addr = {jump_addr[31:1], 1'b0};
					end
					if (1) begin : branch_res
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] offset;
						wire lt;
						wire eq;
						wire ltu;
						wire sign_1;
						wire sign_2;
						wire sign_r;
						wire carry_out;
						wire [32:0] adder_out;
						reg [32:0] op_1_ext;
						reg [32:0] op_2_ext;
						assign offset = $signed(top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.imm_sb);
						assign top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_addr = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.pc + offset;
						assign sign_1 = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs1_data[31];
						assign sign_2 = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs2_data[31];
						assign sign_r = adder_out[31];
						wire [32:1] sv2v_tmp_3F0E1;
						assign sv2v_tmp_3F0E1 = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs1_data;
						always @(*) op_1_ext[31:0] = sv2v_tmp_3F0E1;
						wire [32:1] sv2v_tmp_616CB;
						assign sv2v_tmp_616CB = ~top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs2_data + 1;
						always @(*) op_2_ext[31:0] = sv2v_tmp_616CB;
						localparam rv32i_types_pkg_BR_W = 3;
						always @(*)
							if ((top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_type == rv32i_types_pkg_BLTU) || (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_type == rv32i_types_pkg_BGEU)) begin
								op_1_ext[rv32i_types_pkg_WORD_SIZE] = 1'b0;
								op_2_ext[rv32i_types_pkg_WORD_SIZE] = 1'b0;
							end
							else begin
								op_1_ext[rv32i_types_pkg_WORD_SIZE] = op_1_ext[31];
								op_2_ext[rv32i_types_pkg_WORD_SIZE] = op_2_ext[31];
							end
						assign adder_out = op_1_ext + op_2_ext;
						assign carry_out = adder_out[rv32i_types_pkg_WORD_SIZE];
						assign eq = top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs1_data == top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.rs2_data;
						assign lt = (sign_1 & ~sign_2 ? 1 : (~sign_1 & sign_2 ? 0 : sign_r));
						assign ltu = ~carry_out & |op_2_ext;
						always @(*)
							casez (top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_type)
								rv32i_types_pkg_BEQ: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = eq;
								rv32i_types_pkg_BNE: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = ~eq;
								rv32i_types_pkg_BLT: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = lt;
								rv32i_types_pkg_BGE: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = ~lt;
								rv32i_types_pkg_BLTU: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = ltu;
								rv32i_types_pkg_BGEU: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = ~ltu;
								default: top_level_bASIC.corewrap.RISCVBusiness.execute_stage_i.branch_if.branch_taken = 1'b0;
							endcase
					end
					localparam rv32i_types_pkg_WORD_SIZE = 32;
					wire [31:0] store_swapped;
					endian_swapper store_swap(
						.word_in(rf_if.rs2_data),
						.word_out(store_swapped)
					);
					wire [31:0] dload_ext;
					wire [3:0] byte_en;
					wire [3:0] byte_en_temp;
					reg [3:0] byte_en_standard;
					dmem_extender dmem_ext(
						.dmem_in(top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.rdata),
						.load_type(cu_if.load_type),
						.byte_en(byte_en),
						.ext_out(dload_ext)
					);
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.rdata_s_0 = rf_if.rs1_data;
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.rdata_s_1 = rf_if.rs2_data;
					localparam BUS_ENDIANNESS = "big";
					if (BUS_ENDIANNESS == "big") begin
						assign byte_en = byte_en_temp;
					end
					else if (BUS_ENDIANNESS == "little") assign byte_en = (cu_if.dren ? byte_en_temp : {byte_en_temp[0], byte_en_temp[1], byte_en_temp[2], byte_en_temp[3]});
					assign cu_if.instr = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[63-:32];
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.insn = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[63-:32];
					wire [31:0] imm_I_ext;
					wire [31:0] imm_S_ext;
					wire [31:0] imm_UJ_ext;
					assign imm_I_ext = {{20 {cu_if.imm_I[11]}}, cu_if.imm_I};
					assign imm_UJ_ext = {{11 {cu_if.imm_UJ[20]}}, cu_if.imm_UJ};
					assign imm_S_ext = {{20 {cu_if.imm_S[11]}}, cu_if.imm_S};
					reg [31:0] jump_addr;
					always @(*)
						if (cu_if.j_sel) begin
							jump_if.base = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32];
							jump_if.offset = imm_UJ_ext;
							jump_addr = jump_if.jal_addr;
						end
						else begin
							jump_if.base = rf_if.rs1_data;
							jump_if.offset = imm_I_ext;
							jump_addr = jump_if.jalr_addr;
						end
					wire [31:0] imm_or_shamt;
					assign imm_or_shamt = (cu_if.imm_shamt_sel == 1'b1 ? cu_if.shamt : imm_I_ext);
					assign alu_if.aluop = cu_if.alu_op;
					reg mal_addr;
					always @(*)
						case (cu_if.alu_a_sel)
							2'd0: alu_if.port_a = rf_if.rs1_data;
							2'd1: alu_if.port_a = imm_S_ext;
							2'd2: alu_if.port_a = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32];
							2'd3: alu_if.port_a = {32 {1'sb0}};
						endcase
					always @(*)
						case (cu_if.alu_b_sel)
							2'd0: alu_if.port_b = rf_if.rs1_data;
							2'd1: alu_if.port_b = rf_if.rs2_data;
							2'd2: alu_if.port_b = imm_or_shamt;
							2'd3: alu_if.port_b = cu_if.imm_U;
						endcase
					always @(*)
						if (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_w)
							rf_if.w_data = top_level_bASIC.corewrap.RISCVBusiness.rm_if.reg_wdata;
						else
							case (cu_if.w_sel)
								3'd0: rf_if.w_data = dload_ext;
								3'd1: rf_if.w_data = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[95-:32];
								3'd2: rf_if.w_data = cu_if.imm_U;
								3'd3: rf_if.w_data = alu_if.port_out;
								3'd4: rf_if.w_data = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.rdata;
								default: rf_if.w_data = {32 {1'sb0}};
							endcase
					assign rf_if.wen = ((cu_if.wen | (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_w & top_level_bASIC.corewrap.RISCVBusiness.rm_if.reg_w)) & (~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.npc_sel)) & ~(cu_if.dren & mal_addr);
					wire [31:0] resolved_addr;
					wire branch_taken;
					wire [31:0] branch_addr;
					assign branch_if.rs1_data = rf_if.rs1_data;
					assign branch_if.rs2_data = rf_if.rs2_data;
					assign branch_if.pc = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32];
					assign branch_if.imm_sb = cu_if.imm_SB;
					assign branch_if.branch_type = cu_if.branch_type;
					assign branch_taken = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_br_j ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.branch_jump : branch_if.branch_taken);
					assign branch_addr = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_br_j ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.br_j_addr : branch_if.branch_addr);
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.pc = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32];
					assign resolved_addr = (branch_if.branch_taken ? branch_addr : top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[95-:32]);
					assign top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.brj_addr = ((cu_if.ex_pc_sel == 1'b1) && ~top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_br_j ? jump_addr : resolved_addr);
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mispredict = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[31-:32] ^ branch_taken;
					wire [1:0] byte_offset;
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_load = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.rdata;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.ren = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_ren : cu_if.dren & ~mal_addr);
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_wen : cu_if.dwen & ~mal_addr);
					assign byte_en_temp = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_byte_en : byte_en_standard);
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.byte_en = byte_en;
					assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr = (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_addr : alu_if.port_out);
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.d_mem_busy = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.busy;
					assign byte_offset = alu_if.port_out[1:0];
					localparam rv32i_types_pkg_LD_W = 3;
					always @(*) begin
						top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata = {32 {1'sb0}};
						if (top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem)
							top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata = top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_store;
						else
							case (cu_if.load_type)
								rv32i_types_pkg_LB: top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata = {4 {rf_if.rs2_data[7:0]}};
								rv32i_types_pkg_LH: top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata = {2 {rf_if.rs2_data[15:0]}};
								rv32i_types_pkg_LW: top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata = rf_if.rs2_data;
							endcase
					end
					always @(*)
						case (cu_if.load_type)
							rv32i_types_pkg_LB:
								case (byte_offset)
									2'b00: byte_en_standard = 4'b0001;
									2'b01: byte_en_standard = 4'b0010;
									2'b10: byte_en_standard = 4'b0100;
									2'b11: byte_en_standard = 4'b1000;
									default: byte_en_standard = 4'b0000;
								endcase
							rv32i_types_pkg_LBU:
								case (byte_offset)
									2'b00: byte_en_standard = 4'b0001;
									2'b01: byte_en_standard = 4'b0010;
									2'b10: byte_en_standard = 4'b0100;
									2'b11: byte_en_standard = 4'b1000;
									default: byte_en_standard = 4'b0000;
								endcase
							rv32i_types_pkg_LH:
								case (byte_offset)
									2'b00: byte_en_standard = 4'b0011;
									2'b10: byte_en_standard = 4'b1100;
									default: byte_en_standard = 4'b0000;
								endcase
							rv32i_types_pkg_LHU:
								case (byte_offset)
									2'b00: byte_en_standard = 4'b0011;
									2'b10: byte_en_standard = 4'b1100;
									default: byte_en_standard = 4'b0000;
								endcase
							rv32i_types_pkg_LW: byte_en_standard = 4'b1111;
							default: byte_en_standard = 4'b0000;
						endcase
					reg ifence_reg;
					wire ifence_pulse;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							ifence_reg <= 1'b0;
						else
							ifence_reg <= cu_if.ifence;
					assign ifence_pulse = cu_if.ifence && ~ifence_reg;
					assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.icache_flush = ifence_pulse;
					assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.icache_clear = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dcache_flush = ifence_pulse;
					assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dcache_clear = 1'b0;
					reg dflushed;
					reg iflushed;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							iflushed <= 1'b1;
						else if (ifence_pulse)
							iflushed <= 1'b0;
						else if (top_level_bASIC.corewrap.RISCVBusiness.cc_if.iflush_done)
							iflushed <= 1'b1;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							dflushed <= 1'b1;
						else if (ifence_pulse)
							dflushed <= 1'b0;
						else if (top_level_bASIC.corewrap.RISCVBusiness.cc_if.dflush_done)
							dflushed <= 1'b1;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fence_stall = cu_if.ifence && (~dflushed || ~iflushed);
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.dren = cu_if.dren;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.dwen = cu_if.dwen;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.jump = cu_if.jump;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.branch = cu_if.branch;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.halt = halt;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							halt <= 1'b0;
						else if (cu_if.halt)
							halt <= cu_if.halt;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.swap = cu_if.csr_swap;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.clr = cu_if.csr_clr;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.set = cu_if.csr_set;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.wdata = (cu_if.csr_imm ? {27'h0000000, cu_if.zimm} : rf_if.rs1_data);
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.addr = cu_if.csr_addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.valid_write = ((top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.swap | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.clr) | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.set) & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.instr = cu_if.instr != {32 {1'sb0}};
					always @(*)
						if (byte_en == 4'hf)
							mal_addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr[1:0] != 2'b00;
						else if ((byte_en == 4'h3) || (byte_en == 4'hc))
							mal_addr = (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr[1:0] == 2'b01) || (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr[1:0] == 2'b11);
						else
							mal_addr = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.illegal_insn = (cu_if.illegal_insn & ~top_level_bASIC.corewrap.RISCVBusiness.rm_if.ex_token) | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.invalid_csr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_l = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_l = cu_if.dren & mal_addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_s = 1'b0;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_s = cu_if.dwen & mal_addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.breakpoint = cu_if.breakpoint;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.env_m = cu_if.ecall_insn;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.ret = cu_if.ret_insn;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.badaddr_e = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.epc_e = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[127-:32];
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.token_ex = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[128];
					assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.update_predictor = cu_if.branch;
					assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.prediction = top_level_bASIC.corewrap.RISCVBusiness.fetch_ex_if.fetch_ex_reg[31-:32];
					assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.branch_result = branch_if.branch_taken;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.wb_data = rf_if.w_data;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.wb_en = rf_if.wen;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_data = rf_if.rs2_data;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_addr = alu_if.port_out;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_wen = cu_if.dwen;
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.rd = rf_if.rd;
					wire wb_stall;
					wire [2:0] funct3;
					wire [11:0] funct12;
					wire instr_30;
					assign wb_stall = (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.jump) & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.branch;
					assign funct3 = cu_if.instr[14:12];
					assign funct12 = cu_if.instr[31:20];
					assign instr_30 = cu_if.instr[30];
				end
				assign execute_stage_i.CLK = CLK;
				assign execute_stage_i.nRST = nRST;
				assign halt = execute_stage_i.halt;
				if (1) begin : hazard_unit_i
					wire dmem_access;
					wire branch_jump;
					wire wait_for_imem;
					wire wait_for_dmem;
					wire ex_flush_hazard;
					wire e_ex_stage;
					wire e_f_stage;
					wire intr;
					wire rmgmt_stall;
					assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.if_ex_enable = ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall;
					assign rmgmt_stall = top_level_bASIC.corewrap.RISCVBusiness.rm_if.memory_stall | top_level_bASIC.corewrap.RISCVBusiness.rm_if.execute_stall;
					assign dmem_access = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.dren || top_level_bASIC.corewrap.RISCVBusiness.hazard_if.dwen;
					assign branch_jump = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.jump || (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.branch && top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mispredict);
					assign wait_for_imem = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.iren & top_level_bASIC.corewrap.RISCVBusiness.hazard_if.i_mem_busy;
					assign wait_for_dmem = dmem_access & top_level_bASIC.corewrap.RISCVBusiness.hazard_if.d_mem_busy;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.npc_sel = branch_jump;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.pc_en = (((((((~wait_for_dmem & ~wait_for_imem) & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.halt) & ~ex_flush_hazard) & ~rmgmt_stall) & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fence_stall) | branch_jump) | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.insert_pc) | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ret;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_flush = (ex_flush_hazard | branch_jump) | ((wait_for_imem & dmem_access) & ~top_level_bASIC.corewrap.RISCVBusiness.hazard_if.d_mem_busy);
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall = ((((wait_for_dmem || (wait_for_imem & ~dmem_access)) || top_level_bASIC.corewrap.RISCVBusiness.hazard_if.halt) & (~ex_flush_hazard | e_ex_stage)) || top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fence_stall) || top_level_bASIC.corewrap.RISCVBusiness.rm_if.execute_stall;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ret = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.ret;
					assign e_ex_stage = (((((top_level_bASIC.corewrap.RISCVBusiness.hazard_if.illegal_insn | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_l) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_l) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_s) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_s) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.breakpoint) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.env_m;
					assign e_f_stage = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_insn | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_insn;
					assign intr = (~e_ex_stage & ~e_f_stage) & top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.intr;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.pipe_clear = e_ex_stage | ~(top_level_bASIC.corewrap.RISCVBusiness.hazard_if.token_ex | top_level_bASIC.corewrap.RISCVBusiness.rm_if.active_insn);
					assign ex_flush_hazard = (((intr | e_f_stage) & ~wait_for_dmem) | e_ex_stage) | top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ret;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.insert_priv_pc = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.insert_pc;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.priv_pc = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.priv_pc;
					assign top_level_bASIC.corewrap.RISCVBusiness.hazard_if.iren = !intr;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.wb_enable = (!top_level_bASIC.corewrap.RISCVBusiness.hazard_if.if_ex_stall | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.jump) | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.branch;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_insn = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_insn;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_insn = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_insn;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.illegal_insn = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.illegal_insn;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_l = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_l;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_l = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_l;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_s = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_s;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_s = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_s;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.breakpoint = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.breakpoint;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.env_m = top_level_bASIC.corewrap.RISCVBusiness.hazard_if.env_m;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ex_rmgmt = top_level_bASIC.corewrap.RISCVBusiness.rm_if.exception;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ex_rmgmt_cause = top_level_bASIC.corewrap.RISCVBusiness.rm_if.ex_cause;
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.epc = (e_ex_stage | top_level_bASIC.corewrap.RISCVBusiness.rm_if.exception ? top_level_bASIC.corewrap.RISCVBusiness.hazard_if.epc_e : top_level_bASIC.corewrap.RISCVBusiness.hazard_if.epc_f);
					assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.badaddr = (top_level_bASIC.corewrap.RISCVBusiness.hazard_if.mal_insn | top_level_bASIC.corewrap.RISCVBusiness.hazard_if.fault_insn ? top_level_bASIC.corewrap.RISCVBusiness.hazard_if.badaddr_f : (top_level_bASIC.corewrap.RISCVBusiness.rm_if.exception ? top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_addr : top_level_bASIC.corewrap.RISCVBusiness.hazard_if.badaddr_e));
					assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.if_ex_enable = top_level_bASIC.corewrap.RISCVBusiness.rm_if.if_ex_enable;
				end
				if (1) begin : branch_predictor_i
					wire CLK;
					wire nRST;
					localparam BR_PREDICTOR_TYPE = "not_taken";
					case (BR_PREDICTOR_TYPE)
						"not_taken": begin
							if (1) begin : predictor
								wire CLK;
								wire nRST;
								assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.predict_taken = 0;
								assign top_level_bASIC.corewrap.RISCVBusiness.predict_if.target_addr = top_level_bASIC.corewrap.RISCVBusiness.predict_if.current_pc + 4;
							end
							assign predictor.CLK = CLK;
							assign predictor.nRST = nRST;
						end
					endcase
				end
				assign branch_predictor_i.CLK = CLK;
				assign branch_predictor_i.nRST = nRST;
				if (1) begin : priv_wrapper_i
					wire CLK;
					wire nRST;
					if (1) begin : priv_block_i
						wire CLK;
						wire nRST;
						if (1) begin : prv_intern_if
							wire mip_rup;
							wire mtval_rup;
							wire mcause_rup;
							wire mepc_rup;
							wire mstatus_rup;
							wire intr;
							wire pipe_clear;
							wire mret;
							wire sret;
							wire uret;
							wire timer_int_u;
							wire timer_int_s;
							wire timer_int_m;
							wire soft_int_u;
							wire soft_int_s;
							wire soft_int_m;
							wire ext_int_u;
							wire ext_int_s;
							wire ext_int_m;
							wire reserved_0;
							wire reserved_1;
							wire reserved_2;
							wire clear_timer_int_u;
							wire clear_timer_int_s;
							wire clear_timer_int_m;
							wire clear_soft_int_u;
							wire clear_soft_int_s;
							wire clear_soft_int_m;
							wire clear_ext_int_u;
							wire clear_ext_int_s;
							wire clear_ext_int_m;
							wire mal_insn;
							wire fault_insn_access;
							wire illegal_insn;
							wire breakpoint;
							wire fault_l;
							wire mal_l;
							wire fault_s;
							wire mal_s;
							wire env_u;
							wire env_s;
							wire env_m;
							wire fault_insn_page;
							wire fault_load_page;
							wire fault_store_page;
							wire insert_pc;
							wire swap;
							wire clr;
							wire set;
							wire valid_write;
							wire invalid_csr;
							wire instr_retired;
							wire ex_rmgmt;
							wire [-1:0] ex_rmgmt_cause;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] epc;
							reg [31:0] priv_pc;
							wire [31:0] wdata;
							reg [31:0] rdata;
							wire [31:0] mtval;
							wire [31:0] mip;
							reg [31:0] mip_next;
							wire [31:0] mtval_next;
							wire [31:0] mcause;
							wire [31:0] mcause_next;
							wire [31:0] mepc;
							wire [31:0] mepc_next;
							wire [31:0] mstatus;
							reg [31:0] mstatus_next;
							wire [31:0] mtvec;
							wire [31:0] mie;
							wire [11:0] addr;
						end
						if (1) begin : csr_rfile_i
							wire CLK;
							wire nRST;
							wire [31:0] mvendorid;
							wire [31:0] marchid;
							wire [31:0] mimpid;
							wire [31:0] mhartid;
							reg [31:0] misaid;
							reg [31:0] misaid_next;
							reg [31:0] misaid_temp;
							wire [31:0] misaid_default;
							assign misaid_default[31-:2] = machine_mode_types_1_11_pkg_BASE_RV32;
							assign misaid_default[29-:4] = {4 {1'sb0}};
							localparam machine_mode_types_1_11_pkg_MISAID_EXT_I = 26'h0000001 << 8;
							localparam machine_mode_types_1_11_pkg_MISAID_EXT_M = 26'h0000001 << 12;
							assign misaid_default[25-:26] = machine_mode_types_1_11_pkg_MISAID_EXT_I | machine_mode_types_1_11_pkg_MISAID_EXT_M;
							assign mvendorid = {32 {1'sb0}};
							assign marchid = {32 {1'sb0}};
							assign mimpid = {32 {1'sb0}};
							assign mhartid = {32 {1'sb0}};
							reg [31:0] mstatus;
							wire [31:0] mstatus_next;
							wire [31:0] medeleg;
							wire [31:0] mideleg;
							reg [31:0] mie;
							wire [31:0] mie_next;
							reg [31:0] mtvec;
							wire [31:0] mtvec_next;
							wire [1:1] sv2v_tmp_7FE57;
							assign sv2v_tmp_7FE57 = 1'b0;
							always @(*) mstatus[0] = sv2v_tmp_7FE57;
							wire [1:1] sv2v_tmp_6DA44;
							assign sv2v_tmp_6DA44 = 1'b0;
							always @(*) mstatus[1] = sv2v_tmp_6DA44;
							wire [1:1] sv2v_tmp_7E375;
							assign sv2v_tmp_7E375 = 1'b0;
							always @(*) mstatus[2] = sv2v_tmp_7E375;
							wire [1:1] sv2v_tmp_F2078;
							assign sv2v_tmp_F2078 = 1'b0;
							always @(*) mstatus[4] = sv2v_tmp_F2078;
							wire [1:1] sv2v_tmp_F02E0;
							assign sv2v_tmp_F02E0 = 1'b0;
							always @(*) mstatus[5] = sv2v_tmp_F02E0;
							wire [1:1] sv2v_tmp_DEF9C;
							assign sv2v_tmp_DEF9C = 1'b0;
							always @(*) mstatus[6] = sv2v_tmp_DEF9C;
							wire [1:1] sv2v_tmp_B3F31;
							assign sv2v_tmp_B3F31 = 1'b0;
							always @(*) mstatus[8] = sv2v_tmp_B3F31;
							wire [2:1] sv2v_tmp_3914A;
							assign sv2v_tmp_3914A = 2'b00;
							always @(*) mstatus[10-:2] = sv2v_tmp_3914A;
							wire [2:1] sv2v_tmp_26161;
							assign sv2v_tmp_26161 = machine_mode_types_1_11_pkg_M_LEVEL;
							always @(*) mstatus[12-:2] = sv2v_tmp_26161;
							wire [1:1] sv2v_tmp_EED06;
							assign sv2v_tmp_EED06 = 1'b0;
							always @(*) mstatus[17] = sv2v_tmp_EED06;
							wire [1:1] sv2v_tmp_5AC50;
							assign sv2v_tmp_5AC50 = 1'b0;
							always @(*) mstatus[18] = sv2v_tmp_5AC50;
							wire [1:1] sv2v_tmp_47705;
							assign sv2v_tmp_47705 = 1'b0;
							always @(*) mstatus[19] = sv2v_tmp_47705;
							wire [1:1] sv2v_tmp_C63A7;
							assign sv2v_tmp_C63A7 = 1'b0;
							always @(*) mstatus[20] = sv2v_tmp_C63A7;
							wire [1:1] sv2v_tmp_FCE3C;
							assign sv2v_tmp_FCE3C = 1'b0;
							always @(*) mstatus[21] = sv2v_tmp_FCE3C;
							wire [1:1] sv2v_tmp_638DC;
							assign sv2v_tmp_638DC = 1'b0;
							always @(*) mstatus[22] = sv2v_tmp_638DC;
							wire [2:1] sv2v_tmp_D5581;
							assign sv2v_tmp_D5581 = machine_mode_types_1_11_pkg_XS_ALL_OFF;
							always @(*) mstatus[16-:2] = sv2v_tmp_D5581;
							wire [2:1] sv2v_tmp_460A2;
							assign sv2v_tmp_460A2 = machine_mode_types_1_11_pkg_FS_OFF;
							always @(*) mstatus[14-:2] = sv2v_tmp_460A2;
							wire [1:1] sv2v_tmp_55859;
							assign sv2v_tmp_55859 = (mstatus[14-:2] == machine_mode_types_1_11_pkg_FS_DIRTY) | (mstatus[16-:2] == machine_mode_types_1_11_pkg_XS_SOME_D);
							always @(*) mstatus[31] = sv2v_tmp_55859;
							wire [8:1] sv2v_tmp_97732;
							assign sv2v_tmp_97732 = {8 {1'sb0}};
							always @(*) mstatus[30-:8] = sv2v_tmp_97732;
							assign medeleg = {32 {1'sb0}};
							assign mideleg = {32 {1'sb0}};
							wire [1:1] sv2v_tmp_C7ED9;
							assign sv2v_tmp_C7ED9 = 1'b0;
							always @(*) mie[2] = sv2v_tmp_C7ED9;
							wire [1:1] sv2v_tmp_E75F8;
							assign sv2v_tmp_E75F8 = 1'b0;
							always @(*) mie[6] = sv2v_tmp_E75F8;
							wire [1:1] sv2v_tmp_F1243;
							assign sv2v_tmp_F1243 = 1'b0;
							always @(*) mie[10] = sv2v_tmp_F1243;
							wire [20:1] sv2v_tmp_492EC;
							assign sv2v_tmp_492EC = {20 {1'sb0}};
							always @(*) mie[31-:20] = sv2v_tmp_492EC;
							wire [1:1] sv2v_tmp_E80A1;
							assign sv2v_tmp_E80A1 = 1'b0;
							always @(*) mie[4] = sv2v_tmp_E80A1;
							wire [1:1] sv2v_tmp_F91D8;
							assign sv2v_tmp_F91D8 = 1'b0;
							always @(*) mie[5] = sv2v_tmp_F91D8;
							wire [1:1] sv2v_tmp_5A2C2;
							assign sv2v_tmp_5A2C2 = 1'b0;
							always @(*) mie[0] = sv2v_tmp_5A2C2;
							wire [1:1] sv2v_tmp_9F4B3;
							assign sv2v_tmp_9F4B3 = 1'b0;
							always @(*) mie[1] = sv2v_tmp_9F4B3;
							wire [1:1] sv2v_tmp_A4D0F;
							assign sv2v_tmp_A4D0F = 1'b0;
							always @(*) mie[8] = sv2v_tmp_A4D0F;
							wire [1:1] sv2v_tmp_84425;
							assign sv2v_tmp_84425 = 1'b0;
							always @(*) mie[9] = sv2v_tmp_84425;
							reg [31:0] mscratch;
							wire [31:0] mscratch_next;
							reg [31:0] mepc;
							wire [31:0] mepc_next;
							reg [31:0] mcause;
							wire [31:0] mcause_next;
							reg [31:0] mtval;
							wire [31:0] mtval_next;
							reg [31:0] mip;
							wire [31:0] mip_next;
							wire [1:1] sv2v_tmp_D4FFD;
							assign sv2v_tmp_D4FFD = 1'b0;
							always @(*) mip[2] = sv2v_tmp_D4FFD;
							wire [1:1] sv2v_tmp_6AE0B;
							assign sv2v_tmp_6AE0B = 1'b0;
							always @(*) mip[6] = sv2v_tmp_6AE0B;
							wire [1:1] sv2v_tmp_BDFCE;
							assign sv2v_tmp_BDFCE = 1'b0;
							always @(*) mip[10] = sv2v_tmp_BDFCE;
							wire [20:1] sv2v_tmp_5CFC5;
							assign sv2v_tmp_5CFC5 = {20 {1'sb0}};
							always @(*) mip[31-:20] = sv2v_tmp_5CFC5;
							wire [1:1] sv2v_tmp_FE172;
							assign sv2v_tmp_FE172 = 1'b0;
							always @(*) mip[4] = sv2v_tmp_FE172;
							wire [1:1] sv2v_tmp_8421E;
							assign sv2v_tmp_8421E = 1'b0;
							always @(*) mip[5] = sv2v_tmp_8421E;
							wire [1:1] sv2v_tmp_2E207;
							assign sv2v_tmp_2E207 = 1'b0;
							always @(*) mip[0] = sv2v_tmp_2E207;
							wire [1:1] sv2v_tmp_95EC8;
							assign sv2v_tmp_95EC8 = 1'b0;
							always @(*) mip[1] = sv2v_tmp_95EC8;
							wire [1:1] sv2v_tmp_72F26;
							assign sv2v_tmp_72F26 = 1'b0;
							always @(*) mip[8] = sv2v_tmp_72F26;
							wire [1:1] sv2v_tmp_61DEE;
							assign sv2v_tmp_61DEE = 1'b0;
							always @(*) mip[9] = sv2v_tmp_61DEE;
							wire [31:0] cycle;
							wire [31:0] cycleh;
							wire [31:0] cycle_next;
							wire [31:0] cycleh_next;
							wire [31:0] _time;
							wire [31:0] timeh;
							wire [31:0] time_next;
							wire [31:0] timeh_next;
							wire [31:0] instret;
							wire [31:0] instreth;
							wire [31:0] instret_next;
							wire [31:0] isntreth_next;
							reg [63:0] instretfull;
							wire [63:0] instretfull_next;
							reg [63:0] cyclefull;
							wire [63:0] cyclefull_next;
							reg [63:0] timefull;
							wire [63:0] timefull_next;
							assign _time = timefull[31:0];
							assign timeh = timefull[63:32];
							assign timefull_next = timefull + 1;
							assign cycle = cyclefull[31:0];
							assign cycleh = cyclefull[63:32];
							assign cyclefull_next = cyclefull + 1;
							assign instret = instretfull[31:0];
							assign instreth = instretfull[63:32];
							assign instretfull_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.instr_retired == 1'b1 ? instretfull + 1 : instretfull);
							always @(posedge CLK or negedge nRST)
								if (~nRST) begin
									mstatus[3] <= 1'b0;
									mstatus[7] <= 1'b0;
									mie[7] <= 1'b0;
									mie[3] <= 1'b0;
									mip[3] <= 1'b0;
									mip[7] <= 1'b0;
									mie[11] <= 1'b0;
									mip[11] <= 1'b0;
									misaid <= misaid_default;
									mtvec <= {32 {1'sb0}};
									mcause <= {32 {1'sb0}};
									mepc <= {32 {1'sb0}};
									mtval <= {32 {1'sb0}};
									timefull <= {64 {1'sb0}};
									cyclefull <= {64 {1'sb0}};
									instretfull <= {64 {1'sb0}};
								end
								else begin
									mstatus[3] <= mstatus_next[3];
									mstatus[7] <= mstatus_next[7];
									mie[7] <= mie_next[7];
									mie[3] <= mie_next[3];
									mie[11] <= mie_next[11];
									mip[3] <= mip_next[3];
									mip[7] <= mip_next[7];
									mip[11] <= mip_next[11];
									misaid <= misaid_next;
									mtvec <= mtvec_next;
									mcause <= mcause_next;
									mepc <= mepc_next;
									mtval <= mtval_next;
									mscratch <= mscratch_next;
									timefull <= timefull_next;
									cyclefull <= cyclefull_next;
									instretfull <= instretfull_next;
								end
							reg valid_csr_addr;
							wire csr_op;
							wire swap;
							wire clr;
							wire set;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] rup_data;
							assign csr_op = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.swap | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clr) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.set;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.invalid_csr = csr_op & ~valid_csr_addr;
							assign swap = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.swap & valid_csr_addr) & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.valid_write;
							assign clr = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clr & valid_csr_addr) & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.valid_write;
							assign set = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.set & valid_csr_addr) & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.valid_write;
							assign rup_data = (swap ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.wdata : (clr ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata & ~top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.wdata : (set ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.wdata : top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata)));
							assign mip_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_rup ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next : mip);
							assign mtval_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtval_rup ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtval_next : mtval);
							assign mcause_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause_rup ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause_next : mcause);
							assign mstatus_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MSTATUS_ADDR ? rup_data : (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_rup ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next : mstatus));
							assign mepc_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MEPC_ADDR ? rup_data : (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc_rup ? top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc_next : mepc));
							assign mie_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MIE_ADDR ? rup_data : mie);
							assign mtvec_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MTVEC_ADDR ? rup_data : mtvec);
							assign mscratch_next = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MSCRATCH_ADDR ? rup_data : mscratch);
							localparam machine_mode_types_1_11_pkg_MISAID_EXT_E = 26'h0000001 << 4;
							always @(*) begin
								misaid_temp = rup_data;
								if ((((top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr == machine_mode_types_1_11_pkg_MISA_ADDR) && (misaid_temp[31-:2] != 2'b00)) && ((misaid_temp[25-:26] & machine_mode_types_1_11_pkg_MISAID_EXT_E) ^ ((misaid_temp[25-:26] & machine_mode_types_1_11_pkg_MISAID_EXT_I) != 'b1))) && (misaid_temp[29-:4] == 4'b0000))
									misaid_next = misaid_temp;
								else
									misaid_next = misaid;
							end
							always @(*) begin
								valid_csr_addr = 1'b1;
								casez (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.addr)
									machine_mode_types_1_11_pkg_MVENDORID_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mvendorid;
									machine_mode_types_1_11_pkg_MARCHID_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = marchid;
									machine_mode_types_1_11_pkg_MIMPID_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mimpid;
									machine_mode_types_1_11_pkg_MHARTID_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mhartid;
									machine_mode_types_1_11_pkg_MISA_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = misaid;
									machine_mode_types_1_11_pkg_MSTATUS_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mstatus;
									machine_mode_types_1_11_pkg_MTVEC_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mtvec;
									machine_mode_types_1_11_pkg_MEDELEG_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = medeleg;
									machine_mode_types_1_11_pkg_MIDELEG_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mideleg;
									machine_mode_types_1_11_pkg_MIE_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mie;
									machine_mode_types_1_11_pkg_MSCRATCH_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mscratch;
									machine_mode_types_1_11_pkg_MEPC_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mepc;
									machine_mode_types_1_11_pkg_MCAUSE_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mcause;
									machine_mode_types_1_11_pkg_MTVAL_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mtval;
									machine_mode_types_1_11_pkg_MIP_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = mip;
									machine_mode_types_1_11_pkg_MCYCLE_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = cycle;
									machine_mode_types_1_11_pkg_MINSTRET_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = instret;
									machine_mode_types_1_11_pkg_MCYCLEH_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = cycleh;
									machine_mode_types_1_11_pkg_MINSTRETH_ADDR: top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = instreth;
									default: begin
										valid_csr_addr = 1'b0;
										top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.rdata = {32 {1'sb0}};
									end
								endcase
							end
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtvec = mtvec;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc = mepc;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mie = mie;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus = mstatus;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause = mcause;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip = mip;
						end
						assign csr_rfile_i.CLK = CLK;
						assign csr_rfile_i.nRST = nRST;
						if (1) begin : prv_control_i
							wire CLK;
							wire nRST;
							reg [30:0] ex_src;
							reg exception;
							reg [30:0] intr_src;
							reg interrupt;
							wire clear_interrupt;
							reg interrupt_reg;
							wire interrupt_fired;
							reg update_mie;
							always @(*) begin
								interrupt = 1'b1;
								intr_src = machine_mode_types_1_11_pkg_SOFT_INT_M;
								if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_m)
									intr_src = machine_mode_types_1_11_pkg_EXT_INT_M;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_m)
									intr_src = machine_mode_types_1_11_pkg_SOFT_INT_M;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_m)
									intr_src = machine_mode_types_1_11_pkg_TIMER_INT_M;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_s)
									intr_src = machine_mode_types_1_11_pkg_EXT_INT_S;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_s)
									intr_src = machine_mode_types_1_11_pkg_SOFT_INT_S;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_s)
									intr_src = machine_mode_types_1_11_pkg_TIMER_INT_S;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_u)
									intr_src = machine_mode_types_1_11_pkg_EXT_INT_U;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_u)
									intr_src = machine_mode_types_1_11_pkg_SOFT_INT_U;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_u)
									intr_src = machine_mode_types_1_11_pkg_TIMER_INT_U;
								else
									interrupt = 1'b0;
							end
							assign clear_interrupt = (((((((top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_m || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_m) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_m) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_u) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_u) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_u) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_s) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_s) || top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_s;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_rup = interrupt || clear_interrupt;
							always @(*) begin
								top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip;
								if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[11] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[11] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[3] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[3] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[7] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_m)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[7] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[9] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[9] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[1] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[1] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[5] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_s)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[5] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ext_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[8] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_ext_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[8] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.soft_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[0] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_soft_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[0] = 1'b0;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.timer_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[4] = 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.clear_timer_int_u)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip_next[4] = 1'b0;
							end
							function automatic [30:0] sv2v_cast_31;
								input reg [30:0] inp;
								sv2v_cast_31 = inp;
							endfunction
							always @(*) begin
								exception = 1'b1;
								ex_src = machine_mode_types_1_11_pkg_INSN_MAL;
								if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.breakpoint)
									ex_src = machine_mode_types_1_11_pkg_BREAKPOINT;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_insn_page)
									ex_src = machine_mode_types_1_11_pkg_INSN_PAGE;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_insn_access)
									ex_src = machine_mode_types_1_11_pkg_INSN_ACCESS;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.illegal_insn)
									ex_src = machine_mode_types_1_11_pkg_ILLEGAL_INSN;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_insn)
									ex_src = machine_mode_types_1_11_pkg_INSN_MAL;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.env_u)
									ex_src = machine_mode_types_1_11_pkg_ENV_CALL_U;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.env_s)
									ex_src = machine_mode_types_1_11_pkg_ENV_CALL_S;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.env_m)
									ex_src = machine_mode_types_1_11_pkg_ENV_CALL_M;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_s)
									ex_src = machine_mode_types_1_11_pkg_S_ADDR_MAL;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_l)
									ex_src = machine_mode_types_1_11_pkg_L_ADDR_MAL;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_store_page)
									ex_src = machine_mode_types_1_11_pkg_STORE_PAGE;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_load_page)
									ex_src = machine_mode_types_1_11_pkg_LOAD_PAGE;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_s)
									ex_src = machine_mode_types_1_11_pkg_S_FAULT;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_l)
									ex_src = machine_mode_types_1_11_pkg_L_FAULT;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ex_rmgmt)
									ex_src = sv2v_cast_31(top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ex_rmgmt_cause);
								else
									exception = 1'b0;
							end
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.intr = exception | interrupt_reg;
							assign interrupt_fired = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus[3] & (((top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mie[7] & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip[7]) | (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mie[3] & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip[3])) | (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mie[11] & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mip[11]));
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause_rup = exception | interrupt;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause_next[31] = ~exception;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause_next[30-:31] = (exception ? ex_src : intr_src);
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_rup = (exception | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.intr) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mret;
							always @(*) begin
								top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[3] = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus[3];
								top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[7] = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus[7];
								if (update_mie) begin
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[7] = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus[3];
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[3] = 1'b0;
								end
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mret) begin
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[7] = 1'b1;
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus_next[3] = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mstatus[7];
								end
							end
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc_rup = exception | (interrupt_fired & ~update_mie);
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc_next = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.epc;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtval_rup = (((((((top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_l | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_l) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_s) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_s) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.illegal_insn) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.fault_insn_access) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mal_insn) | top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.ex_rmgmt) & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.pipe_clear;
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtval_next = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtval;
							always @(posedge CLK or negedge nRST)
								if (!nRST)
									interrupt_reg <= 1'b0;
								else if (interrupt_fired)
									interrupt_reg <= 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.pipe_clear)
									interrupt_reg <= 1'b0;
							always @(posedge CLK or negedge nRST)
								if (!nRST)
									update_mie <= 1'b0;
								else if (interrupt_fired && ~update_mie)
									update_mie <= 1'b1;
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.pipe_clear)
									update_mie <= 1'b0;
								else
									update_mie <= 1'b0;
						end
						assign prv_control_i.CLK = CLK;
						assign prv_control_i.nRST = nRST;
						if (1) begin : pipeline_control_i
							assign top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.insert_pc = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mret | (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.pipe_clear & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.intr);
							always @(*) begin
								top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.priv_pc = {32 {1'sbz}};
								if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.intr) begin
									if ((top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtvec[1-:2] == machine_mode_types_1_11_pkg_VECTORED) & top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause[31])
										top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.priv_pc = (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtvec[31-:30] << 2) + (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mcause[30-:31] << 2);
									else
										top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.priv_pc = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mtvec[31-:30] << 2;
								end
								else if (top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mret)
									top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.priv_pc = top_level_bASIC.corewrap.RISCVBusiness.priv_wrapper_i.priv_block_i.prv_intern_if.mepc;
							end
						end
						assign prv_intern_if.timer_int_u = 1'b0;
						assign prv_intern_if.timer_int_s = 1'b0;
						assign prv_intern_if.timer_int_m = top_level_bASIC.interrupt_if.timer_int;
						assign prv_intern_if.soft_int_u = 1'b0;
						assign prv_intern_if.soft_int_s = 1'b0;
						assign prv_intern_if.soft_int_m = top_level_bASIC.interrupt_if.soft_int;
						assign prv_intern_if.ext_int_u = 1'b0;
						assign prv_intern_if.ext_int_s = 1'b0;
						assign prv_intern_if.ext_int_m = top_level_bASIC.interrupt_if.ext_int;
						assign prv_intern_if.reserved_0 = 1'b0;
						assign prv_intern_if.reserved_1 = 1'b0;
						assign prv_intern_if.reserved_2 = 1'b0;
						assign prv_intern_if.clear_timer_int_u = 1'b0;
						assign prv_intern_if.clear_timer_int_s = 1'b0;
						assign prv_intern_if.clear_timer_int_m = top_level_bASIC.interrupt_if.timer_int_clear;
						assign prv_intern_if.clear_soft_int_u = 1'b0;
						assign prv_intern_if.clear_soft_int_s = 1'b0;
						assign prv_intern_if.clear_soft_int_m = top_level_bASIC.interrupt_if.soft_int_clear;
						assign prv_intern_if.clear_ext_int_u = 1'b0;
						assign prv_intern_if.clear_ext_int_s = 1'b0;
						assign prv_intern_if.clear_ext_int_m = top_level_bASIC.interrupt_if.ext_int_clear;
						assign prv_intern_if.pipe_clear = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.pipe_clear;
						assign prv_intern_if.mret = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ret;
						assign prv_intern_if.epc = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.epc;
						assign prv_intern_if.fault_insn_access = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_insn;
						assign prv_intern_if.mal_insn = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_insn;
						assign prv_intern_if.illegal_insn = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.illegal_insn;
						assign prv_intern_if.fault_l = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_l;
						assign prv_intern_if.mal_l = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_l;
						assign prv_intern_if.fault_s = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.fault_s;
						assign prv_intern_if.mal_s = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.mal_s;
						assign prv_intern_if.breakpoint = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.breakpoint;
						assign prv_intern_if.env_m = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.env_m;
						assign prv_intern_if.env_s = 1'b0;
						assign prv_intern_if.env_u = 1'b0;
						assign prv_intern_if.fault_insn_page = 1'b0;
						assign prv_intern_if.fault_load_page = 1'b0;
						assign prv_intern_if.fault_store_page = 1'b0;
						assign prv_intern_if.mtval = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.badaddr;
						assign prv_intern_if.swap = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.swap;
						assign prv_intern_if.clr = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.clr;
						assign prv_intern_if.set = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.set;
						assign prv_intern_if.wdata = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.wdata;
						assign prv_intern_if.addr = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.addr;
						assign prv_intern_if.valid_write = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.valid_write;
						assign prv_intern_if.instr_retired = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.wb_enable & top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.instr;
						assign prv_intern_if.ex_rmgmt = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ex_rmgmt;
						assign prv_intern_if.ex_rmgmt_cause = top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.ex_rmgmt_cause;
						assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.priv_pc = prv_intern_if.priv_pc;
						assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.insert_pc = prv_intern_if.insert_pc;
						assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.intr = prv_intern_if.intr;
						assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.rdata = prv_intern_if.rdata;
						assign top_level_bASIC.corewrap.RISCVBusiness.prv_pipe_if.invalid_csr = prv_intern_if.invalid_csr;
					end
					assign priv_block_i.CLK = CLK;
					assign priv_block_i.nRST = nRST;
				end
				assign priv_wrapper_i.CLK = CLK;
				assign priv_wrapper_i.nRST = nRST;
				if (1) begin : rmgmt
					wire CLK;
					wire nRST;
					if (1) begin : risc_mgmt_i
						wire CLK;
						wire nRST;
						localparam N_EXTENSIONS = 1;
						localparam N_EXT_BITS = 0;
						localparam rv32i_types_pkg_WORD_SIZE = 32;
						wire [31:0] d_insn;
						wire [0:0] d_insn_claim;
						wire [0:0] d_mem_to_reg;
						wire [4:0] d_rsel_s_0;
						wire [4:0] d_rsel_s_1;
						wire [4:0] d_rsel_d;
						wire [0:0] e_start;
						wire [0:0] e_exception;
						wire [0:0] e_busy;
						wire [31:0] e_rdata_s_0;
						wire [31:0] e_rdata_s_1;
						wire [0:0] e_branch_jump;
						wire [31:0] e_pc;
						wire [31:0] e_br_j_addr;
						wire [31:0] e_reg_wdata;
						wire [0:0] e_reg_w;
						wire [0:0] m_exception;
						wire [0:0] m_busy;
						wire [31:0] m_mem_addr;
						wire [0:0] m_mem_ren;
						wire [0:0] m_mem_wen;
						wire [0:0] m_mem_busy;
						wire [3:0] m_mem_byte_en;
						wire [31:0] m_mem_load;
						wire [31:0] m_mem_store;
						wire [0:0] m_reg_wdata;
						wire [0:0] m_reg_w;
						wire [7:0] rv32m_idex;
						wire [0:0] rv32m_exmem;
						if (1) begin : rv32m_idif
							wire insn_claim;
							wire mem_to_reg;
							wire [4:0] rsel_s_0;
							wire [4:0] rsel_s_1;
							wire [4:0] rsel_d;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] insn;
						end
						if (1) begin : rv32m_exif
							wire start;
							wire exception;
							reg busy;
							wire reg_w;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] rdata_s_0;
							wire [31:0] rdata_s_1;
							reg [31:0] reg_wdata;
							wire branch_jump;
							wire [31:0] br_j_addr;
							wire [31:0] pc;
						end
						if (1) begin : rv32m_memif
							wire exception;
							wire busy;
							wire reg_w;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] reg_wdata;
							wire mem_ren;
							wire mem_wen;
							wire mem_busy;
							wire [3:0] mem_byte_en;
							wire [31:0] mem_addr;
							wire [31:0] mem_load;
							wire [31:0] mem_store;
						end
						if (1) begin : rv32m_decode_t
							wire CLK;
							wire nRST;
							wire [7:0] idex;
							localparam rv32m_pkg_RV32M_OPCODE = 7'b0110011;
							localparam OPCODE = 7'b0110011;
							wire [31:0] insn;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.mem_to_reg = 1'b0;
							assign insn = top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.insn;
							localparam rv32m_pkg_RV32M_OPCODE_MINOR = 7'b0000001;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.insn_claim = (insn[6-:7] == OPCODE) && (insn[31-:7] == rv32m_pkg_RV32M_OPCODE_MINOR);
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.rsel_s_0 = insn[19-:5];
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.rsel_s_1 = insn[24-:5];
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.rsel_d = insn[11-:5];
							assign idex[0] = top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_idif.insn_claim;
							assign idex[7] = ~insn[14];
							assign idex[6] = insn[14:13] == 2'b10;
							assign idex[5] = insn[14:13] == 2'b11;
							assign idex[4] = ((insn[14-:3] == 3'b011) || (insn[14-:3] == 3'b101)) || (insn[14-:3] == 3'b111);
							assign idex[3] = (((insn[14-:3] == 3'b001) || (insn[14-:3] == 3'b100)) || (insn[14-:3] == 3'b110)) || (insn[14-:3] == 3'b000);
							assign idex[2] = insn[14-:3] == 3'b010;
							assign idex[1] = ~(|insn[13:12]);
						end
						assign rv32m_decode_t.CLK = CLK;
						assign rv32m_decode_t.nRST = nRST;
						assign rv32m_idex = rv32m_decode_t.idex;
						if (1) begin : rv32m_execute_t
							wire CLK;
							wire nRST;
							wire [7:0] idex;
							wire [0:0] exmem;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.exception = 1'b0;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_w = 1'b1;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.branch_jump = 1'b0;
							localparam rv32i_types_pkg_WORD_SIZE = 32;
							wire [31:0] op_a;
							wire [31:0] op_b;
							reg [31:0] op_a_save;
							reg [31:0] op_b_save;
							wire [2:0] operation;
							reg [2:0] operation_save;
							reg [1:0] is_signed_save;
							wire [1:0] is_signed_curr;
							wire [1:0] is_signed;
							wire operand_diff;
							assign op_a = (operand_diff ? top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_0 : op_a_save);
							assign op_b = (operand_diff ? top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_1 : op_b_save);
							assign operand_diff = ((((op_a_save != top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_0) || (op_b_save != top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_1)) || (is_signed_save != is_signed_curr)) || (operation_save != {idex[7], idex[6], idex[5]})) && idex[0];
							assign is_signed_curr = (idex[4] ? 2'b00 : (idex[3] ? 2'b11 : 2'b10));
							assign is_signed = (operand_diff ? is_signed_curr : is_signed_save);
							assign operation = (operand_diff ? {idex[7], idex[6], idex[5]} : operation_save);
							always @(posedge CLK or negedge nRST)
								if (~nRST) begin
									op_a_save <= {32 {1'sb0}};
									op_b_save <= {32 {1'sb0}};
									is_signed_save <= {2 {1'sb0}};
									operation_save <= {3 {1'sb0}};
								end
								else if (operand_diff) begin
									op_a_save <= top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_0;
									op_b_save <= top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.rdata_s_1;
									is_signed_save <= is_signed_curr;
									operation_save <= {idex[7], idex[6], idex[5]};
								end
							wire [31:0] multiplicand;
							wire [31:0] multiplier;
							wire [63:0] product;
							wire mul_finished;
							wire mul_start;
							assign multiplicand = op_a;
							assign multiplier = op_b;
							assign mul_start = operand_diff && operation[2];
							pp_mul32 mult_i(
								.CLK(CLK),
								.nRST(nRST),
								.multiplicand(multiplicand),
								.multiplier(multiplier),
								.product(product),
								.is_signed(is_signed),
								.start(mul_start),
								.finished(mul_finished)
							);
							wire overflow;
							wire div_zero;
							wire div_finished;
							wire [31:0] divisor;
							wire [31:0] dividend;
							wire [31:0] quotient;
							wire [31:0] remainder;
							wire [31:0] divisor_save;
							wire [31:0] dividend_save;
							wire div_operand_diff;
							wire div_start;
							assign divisor = op_b;
							assign dividend = op_a;
							assign overflow = ((dividend == 32'h80000000) && (divisor == 32'hffffffff)) && idex[3];
							assign div_zero = divisor == 32'h00000000;
							assign div_start = operand_diff && ((~operation[2] & ~overflow) & ~div_zero);
							radix4_divider div_i(
								.CLK(CLK),
								.nRST(nRST),
								.divisor(divisor),
								.dividend(dividend),
								.is_signed(idex[3]),
								.start(div_start),
								.remainder(remainder),
								.quotient(quotient),
								.finished(div_finished)
							);
							always @(*)
								casez (operation)
									3'b1zz: begin
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.busy = ~mul_finished;
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = (idex[1] ? product[31:0] : product[63:rv32i_types_pkg_WORD_SIZE]);
									end
									3'b01z: begin
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.busy = ~div_finished & ~(div_zero | overflow);
										if (div_zero)
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = (idex[3] ? 32'hffffffff : 32'h7fffffff);
										else if (overflow)
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = 32'h80000000;
										else
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = quotient;
									end
									3'b001: begin
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.busy = ~div_finished & ~(div_zero | overflow);
										if (div_zero)
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = dividend;
										else if (overflow)
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = 32'h00000000;
										else
											top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = remainder;
									end
									default: begin
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.busy = 1'b0;
										top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_exif.reg_wdata = 32'hbad3bad3;
									end
								endcase
						end
						assign rv32m_execute_t.CLK = CLK;
						assign rv32m_execute_t.nRST = nRST;
						assign rv32m_execute_t.idex = rv32m_idex;
						assign rv32m_exmem = rv32m_execute_t.exmem;
						if (1) begin : rv32m_memory_t
							wire CLK;
							wire nRST;
							wire [0:0] exmem;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_memif.exception = 1'b0;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_memif.busy = 1'b0;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_memif.reg_w = 1'b0;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_memif.mem_ren = 1'b0;
							assign top_level_bASIC.corewrap.RISCVBusiness.rmgmt.risc_mgmt_i.rv32m_memif.mem_wen = 1'b0;
						end
						assign rv32m_memory_t.CLK = CLK;
						assign rv32m_memory_t.nRST = nRST;
						assign rv32m_memory_t.exmem = rv32m_exmem;
						assign rv32m_idif.insn = d_insn[0+:32];
						assign d_insn_claim[0] = rv32m_idif.insn_claim;
						assign d_mem_to_reg[0] = rv32m_idif.mem_to_reg;
						assign d_rsel_s_0[0+:5] = rv32m_idif.rsel_s_0;
						assign d_rsel_s_1[0+:5] = rv32m_idif.rsel_s_1;
						assign d_rsel_d[0+:5] = rv32m_idif.rsel_d;
						assign e_exception[0] = rv32m_exif.exception;
						assign e_busy[0] = rv32m_exif.busy;
						assign e_branch_jump[0] = rv32m_exif.branch_jump;
						assign e_br_j_addr[0+:32] = rv32m_exif.br_j_addr;
						assign rv32m_exif.pc = e_pc[0+:32];
						assign e_reg_wdata[0+:32] = rv32m_exif.reg_wdata;
						assign e_reg_w[0] = rv32m_exif.reg_w;
						assign rv32m_exif.rdata_s_0 = e_rdata_s_0[0+:32];
						assign rv32m_exif.rdata_s_1 = e_rdata_s_1[0+:32];
						assign rv32m_exif.start = e_start[0];
						assign m_exception[0] = rv32m_memif.exception;
						assign m_busy[0] = rv32m_memif.busy;
						assign m_mem_addr[0+:32] = rv32m_memif.mem_addr;
						assign m_mem_ren[0] = rv32m_memif.mem_ren;
						assign m_mem_wen[0] = rv32m_memif.mem_wen;
						assign m_mem_byte_en[0+:4] = rv32m_memif.mem_byte_en;
						assign m_reg_wdata[0] = rv32m_memif.reg_wdata;
						assign m_reg_w[0] = rv32m_memif.reg_w;
						assign m_mem_store[0+:32] = rv32m_memif.mem_store;
						assign rv32m_memif.mem_busy = m_mem_busy[0];
						assign rv32m_memif.mem_load = m_mem_load[0+:32];
						assign d_insn = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.insn}};
						integer i;
						wire [0:0] tokens;
						wire ext_is_active;
						reg [-1:0] active_ext;
						assign tokens = d_insn_claim;
						assign ext_is_active = |tokens;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.active_insn = ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.ex_token = ext_is_active;
						always @(*) begin
							active_ext = 0;
							for (i = 0; i < N_EXTENSIONS; i = i + 1)
								if (tokens[i])
									active_ext = i;
						end
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.execute_stall = e_busy[active_ext] && ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.memory_stall = m_busy[active_ext] && ext_is_active;
						reg ex_start;
						always @(posedge CLK or negedge nRST)
							if (~nRST)
								ex_start <= 1'b0;
							else if (top_level_bASIC.corewrap.RISCVBusiness.rm_if.if_ex_enable)
								ex_start <= 1'b1;
							else
								ex_start <= 1'b0;
						assign e_start = {N_EXTENSIONS {ex_start}} & tokens;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_r = ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_s_0 = d_rsel_s_0[active_ext * 5+:5];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_s_1 = d_rsel_s_1[active_ext * 5+:5];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.rsel_d = d_rsel_d[active_ext * 5+:5];
						assign e_rdata_s_0 = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.rdata_s_0}};
						assign e_rdata_s_1 = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.rdata_s_1}};
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_reg_w = (e_reg_w[active_ext] || m_reg_w[active_ext]) && ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.reg_w = e_reg_w[active_ext] || m_reg_w[active_ext];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.reg_wdata = (e_reg_w[active_ext] ? e_reg_wdata[active_ext * 32+:32] : m_reg_wdata[active_ext]);
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_br_j = e_branch_jump[active_ext] && ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.branch_jump = e_branch_jump[active_ext];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.br_j_addr = e_br_j_addr[active_ext * 32+:32];
						assign e_pc = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.pc}};
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.req_mem = (m_mem_ren[active_ext] || m_mem_wen[active_ext]) && ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_addr = m_mem_addr[active_ext * 32+:32];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_byte_en = m_mem_byte_en[active_ext * 4+:4];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_store = m_mem_store[active_ext * 32+:32];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_ren = m_mem_ren[active_ext];
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_wen = m_mem_wen[active_ext];
						assign m_mem_busy = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_busy}};
						assign m_mem_load = {N_EXTENSIONS {top_level_bASIC.corewrap.RISCVBusiness.rm_if.mem_load}};
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.exception = (e_exception[active_ext] || m_exception[active_ext]) && ext_is_active;
						assign top_level_bASIC.corewrap.RISCVBusiness.rm_if.ex_cause = active_ext;
					end
					assign risc_mgmt_i.CLK = CLK;
					assign risc_mgmt_i.nRST = nRST;
				end
				assign rmgmt.CLK = CLK;
				assign rmgmt.nRST = nRST;
				if (1) begin : sep_caches
					wire CLK;
					wire nRST;
					localparam DCACHE_TYPE = "pass_through";
					case (DCACHE_TYPE)
						"pass_through": begin
							if (1) begin : dcache
								wire CLK;
								wire nRST;
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr;
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.ren;
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen;
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wdata = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.byte_en;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.rdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.busy = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy;
							end
							assign dcache.CLK = CLK;
							assign dcache.nRST = nRST;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dclear_done = 1'b1;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dflush_done = 1'b1;
						end
						"direct_mapped_tpf": begin
							if (1) begin : dcache
								wire CLK;
								wire nRST;
								wire clear;
								wire flush;
								wire clear_done;
								wire flush_done;
								localparam CACHE_SIZE = 1024;
								localparam BLOCK_SIZE = 2;
								localparam PREFETCH_LENGTH = 1;
								localparam NONCACHE_START_ADDR = 32'h80000000;
								localparam rv32i_types_pkg_WORD_SIZE = 32;
								localparam N_INDICES = 128;
								localparam BLK_OFF_BITS = 1;
								localparam IDX_BITS = 7;
								localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
								localparam TAG_BITS = 22;
								localparam N_BITS_IN_FRAME = 89;
								localparam N_BYTES_IN_FRAME = 12;
								localparam META_BYTE_L = 8;
								localparam META_BYTE_H = 11;
								localparam PFETCH_CNT_BITS = 0;
								reg [88:0] frame_buffer;
								reg [88:0] frame_buffer_next;
								wire [31:0] req_addr;
								reg init_flag;
								reg init_complete;
								wire flush_flag;
								reg flush_reg;
								reg flush_clear;
								wire clear_flag;
								reg clear_clear;
								reg clear_reg;
								wire request;
								wire direct_mem_req;
								reg hit;
								wire tag_match;
								wire [3:0] req_byte_en;
								wire [31:0] req_byte_en_expand;
								reg [31:0] curr_addr;
								reg [31:0] curr_addr_next;
								reg [88:0] cache_wdata;
								reg [31:0] cache_addr;
								reg [11:0] cache_byte_en;
								reg cache_wen;
								reg cache_ren;
								wire [88:0] cache_rdata;
								wire cache_busy;
								wire cache_busy_raw;
								reg [3:0] curr_state;
								reg [3:0] next_state;
								reg [31:0] sm_addr;
								if (1) begin : sm_bus_if
									localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
									wire [31:0] addr;
									localparam rv32i_types_pkg_WORD_SIZE = 32;
									reg [31:0] wdata;
									wire [31:0] rdata;
									reg ren;
									reg wen;
									wire busy;
									reg [3:0] byte_en;
								end
								reg [7:0] flush_cnt;
								reg [7:0] flush_cnt_next;
								reg [1:0] access_cnt;
								reg [1:0] access_cnt_next;
								reg [0:0] prefetch_cnt;
								reg [0:0] prefetch_cnt_next;
								config_ram_wrapper #(
									.N_BYTES(N_BYTES_IN_FRAME),
									.DEPTH(N_INDICES)
								) cache_mem(
									.CLK(CLK),
									.nRST(nRST),
									.wdata(cache_wdata),
									.addr(cache_addr),
									.byte_en(cache_byte_en),
									.wen(cache_wen),
									.ren(cache_ren),
									.rdata(cache_rdata),
									.busy(cache_busy_raw)
								);
								assign cache_busy = cache_busy_raw | ~(cache_wen | cache_ren);
								assign req_addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr;
								assign req_byte_en = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.byte_en;
								assign req_byte_en_expand = {{8 {req_byte_en[3]}}, {8 {req_byte_en[2]}}, {8 {req_byte_en[1]}}, {8 {req_byte_en[0]}}};
								assign direct_mem_req = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr >= NONCACHE_START_ADDR;
								assign flush_flag = (flush | init_flag) | flush_reg;
								assign clear_flag = clear | clear_reg;
								assign request = (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen | top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.ren) && ~direct_mem_req;
								assign clear_done = clear_clear;
								assign flush_done = flush_clear;
								assign tag_match = (req_addr[31-:22] == frame_buffer[85-:22]) && frame_buffer[87];
								always @(*) begin
									hit = 1'b0;
									if (curr_state == EVAL) begin
										if (tag_match && (~top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen || ~cache_busy))
											hit = 1'b1;
									end
									else if (((curr_state == FETCH) && (access_cnt == BLOCK_SIZE)) && ~cache_busy)
										hit = 1'b1;
								end
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.addr = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.addr : sm_bus_if.addr);
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wdata = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata : sm_bus_if.wdata);
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.ren : sm_bus_if.ren);
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen : sm_bus_if.wen);
								assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.byte_en : sm_bus_if.byte_en);
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.rdata = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.rdata : frame_buffer[req_addr[2-:1] * 32+:32]);
								assign sm_bus_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.rdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.busy = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy : ~hit);
								assign sm_bus_if.busy = (direct_mem_req ? 1'b1 : top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy);
								assign sm_bus_if.addr = sm_addr;
								always @(posedge CLK) begin
									frame_buffer <= frame_buffer_next;
									curr_addr <= curr_addr_next;
								end
								always @(posedge CLK or negedge nRST)
									if (~nRST) begin
										flush_cnt <= {8 {1'sb0}};
										access_cnt <= {2 {1'sb0}};
										prefetch_cnt <= 1'b0;
									end
									else begin
										flush_cnt <= flush_cnt_next;
										access_cnt <= access_cnt_next;
										prefetch_cnt <= prefetch_cnt_next;
									end
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										init_flag <= 1'b1;
									else if (init_complete)
										init_flag <= 1'b0;
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										flush_reg <= 1'b0;
									else if (flush_clear)
										flush_reg <= 1'b0;
									else if (flush)
										flush_reg <= 1'b1;
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										clear_reg <= 1'b0;
									else if (clear_clear)
										clear_reg <= 1'b0;
									else if (clear)
										clear_reg <= 1'b1;
								always @(*) begin
									flush_cnt_next = flush_cnt;
									cache_addr = {32 {1'sb0}};
									cache_ren = 0;
									cache_wen = 0;
									cache_byte_en = {12 {1'sb1}};
									cache_wdata = {89 {1'sb0}};
									curr_addr_next = curr_addr;
									frame_buffer_next = frame_buffer;
									access_cnt_next = access_cnt;
									prefetch_cnt_next = prefetch_cnt;
									sm_addr = {32 {1'sb0}};
									sm_bus_if.wen = 0;
									sm_bus_if.ren = 0;
									sm_bus_if.wdata = {32 {1'sb0}};
									sm_bus_if.byte_en = {4 {1'sb1}};
									init_complete = 1'b0;
									flush_clear = 1'b0;
									clear_clear = 1'b0;
									casez (curr_state)
										IDLE:
											if (clear_flag) begin
												curr_addr_next = req_addr;
												flush_cnt_next = 127;
											end
											else if (flush_flag) begin
												curr_addr_next = 0;
												flush_cnt_next = 0;
											end
											else if (request) begin
												cache_addr = req_addr[9-:7];
												cache_ren = 1'b1;
												if (~cache_busy) begin
													curr_addr_next = req_addr;
													frame_buffer_next = cache_rdata;
												end
											end
										EVAL:
											if (~tag_match)
												access_cnt_next = 0;
											else begin
												if (hit && frame_buffer[86]) begin
													{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
													prefetch_cnt_next = 0;
												end
												if (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen) begin
													cache_addr = req_addr[9-:7];
													frame_buffer_next[req_addr[2-:1] * 32+:32] = (frame_buffer[req_addr[2-:1] * 32+:32] & ~req_byte_en_expand) | (req_byte_en_expand & top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata);
													frame_buffer_next[88] = 1'b1;
													cache_wdata = frame_buffer_next;
													cache_wdata[86] = 1'b0;
													cache_wen = 1'b1;
												end
											end
										FETCH: begin
											sm_addr = curr_addr;
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											cache_wdata = frame_buffer;
											cache_addr = curr_addr[9-:7];
											if (access_cnt == BLOCK_SIZE) begin
												sm_bus_if.ren = 1'b0;
												cache_wen = 1'b1;
												if (~cache_busy) begin
													{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
													prefetch_cnt_next = 0;
												end
											end
											else begin
												sm_bus_if.ren = 1'b1;
												cache_wen = 1'b0;
											end
											frame_buffer_next[85-:22] = req_addr[31-:22];
											frame_buffer_next[86] = 0;
											frame_buffer_next[87] = 1;
											frame_buffer_next[88] = top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen;
											if (~sm_bus_if.busy) begin
												if (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen && (sm_addr[2-:1] == req_addr[2-:1]))
													frame_buffer_next[sm_addr[2-:1] * 32+:32] = (sm_bus_if.rdata & ~req_byte_en_expand) | (req_byte_en_expand & top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wdata);
												else
													frame_buffer_next[sm_addr[2-:1] * 32+:32] = sm_bus_if.rdata;
												access_cnt_next = access_cnt + 1;
											end
										end
										WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											if (~sm_bus_if.busy)
												access_cnt_next = access_cnt + 1;
											if (~sm_bus_if.busy && (access_cnt == 1))
												access_cnt_next = {2 {1'sb0}};
										end
										PREFETCH_PREP: begin
											cache_addr = curr_addr[9-:7];
											cache_ren = 1'b1;
											if (~cache_busy) begin
												access_cnt_next = 0;
												frame_buffer_next = cache_rdata;
											end
										end
										PREFETCH_WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											sm_bus_if.byte_en = 4'hf;
											if (~sm_bus_if.busy)
												if (access_cnt == 1)
													access_cnt_next = 0;
												else
													access_cnt_next = access_cnt + 1;
										end
										PREFETCH: begin
											sm_addr = curr_addr;
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											cache_addr = curr_addr[9-:7];
											cache_wdata = frame_buffer;
											if (access_cnt == BLOCK_SIZE) begin
												sm_bus_if.ren = 1'b0;
												cache_wen = 1'b1;
											end
											else begin
												sm_bus_if.ren = 1'b1;
												cache_wen = 1'b0;
											end
											if (~sm_bus_if.busy) begin
												access_cnt_next = access_cnt + 1;
												frame_buffer_next[access_cnt * 32+:32] = sm_bus_if.rdata;
												frame_buffer_next[86] = 1'b1;
												frame_buffer_next[87] = 1'b1;
												frame_buffer_next[88] = 1'b0;
												frame_buffer_next[85-:22] = curr_addr[31-:22];
											end
											if (~cache_busy) begin
												prefetch_cnt_next = prefetch_cnt + 1;
												{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
											end
										end
										CLEAR_PREP: begin
											cache_addr = curr_addr[9-:7];
											cache_ren = 1'b1;
											if (~cache_busy) begin
												frame_buffer_next = cache_rdata;
												if (frame_buffer_next[88] && frame_buffer_next[87])
													access_cnt_next = 0;
											end
										end
										CLEAR_WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											if (~sm_bus_if.busy)
												access_cnt_next = access_cnt + 1;
										end
										CLEAR_UPDATE: begin
											cache_addr = curr_addr[9-:7];
											cache_wen = 1'b1;
											cache_wdata = {89 {1'sb0}};
											cache_byte_en = {12 {1'sb1}};
											if (~cache_busy) begin
												flush_cnt_next = flush_cnt + 1;
												curr_addr_next[9-:7] = curr_addr[9-:7] + 1;
												if (flush_cnt == 127) begin
													init_complete = 1'b1;
													if (flush_reg)
														flush_clear = 1'b1;
													else if (clear_reg)
														clear_clear = 1'b1;
												end
											end
										end
									endcase
								end
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										curr_state <= IDLE;
									else
										curr_state <= next_state;
								always @(*) begin
									next_state = curr_state;
									casez (curr_state)
										IDLE:
											if (clear_flag || flush_flag)
												next_state = CLEAR_PREP;
											else if (~cache_busy && request)
												next_state = EVAL;
										EVAL:
											if (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.ren && tag_match) begin
												if (frame_buffer[86])
													next_state = PREFETCH_PREP;
												else
													next_state = IDLE;
											end
											else if (top_level_bASIC.corewrap.RISCVBusiness.tspp_dcache_gen_bus_if.wen && tag_match) begin
												if (~cache_busy)
													if (frame_buffer[86])
														next_state = PREFETCH_PREP;
													else
														next_state = IDLE;
											end
											else if (frame_buffer[87] && frame_buffer[88])
												next_state = WB;
											else
												next_state = FETCH;
										FETCH:
											if ((access_cnt == BLOCK_SIZE) && ~cache_busy)
												next_state = PREFETCH_PREP;
										PREFETCH_PREP:
											if (~cache_busy)
												if (frame_buffer_next[88] && frame_buffer_next[87])
													next_state = PREFETCH_WB;
												else
													next_state = PREFETCH;
										PREFETCH_WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = PREFETCH;
										PREFETCH:
											if ((access_cnt == BLOCK_SIZE) && ~cache_busy)
												if (prefetch_cnt == 0)
													next_state = IDLE;
												else
													next_state = PREFETCH_PREP;
										WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = FETCH;
										CLEAR_PREP:
											if (~cache_busy)
												if ((~init_flag && frame_buffer_next[88]) && frame_buffer_next[87])
													next_state = CLEAR_WB;
												else
													next_state = CLEAR_UPDATE;
										CLEAR_WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = CLEAR_UPDATE;
										CLEAR_UPDATE:
											if (~cache_busy)
												if (flush_cnt != 127)
													next_state = CLEAR_PREP;
												else
													next_state = IDLE;
									endcase
								end
							end
							assign dcache.CLK = CLK;
							assign dcache.nRST = nRST;
							assign dcache.flush = top_level_bASIC.corewrap.RISCVBusiness.cc_if.dcache_flush;
							assign dcache.clear = top_level_bASIC.corewrap.RISCVBusiness.cc_if.dcache_clear;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dflush_done = dcache.flush_done;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.dclear_done = dcache.clear_done;
						end
					endcase
					localparam ICACHE_TYPE = "pass_through";
					case (ICACHE_TYPE)
						"pass_through": begin
							if (1) begin : icache
								wire CLK;
								wire nRST;
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr;
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.ren;
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wen = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen;
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wdata = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.byte_en;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.rdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.busy = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy;
							end
							assign icache.CLK = CLK;
							assign icache.nRST = nRST;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.iclear_done = 1'b1;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.iflush_done = 1'b1;
						end
						"direct_mapped_tpf": begin
							if (1) begin : icache
								wire CLK;
								wire nRST;
								wire clear;
								wire flush;
								wire clear_done;
								wire flush_done;
								localparam CACHE_SIZE = 1024;
								localparam BLOCK_SIZE = 2;
								localparam PREFETCH_LENGTH = 1;
								localparam NONCACHE_START_ADDR = 32'h80000000;
								localparam rv32i_types_pkg_WORD_SIZE = 32;
								localparam N_INDICES = 128;
								localparam BLK_OFF_BITS = 1;
								localparam IDX_BITS = 7;
								localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
								localparam TAG_BITS = 22;
								localparam N_BITS_IN_FRAME = 89;
								localparam N_BYTES_IN_FRAME = 12;
								localparam META_BYTE_L = 8;
								localparam META_BYTE_H = 11;
								localparam PFETCH_CNT_BITS = 0;
								reg [88:0] frame_buffer;
								reg [88:0] frame_buffer_next;
								wire [31:0] req_addr;
								reg init_flag;
								reg init_complete;
								wire flush_flag;
								reg flush_reg;
								reg flush_clear;
								wire clear_flag;
								reg clear_clear;
								reg clear_reg;
								wire request;
								wire direct_mem_req;
								reg hit;
								wire tag_match;
								wire [3:0] req_byte_en;
								wire [31:0] req_byte_en_expand;
								reg [31:0] curr_addr;
								reg [31:0] curr_addr_next;
								reg [88:0] cache_wdata;
								reg [31:0] cache_addr;
								reg [11:0] cache_byte_en;
								reg cache_wen;
								reg cache_ren;
								wire [88:0] cache_rdata;
								wire cache_busy;
								wire cache_busy_raw;
								reg [3:0] curr_state;
								reg [3:0] next_state;
								reg [31:0] sm_addr;
								if (1) begin : sm_bus_if
									localparam rv32i_types_pkg_RAM_ADDR_SIZE = 32;
									wire [31:0] addr;
									localparam rv32i_types_pkg_WORD_SIZE = 32;
									reg [31:0] wdata;
									wire [31:0] rdata;
									reg ren;
									reg wen;
									wire busy;
									reg [3:0] byte_en;
								end
								reg [7:0] flush_cnt;
								reg [7:0] flush_cnt_next;
								reg [1:0] access_cnt;
								reg [1:0] access_cnt_next;
								reg [0:0] prefetch_cnt;
								reg [0:0] prefetch_cnt_next;
								config_ram_wrapper #(
									.N_BYTES(N_BYTES_IN_FRAME),
									.DEPTH(N_INDICES)
								) cache_mem(
									.CLK(CLK),
									.nRST(nRST),
									.wdata(cache_wdata),
									.addr(cache_addr),
									.byte_en(cache_byte_en),
									.wen(cache_wen),
									.ren(cache_ren),
									.rdata(cache_rdata),
									.busy(cache_busy_raw)
								);
								assign cache_busy = cache_busy_raw | ~(cache_wen | cache_ren);
								assign req_addr = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr;
								assign req_byte_en = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.byte_en;
								assign req_byte_en_expand = {{8 {req_byte_en[3]}}, {8 {req_byte_en[2]}}, {8 {req_byte_en[1]}}, {8 {req_byte_en[0]}}};
								assign direct_mem_req = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr >= NONCACHE_START_ADDR;
								assign flush_flag = (flush | init_flag) | flush_reg;
								assign clear_flag = clear | clear_reg;
								assign request = (top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen | top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.ren) && ~direct_mem_req;
								assign clear_done = clear_clear;
								assign flush_done = flush_clear;
								assign tag_match = (req_addr[31-:22] == frame_buffer[85-:22]) && frame_buffer[87];
								always @(*) begin
									hit = 1'b0;
									if (curr_state == EVAL) begin
										if (tag_match && (~top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen || ~cache_busy))
											hit = 1'b1;
									end
									else if (((curr_state == FETCH) && (access_cnt == BLOCK_SIZE)) && ~cache_busy)
										hit = 1'b1;
								end
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.addr = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.addr : sm_bus_if.addr);
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wdata = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wdata : sm_bus_if.wdata);
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.ren : sm_bus_if.ren);
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wen = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen : sm_bus_if.wen);
								assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.byte_en = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.byte_en : sm_bus_if.byte_en);
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.rdata = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.rdata : frame_buffer[req_addr[2-:1] * 32+:32]);
								assign sm_bus_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.rdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.busy = (direct_mem_req ? top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy : ~hit);
								assign sm_bus_if.busy = (direct_mem_req ? 1'b1 : top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy);
								assign sm_bus_if.addr = sm_addr;
								always @(posedge CLK) begin
									frame_buffer <= frame_buffer_next;
									curr_addr <= curr_addr_next;
								end
								always @(posedge CLK or negedge nRST)
									if (~nRST) begin
										flush_cnt <= {8 {1'sb0}};
										access_cnt <= {2 {1'sb0}};
										prefetch_cnt <= 1'b0;
									end
									else begin
										flush_cnt <= flush_cnt_next;
										access_cnt <= access_cnt_next;
										prefetch_cnt <= prefetch_cnt_next;
									end
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										init_flag <= 1'b1;
									else if (init_complete)
										init_flag <= 1'b0;
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										flush_reg <= 1'b0;
									else if (flush_clear)
										flush_reg <= 1'b0;
									else if (flush)
										flush_reg <= 1'b1;
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										clear_reg <= 1'b0;
									else if (clear_clear)
										clear_reg <= 1'b0;
									else if (clear)
										clear_reg <= 1'b1;
								always @(*) begin
									flush_cnt_next = flush_cnt;
									cache_addr = {32 {1'sb0}};
									cache_ren = 0;
									cache_wen = 0;
									cache_byte_en = {12 {1'sb1}};
									cache_wdata = {89 {1'sb0}};
									curr_addr_next = curr_addr;
									frame_buffer_next = frame_buffer;
									access_cnt_next = access_cnt;
									prefetch_cnt_next = prefetch_cnt;
									sm_addr = {32 {1'sb0}};
									sm_bus_if.wen = 0;
									sm_bus_if.ren = 0;
									sm_bus_if.wdata = {32 {1'sb0}};
									sm_bus_if.byte_en = {4 {1'sb1}};
									init_complete = 1'b0;
									flush_clear = 1'b0;
									clear_clear = 1'b0;
									casez (curr_state)
										IDLE:
											if (clear_flag) begin
												curr_addr_next = req_addr;
												flush_cnt_next = 127;
											end
											else if (flush_flag) begin
												curr_addr_next = 0;
												flush_cnt_next = 0;
											end
											else if (request) begin
												cache_addr = req_addr[9-:7];
												cache_ren = 1'b1;
												if (~cache_busy) begin
													curr_addr_next = req_addr;
													frame_buffer_next = cache_rdata;
												end
											end
										EVAL:
											if (~tag_match)
												access_cnt_next = 0;
											else begin
												if (hit && frame_buffer[86]) begin
													{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
													prefetch_cnt_next = 0;
												end
												if (top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen) begin
													cache_addr = req_addr[9-:7];
													frame_buffer_next[req_addr[2-:1] * 32+:32] = (frame_buffer[req_addr[2-:1] * 32+:32] & ~req_byte_en_expand) | (req_byte_en_expand & top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wdata);
													frame_buffer_next[88] = 1'b1;
													cache_wdata = frame_buffer_next;
													cache_wdata[86] = 1'b0;
													cache_wen = 1'b1;
												end
											end
										FETCH: begin
											sm_addr = curr_addr;
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											cache_wdata = frame_buffer;
											cache_addr = curr_addr[9-:7];
											if (access_cnt == BLOCK_SIZE) begin
												sm_bus_if.ren = 1'b0;
												cache_wen = 1'b1;
												if (~cache_busy) begin
													{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
													prefetch_cnt_next = 0;
												end
											end
											else begin
												sm_bus_if.ren = 1'b1;
												cache_wen = 1'b0;
											end
											frame_buffer_next[85-:22] = req_addr[31-:22];
											frame_buffer_next[86] = 0;
											frame_buffer_next[87] = 1;
											frame_buffer_next[88] = top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen;
											if (~sm_bus_if.busy) begin
												if (top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen && (sm_addr[2-:1] == req_addr[2-:1]))
													frame_buffer_next[sm_addr[2-:1] * 32+:32] = (sm_bus_if.rdata & ~req_byte_en_expand) | (req_byte_en_expand & top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wdata);
												else
													frame_buffer_next[sm_addr[2-:1] * 32+:32] = sm_bus_if.rdata;
												access_cnt_next = access_cnt + 1;
											end
										end
										WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											if (~sm_bus_if.busy)
												access_cnt_next = access_cnt + 1;
											if (~sm_bus_if.busy && (access_cnt == 1))
												access_cnt_next = {2 {1'sb0}};
										end
										PREFETCH_PREP: begin
											cache_addr = curr_addr[9-:7];
											cache_ren = 1'b1;
											if (~cache_busy) begin
												access_cnt_next = 0;
												frame_buffer_next = cache_rdata;
											end
										end
										PREFETCH_WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											sm_bus_if.byte_en = 4'hf;
											if (~sm_bus_if.busy)
												if (access_cnt == 1)
													access_cnt_next = 0;
												else
													access_cnt_next = access_cnt + 1;
										end
										PREFETCH: begin
											sm_addr = curr_addr;
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											cache_addr = curr_addr[9-:7];
											cache_wdata = frame_buffer;
											if (access_cnt == BLOCK_SIZE) begin
												sm_bus_if.ren = 1'b0;
												cache_wen = 1'b1;
											end
											else begin
												sm_bus_if.ren = 1'b1;
												cache_wen = 1'b0;
											end
											if (~sm_bus_if.busy) begin
												access_cnt_next = access_cnt + 1;
												frame_buffer_next[access_cnt * 32+:32] = sm_bus_if.rdata;
												frame_buffer_next[86] = 1'b1;
												frame_buffer_next[87] = 1'b1;
												frame_buffer_next[88] = 1'b0;
												frame_buffer_next[85-:22] = curr_addr[31-:22];
											end
											if (~cache_busy) begin
												prefetch_cnt_next = prefetch_cnt + 1;
												{curr_addr_next[31-:22], curr_addr_next[9-:7]} = {curr_addr[31-:22], curr_addr[9-:7]} + 1;
											end
										end
										CLEAR_PREP: begin
											cache_addr = curr_addr[9-:7];
											cache_ren = 1'b1;
											if (~cache_busy) begin
												frame_buffer_next = cache_rdata;
												if (frame_buffer_next[88] && frame_buffer_next[87])
													access_cnt_next = 0;
											end
										end
										CLEAR_WB: begin
											sm_addr[9-:7] = curr_addr[9-:7];
											sm_addr[1-:2] = 2'b00;
											sm_addr[2-:1] = access_cnt[0:0];
											sm_addr[31-:22] = frame_buffer[85-:22];
											sm_bus_if.wdata = frame_buffer[access_cnt * 32+:32];
											sm_bus_if.wen = 1'b1;
											if (~sm_bus_if.busy)
												access_cnt_next = access_cnt + 1;
										end
										CLEAR_UPDATE: begin
											cache_addr = curr_addr[9-:7];
											cache_wen = 1'b1;
											cache_wdata = {89 {1'sb0}};
											cache_byte_en = {12 {1'sb1}};
											if (~cache_busy) begin
												flush_cnt_next = flush_cnt + 1;
												curr_addr_next[9-:7] = curr_addr[9-:7] + 1;
												if (flush_cnt == 127) begin
													init_complete = 1'b1;
													if (flush_reg)
														flush_clear = 1'b1;
													else if (clear_reg)
														clear_clear = 1'b1;
												end
											end
										end
									endcase
								end
								always @(posedge CLK or negedge nRST)
									if (~nRST)
										curr_state <= IDLE;
									else
										curr_state <= next_state;
								always @(*) begin
									next_state = curr_state;
									casez (curr_state)
										IDLE:
											if (clear_flag || flush_flag)
												next_state = CLEAR_PREP;
											else if (~cache_busy && request)
												next_state = EVAL;
										EVAL:
											if (top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.ren && tag_match) begin
												if (frame_buffer[86])
													next_state = PREFETCH_PREP;
												else
													next_state = IDLE;
											end
											else if (top_level_bASIC.corewrap.RISCVBusiness.tspp_icache_gen_bus_if.wen && tag_match) begin
												if (~cache_busy)
													if (frame_buffer[86])
														next_state = PREFETCH_PREP;
													else
														next_state = IDLE;
											end
											else if (frame_buffer[87] && frame_buffer[88])
												next_state = WB;
											else
												next_state = FETCH;
										FETCH:
											if ((access_cnt == BLOCK_SIZE) && ~cache_busy)
												next_state = PREFETCH_PREP;
										PREFETCH_PREP:
											if (~cache_busy)
												if (frame_buffer_next[88] && frame_buffer_next[87])
													next_state = PREFETCH_WB;
												else
													next_state = PREFETCH;
										PREFETCH_WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = PREFETCH;
										PREFETCH:
											if ((access_cnt == BLOCK_SIZE) && ~cache_busy)
												if (prefetch_cnt == 0)
													next_state = IDLE;
												else
													next_state = PREFETCH_PREP;
										WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = FETCH;
										CLEAR_PREP:
											if (~cache_busy)
												if ((~init_flag && frame_buffer_next[88]) && frame_buffer_next[87])
													next_state = CLEAR_WB;
												else
													next_state = CLEAR_UPDATE;
										CLEAR_WB:
											if (~sm_bus_if.busy && (access_cnt == 1))
												next_state = CLEAR_UPDATE;
										CLEAR_UPDATE:
											if (~cache_busy)
												if (flush_cnt != 127)
													next_state = CLEAR_PREP;
												else
													next_state = IDLE;
									endcase
								end
							end
							assign icache.CLK = CLK;
							assign icache.nRST = nRST;
							assign icache.flush = top_level_bASIC.corewrap.RISCVBusiness.cc_if.icache_flush;
							assign icache.clear = top_level_bASIC.corewrap.RISCVBusiness.cc_if.icache_clear;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.iflush_done = icache.flush_done;
							assign top_level_bASIC.corewrap.RISCVBusiness.cc_if.iclear_done = icache.clear_done;
						end
					endcase
				end
				assign sep_caches.CLK = CLK;
				assign sep_caches.nRST = nRST;
				if (1) begin : mc
					wire CLK;
					wire nRST;
					reg [31:0] current_state;
					reg [31:0] next_state;
					wire [31:0] wdata;
					wire [31:0] rdata;
					always @(posedge CLK or negedge nRST)
						if (nRST == 0)
							current_state <= IDLE;
						else
							current_state <= next_state;
					always @(*)
						case (current_state)
							IDLE:
								if (top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren || top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen)
									next_state = DATA_REQ;
								else if (top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren)
									next_state = INSTR_REQ;
								else
									next_state = IDLE;
							INSTR_REQ:
								if (!top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren)
									next_state = IDLE;
								else if (top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren || top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen)
									next_state = INSTR_DATA_REQ;
								else
									next_state = INSTR_WAIT;
							INSTR_DATA_REQ:
								if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy == 1'b0)
									next_state = DATA_WAIT;
								else
									next_state = INSTR_DATA_REQ;
							DATA_REQ:
								if (top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren)
									next_state = DATA_INSTR_REQ;
								else
									next_state = DATA_WAIT;
							DATA_INSTR_REQ:
								if (!top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren && (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy == 1'b0))
									next_state = IDLE;
								else if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy == 1'b0)
									next_state = INSTR_WAIT;
								else
									next_state = DATA_INSTR_REQ;
							INSTR_WAIT:
								if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy == 1'b0) begin
									if (top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren || top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen)
										next_state = DATA_REQ;
									else
										next_state = IDLE;
								end
								else if (top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren || top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen)
									next_state = INSTR_DATA_REQ;
								else
									next_state = INSTR_WAIT;
							DATA_WAIT:
								if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy == 1'b0) begin
									if (top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren)
										next_state = INSTR_REQ;
									else
										next_state = IDLE;
								end
								else if (top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren)
									next_state = DATA_INSTR_REQ;
								else
									next_state = DATA_WAIT;
							default: next_state = IDLE;
						endcase
					always @(*)
						case (current_state)
							IDLE: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = 1'b1;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = 1'b1;
							end
							INSTR_REQ: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wen;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.addr;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = 1'b1;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = 1'b1;
							end
							INSTR_DATA_REQ: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.addr;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = 1'b1;
							end
							INSTR_WAIT: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = 0;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = 1'b1;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy;
							end
							DATA_REQ: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.addr;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = 1'b1;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = 1'b1;
							end
							DATA_INSTR_REQ: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.wen;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.ren;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.addr;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = 1'b1;
							end
							DATA_WAIT: begin
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wen;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.ren;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.addr;
								top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.byte_en;
								top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.busy = 1'b1;
								top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.busy = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy;
							end
						endcase
					localparam BUS_ENDIANNESS = "big";
					if (BUS_ENDIANNESS == "big") begin
						assign wdata = top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wdata;
						assign rdata = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.rdata;
					end
					else if (BUS_ENDIANNESS == "little") begin
						wire [31:0] little_endian_wdata;
						wire [31:0] little_endian_rdata;
						endian_swapper wswap(
							.word_in(top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.wdata),
							.word_out(little_endian_wdata)
						);
						endian_swapper rswap(
							.word_in(top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.rdata),
							.word_out(little_endian_rdata)
						);
						assign wdata = little_endian_wdata;
						assign rdata = little_endian_rdata;
					end
					assign top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wdata = wdata;
					assign top_level_bASIC.corewrap.RISCVBusiness.dcache_mc_if.rdata = rdata;
					assign top_level_bASIC.corewrap.RISCVBusiness.icache_mc_if.rdata = rdata;
				end
				assign mc.CLK = CLK;
				assign mc.nRST = nRST;
				if (1) begin : sparce_wrapper_i
					wire CLK;
					wire nRST;
					localparam SPARCE_ENABLED = "enabled";
					case (SPARCE_ENABLED)
						"disabled": begin
							if (1) begin : sparce
								wire CLK;
								wire nRST;
								assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.skipping = 1'b0;
								assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sparce_target = {32 {1'sb0}};
							end
							assign sparce.CLK = CLK;
							assign sparce.nRST = nRST;
						end
						"enabled": begin
							if (1) begin : sparce
								wire CLK;
								wire nRST;
								if (1) begin : internal_if
									localparam rv32i_types_pkg_WORD_SIZE = 32;
									wire [31:0] wb_data;
									wire is_sparse;
									wire wb_en;
									wire rs1_sparsity;
									wire rs2_sparsity;
									wire [31:0] pc;
									wire [31:0] sasa_addr;
									wire [31:0] sasa_data;
									wire [31:0] preceding_pc;
									wire sasa_wen;
									wire valid;
									wire sasa_enable;
									wire [4:0] sasa_rs1;
									wire [4:0] sasa_rs2;
									wire [4:0] rd;
									wire condition;
									wire [4:0] insts_to_skip;
									wire [31:0] sparce_target;
									wire skipping;
									wire ctrl_flow_enable;
									wire [31:0] rdata;
								end
								assign internal_if.pc = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.pc;
								assign internal_if.wb_data = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.wb_data;
								assign internal_if.wb_en = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.wb_en;
								assign internal_if.sasa_data = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_data;
								assign internal_if.sasa_addr = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_addr;
								assign internal_if.sasa_wen = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sasa_wen;
								assign internal_if.rd = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.rd;
								assign internal_if.sasa_enable = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.if_ex_enable;
								assign internal_if.rdata = top_level_bASIC.corewrap.RISCVBusiness.sparce_if.rdata;
								assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.skipping = internal_if.skipping;
								assign top_level_bASIC.corewrap.RISCVBusiness.sparce_if.sparce_target = internal_if.sparce_target;
								if (1) begin : sparce_svc_i
									assign top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.is_sparse = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.wb_data == 0;
								end
								if (1) begin : sparce_sprf_i
									wire CLK;
									wire nRST;
									reg [31:1] sparsity_reg;
									wire [31:0] sparsity_out;
									assign sparsity_out[0] = 1'b1;
									assign sparsity_out[31:1] = sparsity_reg[31:1];
									always @(posedge CLK or negedge nRST)
										if (!nRST)
											sparsity_reg[31:1] <= {31 {1'sb0}};
										else begin
											sparsity_reg[31:1] <= sparsity_reg[31:1];
											if (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.wb_en)
												sparsity_reg[top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rd] <= top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.is_sparse;
										end
									always @(*) begin
										if (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1 == {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1)) {1'sb0}})
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs1_sparsity = 1'b1;
										else if ((top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1 == top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rd) && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.wb_en)
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs1_sparsity = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.is_sparse;
										else
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs1_sparsity = sparsity_out[top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1];
										if (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2 == {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2)) {1'sb0}})
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs2_sparsity = 1'b1;
										else if ((top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2 == top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rd) && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.wb_en)
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs2_sparsity = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.is_sparse;
										else
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs2_sparsity = sparsity_out[top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2];
									end
								end
								assign sparce_sprf_i.CLK = CLK;
								assign sparce_sprf_i.nRST = nRST;
								localparam signed [31:0] _param_B5178_SASA_SETS = 4;
								if (1) begin : sparce_sasa_table_i
									localparam SASA_ENTRIES = 16;
									localparam SASA_SETS = _param_B5178_SASA_SETS;
									localparam SASA_ADDR = 32'h90000000;
									wire CLK;
									wire nRST;
									reg [31:0] usage;
									reg [15:0] valid;
									reg [223:0] tag;
									reg [79:0] rs1;
									reg [79:0] rs2;
									reg [15:0] sasa_cond;
									reg [79:0] insts_to_skip;
									reg [31:0] input_data;
									wire [1:0] input_idx;
									wire [1:0] pc_idx;
									wire [13:0] pc_tag;
									wire sasa_match;
									reg existing_entry;
									reg [1:0] existing_entry_set;
									reg [_param_B5178_SASA_SETS:0] sasa_hits;
									reg sasa_config;
									wire sasa_config_match;
									assign sasa_config_match = (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_enable && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_wen) && (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_addr == 33'sd2415919108);
									always @(posedge CLK or negedge nRST) begin : sasa_configuration
										if (!nRST)
											sasa_config <= 1'b0;
										else begin
											sasa_config <= sasa_config;
											if (sasa_config_match)
												sasa_config <= top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data;
										end
									end
									assign input_idx = input_data[31-:16];
									assign pc_idx = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.pc >> 2;
									assign pc_tag = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.pc >> 4;
									assign sasa_match = (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_enable && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_wen) && (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_addr == SASA_ADDR);
									function automatic [0:0] sv2v_cast_1;
										input reg [0:0] inp;
										sv2v_cast_1 = inp;
									endfunction
									always @(*) begin : sasa_input_conversion
										input_data[31-:16] = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data[31:16];
										input_data[15-:5] = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data[15:11];
										input_data[10-:5] = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data[10:6];
										input_data[5] = sv2v_cast_1(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data[5]);
										input_data[4-:5] = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_data[4:0];
									end
									always @(posedge CLK or negedge nRST) begin : sasa_table_entries
										if (!nRST) begin : sv2v_autoblock_22
											reg signed [31:0] i;
											for (i = 0; i < SASA_SETS; i = i + 1)
												begin : sv2v_autoblock_23
													reg signed [31:0] j;
													for (j = 0; j < 4; j = j + 1)
														begin
															usage[((i * 4) + j) * 2+:2] <= 3 - i;
															valid[(i * 4) + j] <= 1'b0;
															tag[((i * 4) + j) * 14+:14] <= {14 {1'sb0}};
															rs1[((i * 4) + j) * 5+:5] <= {5 {1'sb0}};
															rs2[((i * 4) + j) * 5+:5] <= {5 {1'sb0}};
															sasa_cond[(i * 4) + j] <= SASA_COND_OR;
															insts_to_skip[((i * 4) + j) * 5+:5] <= {5 {1'sb0}};
														end
												end
										end
										else begin
											usage <= usage;
											valid <= valid;
											tag <= tag;
											rs1 <= rs1;
											rs2 <= rs2;
											sasa_cond <= sasa_cond;
											insts_to_skip <= insts_to_skip;
											if (sasa_match) begin
												if (!existing_entry) begin : sv2v_autoblock_24
													reg signed [31:0] i;
													for (i = 0; i < SASA_SETS; i = i + 1)
														begin
															if ((usage[((i * 4) + input_idx) * 2+:2] == {2 {1'sb1}}) || 1'd0) begin
																valid[(i * 4) + input_idx] <= 1;
																tag[((i * 4) + input_idx) * 14+:14] <= input_data[31-:16] >> 2;
																rs1[((i * 4) + input_idx) * 5+:5] <= input_data[15-:5];
																rs2[((i * 4) + input_idx) * 5+:5] <= input_data[10-:5];
																sasa_cond[(i * 4) + input_idx] <= input_data[5];
																insts_to_skip[((i * 4) + input_idx) * 5+:5] <= input_data[4-:5];
															end
															usage[((i * 4) + input_idx) * 2+:2] <= (usage[((i * 4) + input_idx) * 2+:2] + 1) % SASA_SETS;
														end
												end
												else begin
													valid[(existing_entry_set * 4) + input_idx] <= 1;
													tag[((existing_entry_set * 4) + input_idx) * 14+:14] <= input_data[31-:16] >> 2;
													rs1[((existing_entry_set * 4) + input_idx) * 5+:5] <= input_data[15-:5];
													rs2[((existing_entry_set * 4) + input_idx) * 5+:5] <= input_data[10-:5];
													sasa_cond[(existing_entry_set * 4) + input_idx] <= input_data[5];
													insts_to_skip[((existing_entry_set * 4) + input_idx) * 5+:5] <= input_data[4-:5];
													begin : sv2v_autoblock_25
														reg signed [31:0] i;
														for (i = 0; i < SASA_SETS; i = i + 1)
															if (usage[((i * 4) + input_idx) * 2+:2] < usage[((existing_entry_set * 4) + input_idx) * 2+:2])
																usage[((i * 4) + input_idx) * 2+:2] <= (usage[((i * 4) + input_idx) * 2+:2] + 1) % SASA_SETS;
													end
												end
											end
											else if (sasa_hits != 0) begin : sv2v_autoblock_26
												reg signed [31:0] i;
												for (i = 0; i < SASA_SETS; i = i + 1)
													if (usage[((i * 4) + pc_idx) * 2+:2] < usage[(((sasa_hits - 1) * 4) + pc_idx) * 2+:2])
														usage[((i * 4) + pc_idx) * 2+:2] <= (usage[((i * 4) + pc_idx) * 2+:2] + 1) % SASA_SETS;
													else if (i == (sasa_hits - 1))
														usage[((i * 4) + pc_idx) * 2+:2] <= {2 {1'sb0}};
											end
										end
									end
									always @(*) begin : sasa_outputs
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1 = {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1)) {1'sb0}};
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2 = {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2)) {1'sb0}};
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.insts_to_skip = {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.insts_to_skip)) {1'sb0}};
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.preceding_pc = top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.pc;
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.condition = SASA_COND_OR;
										top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.valid = 1'b0;
										sasa_hits = {5 {1'sb0}};
										existing_entry = 0;
										existing_entry_set = 0;
										begin : sv2v_autoblock_27
											reg signed [31:0] i;
											for (i = 0; i < SASA_SETS; i = i + 1)
												begin
													if (valid[(i * 4) + pc_idx] && (tag[((i * 4) + pc_idx) * 14+:14] == pc_tag)) begin
														sasa_hits = i + 1;
														top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.valid = (sasa_config == 1'b0) && (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.pc[31:18] == {14 * $bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.pc[0])) {1'sb0}});
														top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs1 = rs1[((i * 4) + pc_idx) * 5+:5];
														top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sasa_rs2 = rs2[((i * 4) + pc_idx) * 5+:5];
														top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.condition = sasa_cond[(i * 4) + pc_idx];
														top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.insts_to_skip = insts_to_skip[((i * 4) + pc_idx) * 5+:5];
													end
													if (valid[(i * 4) + input_idx] && (tag[((i * 4) + input_idx) * 14+:14] == (input_data[31-:16] >> 2))) begin
														existing_entry = 1;
														existing_entry_set = i;
													end
												end
										end
									end
								end
								assign sparce_sasa_table_i.CLK = CLK;
								assign sparce_sasa_table_i.nRST = nRST;
								if (1) begin : sparce_psru_i
									always @(*)
										if (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.valid) begin
											if (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.condition == SASA_COND_OR)
												top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.skipping = (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs1_sparsity || top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs2_sparsity) && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.ctrl_flow_enable;
											else
												top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.skipping = (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs1_sparsity && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rs2_sparsity) && top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.ctrl_flow_enable;
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sparce_target = (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.preceding_pc + (top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.insts_to_skip << 2)) + 4;
										end
										else begin
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.skipping = 1'b0;
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sparce_target = {$bits(type(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.sparce_target)) {1'sb1}};
										end
								end
								if (1) begin : sparce_cfid_i
									localparam rv32i_types_pkg_OP_W = 7;
									wire [6:0] cf_op;
									function automatic [6:0] sv2v_cast_7;
										input reg [6:0] inp;
										sv2v_cast_7 = inp;
									endfunction
									assign cf_op = sv2v_cast_7(top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.rdata[6:0]);
									always @(*)
										if (((cf_op == rv32i_types_pkg_BRANCH) || (cf_op == rv32i_types_pkg_JAL)) || (cf_op == rv32i_types_pkg_JALR))
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.ctrl_flow_enable = 0;
										else
											top_level_bASIC.corewrap.RISCVBusiness.sparce_wrapper_i.sparce.internal_if.ctrl_flow_enable = 1;
								end
							end
							assign sparce.CLK = CLK;
							assign sparce.nRST = nRST;
						end
					endcase
				end
				assign sparce_wrapper_i.CLK = CLK;
				assign sparce_wrapper_i.nRST = nRST;
				localparam _bbase_5477D_ahb_m = 0;
				if (1) begin : bt
					wire CLK;
					wire nRST;
					localparam _mbase_ahb_m = _bbase_5477D_ahb_m;
					reg state;
					reg n_state;
					always @(posedge CLK or negedge nRST)
						if (~nRST)
							state <= IDLE;
						else
							state <= n_state;
					always @(*)
						if ((state == DATA) & !top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HREADY)
							n_state = state;
						else
							n_state = (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren | top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen ? DATA : IDLE);
					always @(*)
						if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en == 4'b1111)
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HSIZE = 3'b010;
						else if ((top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en == 4'b1100) || (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.byte_en == 4'b0011))
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HSIZE = 3'b001;
						else
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HSIZE = 3'b000;
					always @(*) begin
						if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.ren) begin
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HTRANS = 2'b10;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWRITE = 1'b0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HADDR = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWDATA = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wdata;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HBURST = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HPROT = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HMASTLOCK = 0;
						end
						else if (top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wen) begin
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HTRANS = 2'b10;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWRITE = 1'b1;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HADDR = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.addr;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWDATA = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wdata;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HBURST = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HPROT = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HMASTLOCK = 0;
						end
						else begin
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HTRANS = 2'b00;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWRITE = 1'b0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HADDR = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWDATA = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wdata;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HBURST = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HPROT = 0;
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HMASTLOCK = 0;
						end
						if (state == DATA)
							top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HWDATA = top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.wdata;
					end
					assign top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.busy = (state == IDLE) || ~(top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HREADY && (state == DATA));
					assign top_level_bASIC.corewrap.RISCVBusiness.pipeline_trans_if.rdata = top_level_bASIC.corewrap.ahb_ifs[_mbase_ahb_m].HRDATA;
				end
				assign bt.CLK = CLK;
				assign bt.nRST = nRST;
			end
			assign RISCVBusiness.CLK = clk;
			assign RISCVBusiness.nRST = M0_RST;
			wire [4:1] sv2v_tmp_66F49;
			assign sv2v_tmp_66F49 = {4 {1'sb0}};
			always @(*) ahb_ifs[1].HPROT = sv2v_tmp_66F49;
			wire [1:1] sv2v_tmp_41F41;
			assign sv2v_tmp_41F41 = 1'b0;
			always @(*) ahb_ifs[1].HMASTLOCK = sv2v_tmp_41F41;
			wire [1:1] sv2v_tmp_debugger_top_HWRITE;
			always @(*) ahb_ifs[1].HWRITE = sv2v_tmp_debugger_top_HWRITE;
			wire [3:1] sv2v_tmp_debugger_top_HSIZE;
			always @(*) ahb_ifs[1].HSIZE = sv2v_tmp_debugger_top_HSIZE;
			wire [3:1] sv2v_tmp_debugger_top_HBURST;
			always @(*) ahb_ifs[1].HBURST = sv2v_tmp_debugger_top_HBURST;
			wire [2:1] sv2v_tmp_debugger_top_HTRANS;
			always @(*) ahb_ifs[1].HTRANS = sv2v_tmp_debugger_top_HTRANS;
			wire [32:1] sv2v_tmp_debugger_top_HADDR;
			always @(*) ahb_ifs[1].HADDR = sv2v_tmp_debugger_top_HADDR;
			wire [32:1] sv2v_tmp_debugger_top_HWDATA;
			always @(*) ahb_ifs[1].HWDATA = sv2v_tmp_debugger_top_HWDATA;
			debugger_top debugger_top(
				.clk(clk),
				.rst_n(rst_n),
				.HREADY(ahb_ifs[1].HREADY),
				.HRDATA(ahb_ifs[1].HRDATA),
				.HWRITE(sv2v_tmp_debugger_top_HWRITE),
				.HSIZE(sv2v_tmp_debugger_top_HSIZE),
				.HBURST(sv2v_tmp_debugger_top_HBURST),
				.HTRANS(sv2v_tmp_debugger_top_HTRANS),
				.HADDR(sv2v_tmp_debugger_top_HADDR),
				.HWDATA(sv2v_tmp_debugger_top_HWDATA),
				.M0_RST(M0_RST),
				.rx(rx),
				.tx(tx)
			);
		end
		assign corewrap.clk = clk;
		assign corewrap.rst_n = rst_n;
		assign corewrap.rx = uart_debug_rx;
		assign corewrap.wbs_stb_i = wbs_stb_i;
		assign corewrap.wbs_cyc_i = wbs_cyc_i;
		assign corewrap.wbs_we_i = wbs_we_i;
		assign corewrap.wbs_sel_i = wbs_sel_i;
		assign corewrap.wbs_dat_i = wbs_dat_i;
		assign corewrap.wbs_adr_i = wbs_adr_i;
		assign wbs_ack_o = corewrap.wbs_ack_o;
		assign wbs_dat_o = corewrap.wbs_dat_o;
		wire uar_tx;
		assign uar_tx = corewrap.tx;
	endgenerate
	generate
		localparam signed [31:0] _param_7B60C_NUM_SLAVES = 5;
		if (1) begin : ahb2apb
			wire clk;
			wire n_rst;
			localparam NUM_SLAVES = _param_7B60C_NUM_SLAVES;
			wire [31:0] PRDATA_PSlave;
			APB_Bridge APB_Bridge(
				.clk(clk),
				.n_rst(n_rst),
				.HTRANS(top_level_bASIC.ahb2apbIf_ahbif.HTRANS),
				.HWRITE(top_level_bASIC.ahb2apbIf_ahbif.HWRITE),
				.HADDR(top_level_bASIC.ahb2apbIf_ahbif.HADDR),
				.HWDATA(top_level_bASIC.ahb2apbIf_ahbif.HWDATA),
				.PRDATA(PRDATA_PSlave),
				.HREADY(top_level_bASIC.ahb2apbIf_ahbif.HREADYOUT),
				.HRESP(top_level_bASIC.ahb2apbIf_ahbif.HRESP),
				.HRDATA(top_level_bASIC.ahb2apbIf_ahbif.HRDATA),
				.PWDATA(top_level_bASIC.ahb2apbIf_apbif.PWDATA),
				.PADDR(top_level_bASIC.ahb2apbIf_apbif.PADDR),
				.PWRITE(top_level_bASIC.ahb2apbIf_apbif.PWRITE),
				.PENABLE(top_level_bASIC.ahb2apbIf_apbif.PENABLE),
				.psel_en(top_level_bASIC.ahb2apbIf_apbif.PSEL)
			);
			APB_Decoder #(.NUM_SLAVES(NUM_SLAVES)) APB_Decoder(
				.PADDR(top_level_bASIC.ahb2apbIf_apbif.PADDR),
				.psel_en(top_level_bASIC.ahb2apbIf_apbif.PSEL),
				.PRData_in(top_level_bASIC.ahb2apbIf.PRData_slave),
				.PRDATA_PSlave(PRDATA_PSlave),
				.PSEL_slave(top_level_bASIC.ahb2apbIf.PSEL_slave)
			);
		end
		assign ahb2apb.clk = clk;
		assign ahb2apb.n_rst = rst_n;
	endgenerate
	generate
		localparam signed [31:0] _param_6B795_NUM_PINS = GPIO_PIN_COUNT;
		if (1) begin : Gpio0
			localparam NUM_PINS = _param_6B795_NUM_PINS;
			wire clk;
			wire n_rst;
			localparam DATA_IND = 0;
			localparam EN_IND = 1;
			localparam INTR_EN_IND = 2;
			localparam INTR_POSEDGE_IND = 3;
			localparam INTR_NEGEDGE_IND = 4;
			localparam INTR_CLR_IND = 5;
			localparam INTR_STAT_IND = 6;
			localparam NUM_REGISTERS = 7;
			genvar i;
			reg [(7 * _param_6B795_NUM_PINS) - 1:0] registers;
			wire [31:0] apb_data;
			wire [223:0] apb_read;
			wire [6:0] wen;
			reg [_param_6B795_NUM_PINS - 1:0] read_r;
			reg [_param_6B795_NUM_PINS - 1:0] sync_in;
			reg [_param_6B795_NUM_PINS - 1:0] sync_out;
			wire [_param_6B795_NUM_PINS - 1:0] pos_edge;
			wire [_param_6B795_NUM_PINS - 1:0] neg_edge;
			wire [_param_6B795_NUM_PINS - 1:0] gen_intr;
			assign top_level_bASIC.gpioIf0.en_data = registers[EN_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS];
			assign top_level_bASIC.gpioIf0.interrupt = registers[INTR_STAT_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS];
			GPIO_SlaveInterface #(
				.NUM_REGS(NUM_REGISTERS),
				.ADDR_OFFSET(4)
			) apbs(
				.clk(clk),
				.n_rst(n_rst),
				.PADDR(top_level_bASIC.gpio0apbIf.PADDR),
				.PWDATA(top_level_bASIC.gpio0apbIf.PWDATA),
				.PENABLE(top_level_bASIC.gpio0apbIf.PENABLE),
				.PWRITE(top_level_bASIC.gpio0apbIf.PWRITE),
				.PSEL(top_level_bASIC.gpio0apbIf.PSEL),
				.PRDATA(top_level_bASIC.gpio0apbIf.PRDATA),
				.read_data(apb_read),
				.w_enable(wen),
				.w_data(apb_data)
			);
			edge_detector #(.WIDTH(NUM_PINS)) edgd(
				.clk(clk),
				.n_rst(n_rst),
				.signal(sync_out),
				.pos_edge(pos_edge),
				.neg_edge(neg_edge)
			);
			assign top_level_bASIC.gpioIf0.gpio_bidir[0] = (top_level_bASIC.gpioIf0.en_data[0] ? top_level_bASIC.gpioIf0.w_data[0] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[1] = (top_level_bASIC.gpioIf0.en_data[1] ? top_level_bASIC.gpioIf0.w_data[1] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[2] = (top_level_bASIC.gpioIf0.en_data[2] ? top_level_bASIC.gpioIf0.w_data[2] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[3] = (top_level_bASIC.gpioIf0.en_data[3] ? top_level_bASIC.gpioIf0.w_data[3] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[4] = (top_level_bASIC.gpioIf0.en_data[4] ? top_level_bASIC.gpioIf0.w_data[4] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[5] = (top_level_bASIC.gpioIf0.en_data[5] ? top_level_bASIC.gpioIf0.w_data[5] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[6] = (top_level_bASIC.gpioIf0.en_data[6] ? top_level_bASIC.gpioIf0.w_data[6] : 1'bz);
			assign top_level_bASIC.gpioIf0.gpio_bidir[7] = (top_level_bASIC.gpioIf0.en_data[7] ? top_level_bASIC.gpioIf0.w_data[7] : 1'bz);
			always @(posedge clk) begin
				top_level_bASIC.gpioIf0.r_data <= top_level_bASIC.gpioIf0.gpio_bidir;
				top_level_bASIC.gpioIf0.w_data <= registers[0+:_param_6B795_NUM_PINS];
			end
			always @(posedge clk or negedge n_rst)
				if (~n_rst) begin
					sync_out <= {_param_6B795_NUM_PINS {1'sb0}};
					sync_in <= {_param_6B795_NUM_PINS {1'sb0}};
				end
				else begin
					sync_in <= top_level_bASIC.gpioIf0.r_data;
					sync_out <= sync_in;
				end
			always @(posedge clk or negedge n_rst)
				if (~n_rst)
					read_r <= {_param_6B795_NUM_PINS {1'sb0}};
				else
					read_r <= sync_out;
			for (i = 0; i < NUM_REGISTERS; i = i + 1) begin : APB_write_slave_regs
				if ((i != INTR_STAT_IND) && (i != INTR_CLR_IND)) always @(posedge clk or negedge n_rst)
					if (~n_rst)
						registers[i * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= {_param_6B795_NUM_PINS {1'sb0}};
					else if (wen[i])
						registers[i * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= apb_data[NUM_PINS - 1:0];
			end
			for (i = 0; i < NUM_REGISTERS; i = i + 1) begin : generate_r_data_APB_slave
				if (NUM_PINS < 32) assign apb_read[(i * 32) + (31 >= NUM_PINS ? 31 : (31 + (31 >= NUM_PINS ? 32 - NUM_PINS : NUM_PINS - 30)) - 1)-:(31 >= NUM_PINS ? 32 - NUM_PINS : NUM_PINS - 30)] = {(31 >= NUM_PINS ? 32 - NUM_PINS : NUM_PINS - 30) {1'sb0}};
				if (i == DATA_IND) begin
					assign apb_read[(i * 32) + (NUM_PINS - 1)-:NUM_PINS] = read_r;
				end
				else assign apb_read[(i * 32) + (NUM_PINS - 1)-:NUM_PINS] = registers[i * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS];
			end
			assign gen_intr = ((registers[INTR_POSEDGE_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] & pos_edge) | (registers[INTR_NEGEDGE_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] & neg_edge)) & registers[INTR_EN_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS];
			always @(posedge clk or negedge n_rst)
				if (~n_rst)
					registers[INTR_STAT_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= {_param_6B795_NUM_PINS {1'sb0}};
				else
					registers[INTR_STAT_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= (registers[INTR_STAT_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] & ~registers[INTR_CLR_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS]) | gen_intr;
			always @(posedge clk or negedge n_rst)
				if (~n_rst)
					registers[INTR_CLR_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= {_param_6B795_NUM_PINS {1'sb0}};
				else if (wen[INTR_CLR_IND])
					registers[INTR_CLR_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= {_param_6B795_NUM_PINS {1'sb1}};
				else
					registers[INTR_CLR_IND * _param_6B795_NUM_PINS+:_param_6B795_NUM_PINS] <= {_param_6B795_NUM_PINS {1'sb0}};
		end
		assign Gpio0.clk = clk;
		assign Gpio0.n_rst = rst_n;
	endgenerate
	assign gpio0apbIf.PWDATA = ahb2apbIf_apbif.PWDATA;
	assign gpio0apbIf.PADDR = ahb2apbIf_apbif.PADDR;
	assign gpio0apbIf.PWRITE = ahb2apbIf_apbif.PWRITE;
	assign gpio0apbIf.PENABLE = ahb2apbIf_apbif.PENABLE;
	assign gpio0apbIf.PSEL = ahb2apbIf.PSEL_slave[GPIO0_APB_IDX];
	assign ahb2apbIf.PRData_slave[0+:32] = gpio0apbIf.PRDATA;
	wire [PWM_PIN_COUNT:0] pwm_out;
	pwm #(.NUM_CHANNELS(PWM_PIN_COUNT)) Pwm0(
		.clk(clk),
		.n_rst(rst_n),
		.paddr(pwm0If.PADDR),
		.pwdata(pwm0If.PWDATA),
		.psel(pwm0If.PSEL),
		.penable(pwm0If.PENABLE),
		.pwrite(pwm0If.PWRITE),
		.prdata(pwm0If.PRDATA),
		.pwm_out(pwm_out)
	);
	assign pwm0If.PWDATA = ahb2apbIf_apbif.PWDATA;
	assign pwm0If.PADDR = ahb2apbIf_apbif.PADDR;
	assign pwm0If.PWRITE = ahb2apbIf_apbif.PWRITE;
	assign pwm0If.PENABLE = ahb2apbIf_apbif.PENABLE;
	assign pwm0If.PSEL = ahb2apbIf.PSEL_slave[PWM0_APB_IDX];
	assign ahb2apbIf.PRData_slave[32+:32] = pwm0If.PRDATA;
	generate
		localparam signed [31:0] _param_F45AC_NUM_CHANNELS = TIMER_PIN_COUNT;
		if (1) begin : Timer0
			localparam NUM_CHANNELS = _param_F45AC_NUM_CHANNELS;
			wire HCLK;
			wire n_RST;
			wire tim_clk;
			wire tim_overflow;
			wire [_param_F45AC_NUM_CHANNELS - 1:0] edge_detected;
			wire [_param_F45AC_NUM_CHANNELS - 1:0] compare_success;
			reg [_param_F45AC_NUM_CHANNELS - 1:0] compare_output;
			localparam IOS_IND = 0;
			localparam TCF_IND = 1;
			localparam TCNT_IND = 2;
			localparam TSCR_IND = 3;
			localparam TOV_IND = 4;
			localparam TCR_IND = 5;
			localparam TIE_IND = 6;
			localparam TSCR2_IND = 7;
			localparam FLG1_IND = 8;
			localparam FLG2_IND = 9;
			localparam TC0_IND = 10;
			localparam TC1_IND = 11;
			localparam TC2_IND = 12;
			localparam TC3_IND = 13;
			localparam TC4_IND = 14;
			localparam TC5_IND = 15;
			localparam TC6_IND = 16;
			localparam TC7_IND = 17;
			localparam NUM_REGISTERS = 18;
			localparam NUM_TCn = 1;
			wire [31:0] apb_data;
			wire [575:0] apb_read;
			wire [17:0] wen;
			wire [_param_F45AC_NUM_CHANNELS - 1:0] success;
			reg [31:0] IOS;
			reg [31:0] TCF;
			reg [31:0] TCNT;
			reg [31:0] TSCR;
			reg [31:0] TOV;
			reg [31:0] TCR;
			reg [31:0] TIE;
			reg [31:0] TSCR2;
			reg [31:0] FLG1;
			reg [47:0] FLG2;
			reg [255:0] TCN;
			APB_SlaveInterface_timer #(
				.NUM_REGS(NUM_REGISTERS),
				.ADDR_OFFSET(0)
			) apbs(
				.clk(HCLK),
				.n_rst(n_RST),
				.PADDR(top_level_bASIC.apb_timer0If.PADDR),
				.PWDATA(top_level_bASIC.apb_timer0If.PWDATA),
				.PENABLE(top_level_bASIC.apb_timer0If.PENABLE),
				.PWRITE(top_level_bASIC.apb_timer0If.PWRITE),
				.PSEL(top_level_bASIC.apb_timer0If.PSEL),
				.PRDATA(top_level_bASIC.apb_timer0If.PRDATA),
				.read_data(apb_read),
				.w_enable(wen),
				.w_data(apb_data)
			);
			always @(posedge HCLK or negedge n_RST)
				if (~n_RST) begin
					IOS <= {32 {1'sb0}};
					TSCR <= {32 {1'sb0}};
					TOV <= {32 {1'sb0}};
					TCR <= {32 {1'sb0}};
					TIE <= {32 {1'sb0}};
					TSCR2 <= {32 {1'sb0}};
				end
				else if (wen[IOS_IND])
					IOS <= apb_data[31:0];
				else if (wen[TSCR_IND])
					TSCR <= apb_data[31:0];
				else if (wen[TOV_IND])
					TOV <= apb_data[31:0];
				else if (wen[TCR_IND])
					TCR <= apb_data[31:0];
				else if (wen[TIE_IND])
					TIE <= apb_data[31:0];
				else if (wen[TSCR2_IND])
					TSCR2 <= apb_data[31:0];
			always @(posedge HCLK or negedge n_RST)
				if (~n_RST)
					TCF <= {32 {1'sb0}};
				else if (wen[TCF_IND])
					TCF <= apb_data[31:0];
				else
					TCF <= {32 {1'sb0}};
			always @(posedge HCLK or negedge n_RST)
				if (~n_RST) begin
					FLG1 <= {32 {1'sb0}};
					FLG2 <= {48 {1'sb0}};
				end
				else if (wen[FLG1_IND])
					FLG1 <= FLG1 & ~apb_data;
				else if (wen[FLG2_IND])
					FLG2 <= FLG2 & ~apb_data;
				else begin
					FLG1[7-:8] <= ((edge_detected & ~IOS[7-:8]) | (compare_success & IOS[7-:8])) | FLG1[7-:8];
					FLG2[23] <= tim_overflow | FLG2[23];
				end
			genvar i;
			for (i = 0; i < NUM_TCn; i = i + 1) begin : TCn_write_slave_regs
				always @(posedge HCLK or negedge n_RST)
					if (~n_RST)
						TCN[i * 32+:32] <= {32 {1'sb0}};
					else if (wen[17 + i] & IOS[i])
						TCN[i * 32+:32] <= apb_data[31:0];
					else if (!IOS[i] & edge_detected[i])
						TCN[i * 32+:32] <= TCNT;
			end
			assign apb_read[0+:32] = IOS;
			assign apb_read[32+:32] = {32 {1'sb0}};
			assign apb_read[64+:32] = TCNT;
			assign apb_read[96+:32] = TSCR;
			assign apb_read[128+:32] = TOV;
			assign apb_read[160+:32] = TCR;
			assign apb_read[192+:32] = TIE;
			assign apb_read[224+:32] = TSCR2;
			assign apb_read[256+:32] = FLG1;
			assign apb_read[288+:32] = FLG2;
			for (i = 0; i < NUM_TCn; i = i + 1) begin : generate_r_data_TCN
				assign apb_read[(17 + i) * 32+:32] = TCN[i * 32+:32];
			end
			clock_divider clkdiv(
				.HCLK(HCLK),
				.n_RST(n_RST),
				.PRE(TSCR2[2-:3]),
				.tim_clk(tim_clk)
			);
			reg [_param_F45AC_NUM_CHANNELS - 1:0] r_signal_synch;
			always @(posedge HCLK or negedge n_RST)
				if (~n_RST)
					r_signal_synch <= 0;
				else
					r_signal_synch <= top_level_bASIC.pin_timer0If.r_data;
			edge_detector_timer edge_detect(
				.clk(HCLK),
				.n_rst(n_RST),
				.signal(r_signal_synch),
				.EDGEnA(TCR[15-:8]),
				.EDGEnB(TCR[7-:8]),
				.edge_detected(edge_detected)
			);
			for (i = 0; i < NUM_CHANNELS; i = i + 1) begin : compare_succ_gen
				assign compare_success[i] = TCNT == TCN[i * 32+:32];
			end
			for (i = 0; i < NUM_CHANNELS; i = i + 1) begin : compare_output_gen
				assign success[i] = compare_success[i] | TCF[i];
				always @(posedge HCLK or negedge n_RST)
					if (!n_RST)
						compare_output[i] <= 1'b0;
					else if (TOV[i] & tim_overflow)
						compare_output[i] <= ~compare_output[i];
					else if ((!TCR[24 + i] & TCR[16 + i]) & success[i])
						compare_output[i] <= ~compare_output[i];
					else if ((TCR[24 + i] & !TCR[16 + i]) & success[i])
						compare_output[i] <= 0;
					else if ((TCR[24 + i] & TCR[16 + i]) & success[i])
						compare_output[i] <= 1;
			end
			for (i = 0; i < NUM_CHANNELS; i = i + 1) begin : output_enable_gen
				assign top_level_bASIC.pin_timer0If.output_en[i] = IOS[i] && (TCR[24 + i] || TCR[16 + i]);
			end
			assign top_level_bASIC.pin_timer0If.timer_bidir = (top_level_bASIC.pin_timer0If.output_en ? top_level_bASIC.pin_timer0If.w_data : 1'bz);
			always @(posedge HCLK) begin
				top_level_bASIC.pin_timer0If.r_data <= top_level_bASIC.pin_timer0If.timer_bidir;
				top_level_bASIC.pin_timer0If.w_data <= compare_output;
			end
			always @(posedge tim_clk or negedge n_RST)
				if (!n_RST)
					TCNT <= {32 {1'sb0}};
				else if (TSCR2[6] & success[NUM_CHANNELS - 1])
					TCNT <= {32 {1'sb0}};
				else if (wen[TCNT_IND])
					TCNT <= apb_data[31:0];
				else if (TSCR[7])
					TCNT <= TCNT + 1;
			assign tim_overflow = ((TCNT == 32'hffffffff) && TSCR[7]) && ~TSCR2[6];
			reg [_param_F45AC_NUM_CHANNELS - 1:0] channel_irq_r;
			reg tim_irq_r;
			always @(posedge HCLK or negedge n_RST)
				if (~n_RST) begin
					channel_irq_r <= {_param_F45AC_NUM_CHANNELS {1'sb0}};
					tim_irq_r <= 1'b0;
				end
				else begin
					channel_irq_r <= FLG1[7-:8];
					tim_irq_r <= FLG2[23];
				end
		end
		assign Timer0.HCLK = clk;
		assign Timer0.n_RST = rst_n;
	endgenerate
	assign apb_timer0If.PWDATA = ahb2apbIf_apbif.PWDATA;
	assign apb_timer0If.PADDR = ahb2apbIf_apbif.PADDR;
	assign apb_timer0If.PWRITE = ahb2apbIf_apbif.PWRITE;
	assign apb_timer0If.PENABLE = ahb2apbIf_apbif.PENABLE;
	assign apb_timer0If.PSEL = ahb2apbIf.PSEL_slave[TIMER0_APB_IDX];
	assign ahb2apbIf.PRData_slave[64+:32] = apb_timer0If.PRDATA;
	wire master_active;
	reg SDA_write;
	reg SCL_write;
	localparam [0:0] MASTER = 0;
	localparam [0:0] SLAVE = 1;
	generate
		if (1) begin : I2c0
			wire clk;
			wire n_rst;
			wire master_active;
			wire master_intermediate;
			if (1) begin : bus
				wire [7:0] tx_data;
				wire [7:0] rx_data_slave;
				wire [7:0] rx_data_master;
				reg [7:0] rx_data;
				wire address_mode;
				wire data_direction;
				wire ms_select;
				wire [9:0] bus_address;
				wire [5:0] packet_size;
				wire en_clock_strech;
				wire TX_fifo_empty;
				wire RX_fifo_full;
				wire RX_fifo_almost_full;
				wire transaction_begin;
				wire [31:0] clk_divider;
				wire ack_error_set_slave;
				wire ack_error_set_master;
				reg ack_error_set;
				wire transaction_begin_clear;
				wire busy_slave;
				wire busy_master;
				reg busy;
				wire TX_read_enable_slave;
				wire TX_read_enable_master;
				reg TX_read_enable;
				wire RX_write_enable_slave;
				wire RX_write_enable_master;
				reg RX_write_enable;
				wire line_busy;
				reg set_transaction_complete;
				wire set_transaction_complete_slave;
				wire set_transaction_complete_master;
				wire set_arbitration_lost;
				wire SDA_sync;
				wire SCL_sync;
				wire SDA_out_slave;
				wire SDA_out_master;
				wire SCL_out_slave;
				wire SCL_out_master;
			end
			sync_high SDA_SYNC(
				.clk(clk),
				.n_rst(n_rst),
				.async_in(top_level_bASIC.i2c.SDA),
				.sync_out(bus.SDA_sync)
			);
			sync_high SCL_SYNC(
				.clk(clk),
				.n_rst(n_rst),
				.async_in(top_level_bASIC.i2c.SCL),
				.sync_out(bus.SCL_sync)
			);
			assign master_intermediate = bus.ms_select;
			assign master_active = (master_intermediate == MASTER ? 1'b1 : 1'b0);
			if (1) begin : REGISTER
				wire clk;
				wire n_rst;
				wire interrupt;
				apbSlave IX(
					.pclk(clk),
					.n_rst(n_rst),
					.pdata(top_level_bASIC.apb_i2c0If.PWDATA),
					.paddr(top_level_bASIC.apb_i2c0If.PADDR),
					.penable(top_level_bASIC.apb_i2c0If.PENABLE),
					.psel(top_level_bASIC.apb_i2c0If.PSEL),
					.pwrite(top_level_bASIC.apb_i2c0If.PWRITE),
					.rx_data(top_level_bASIC.I2c0.bus.rx_data),
					.rx_w_ena(top_level_bASIC.I2c0.bus.RX_write_enable),
					.i2c_status({top_level_bASIC.I2c0.bus.transaction_begin_clear, top_level_bASIC.I2c0.bus.busy, top_level_bASIC.I2c0.bus.set_arbitration_lost, top_level_bASIC.I2c0.bus.set_transaction_complete, top_level_bASIC.I2c0.bus.line_busy, top_level_bASIC.I2c0.bus.ack_error_set}),
					.scl(clk),
					.tx_r_ena(top_level_bASIC.I2c0.bus.TX_read_enable),
					.prdata(top_level_bASIC.apb_i2c0If.PRDATA),
					.i2c_interrupt(interrupt),
					.tx_data(top_level_bASIC.I2c0.bus.tx_data),
					.rx_full(top_level_bASIC.I2c0.bus.RX_fifo_full),
					.rx_almost_full(top_level_bASIC.I2c0.bus.RX_fifo_almost_full),
					.control_out({top_level_bASIC.I2c0.bus.en_clock_strech, top_level_bASIC.I2c0.bus.transaction_begin, top_level_bASIC.I2c0.bus.data_direction, top_level_bASIC.I2c0.bus.packet_size[5:0], top_level_bASIC.I2c0.bus.ms_select, top_level_bASIC.I2c0.bus.address_mode}),
					.address(top_level_bASIC.I2c0.bus.bus_address),
					.clk_out(top_level_bASIC.I2c0.bus.clk_divider),
					.tx_empty(top_level_bASIC.I2c0.bus.TX_fifo_empty)
				);
			end
			assign REGISTER.clk = clk;
			assign REGISTER.n_rst = n_rst;
			assign top_level_bASIC.i2c.interrupt = REGISTER.interrupt;
			if (1) begin : SLAVE
				wire clk;
				wire n_rst;
				slave_inner SLAVE_INNER(
					.clk(clk),
					.n_rst(n_rst),
					.tx_data(top_level_bASIC.I2c0.bus.tx_data),
					.address_mode(top_level_bASIC.I2c0.bus.address_mode),
					.ms_select(top_level_bASIC.I2c0.bus.ms_select),
					.bus_address(top_level_bASIC.I2c0.bus.bus_address),
					.en_clock_strech(top_level_bASIC.I2c0.bus.en_clock_strech),
					.TX_fifo_empty(top_level_bASIC.I2c0.bus.TX_fifo_empty),
					.RX_fifo_full(top_level_bASIC.I2c0.bus.RX_fifo_full),
					.RX_fifo_almost_full(top_level_bASIC.I2c0.bus.RX_fifo_almost_full),
					.SDA_sync(top_level_bASIC.I2c0.bus.SDA_sync),
					.SCL_sync(top_level_bASIC.I2c0.bus.SCL_sync),
					.rx_data_slave(top_level_bASIC.I2c0.bus.rx_data_slave),
					.set_transaction_complete_slave(top_level_bASIC.I2c0.bus.set_transaction_complete_slave),
					.ack_error_set_slave(top_level_bASIC.I2c0.bus.ack_error_set_slave),
					.busy_slave(top_level_bASIC.I2c0.bus.busy_slave),
					.TX_read_enable_slave(top_level_bASIC.I2c0.bus.TX_read_enable_slave),
					.RX_write_enable_slave(top_level_bASIC.I2c0.bus.RX_write_enable_slave),
					.SDA_out_slave(top_level_bASIC.I2c0.bus.SDA_out_slave),
					.SCL_out_slave(top_level_bASIC.I2c0.bus.SCL_out_slave)
				);
			end
			assign SLAVE.clk = clk;
			assign SLAVE.n_rst = n_rst;
			if (1) begin : MASTER
				wire clk;
				wire n_rst;
				wire abort;
				wire ack_bit;
				wire ack_gen;
				wire bus_busy;
				wire byte_complete;
				wire decrement_byte_counter;
				wire load_buffers;
				wire output_wait_expired;
				wire one;
				wire shift_load;
				wire shift_strobe;
				wire should_nack;
				wire timer_active;
				wire zero;
				wire shift_direction;
				wire [1:0] output_select;
				wire [1:0] shift_input_select;
				wire SDA_sync;
				wire SCL_sync;
				wire SDA_out;
				wire SCL_out;
				assign SDA_sync = top_level_bASIC.I2c0.bus.SDA_sync;
				assign SCL_sync = top_level_bASIC.I2c0.bus.SCL_sync;
				wire SDA_out_shift_register;
				wire SDA_out_start_stop_gen;
				wire SCL_out_timer;
				wire SCL_out_start_stop_gen;
				wire [9:0] bus_address;
				wire [31:0] clock_div;
				wire clock_stretch_enabled;
				wire data_direction;
				wire address_mode;
				assign should_nack = (!top_level_bASIC.I2c0.bus.en_clock_strech && top_level_bASIC.I2c0.bus.RX_fifo_almost_full) || zero;
				assign top_level_bASIC.I2c0.bus.line_busy = bus_busy;
				assign top_level_bASIC.I2c0.bus.SDA_out_master = SDA_out;
				assign top_level_bASIC.I2c0.bus.SCL_out_master = SCL_out;
				wire start;
				wire stop;
				master_controller MASTER_CONTROLLER(
					.clk(clk),
					.n_rst(n_rst),
					.address_mode(address_mode),
					.ms_select(top_level_bASIC.I2c0.bus.ms_select),
					.bus_busy(bus_busy),
					.begin_transaction_flag(top_level_bASIC.I2c0.bus.transaction_begin),
					.ack_bit(ack_bit),
					.data_direction(data_direction),
					.output_wait_expired(output_wait_expired),
					.byte_complete(byte_complete),
					.zero_bytes_left(zero),
					.abort(abort),
					.stretch_enabled(clock_stretch_enabled),
					.rx_fifo_full(top_level_bASIC.I2c0.bus.RX_fifo_full),
					.tx_fifo_empty(top_level_bASIC.I2c0.bus.TX_fifo_empty),
					.shift_input_select(shift_input_select),
					.output_select(output_select),
					.shift_direction(shift_direction),
					.shift_load(shift_load),
					.timer_active(timer_active),
					.load_buffers(load_buffers),
					.decrement_byte_counter(decrement_byte_counter),
					.set_ack_error(top_level_bASIC.I2c0.bus.ack_error_set_master),
					.set_arbitration_lost(top_level_bASIC.I2c0.bus.set_arbitration_lost),
					.clear_transaction_begin(top_level_bASIC.I2c0.bus.transaction_begin_clear),
					.start(start),
					.stop(stop),
					.tx_fifo_enable(top_level_bASIC.I2c0.bus.TX_read_enable_master),
					.rx_fifo_enable(top_level_bASIC.I2c0.bus.RX_write_enable_master),
					.busy(top_level_bASIC.I2c0.bus.busy_master),
					.set_transaction_complete(top_level_bASIC.I2c0.bus.set_transaction_complete_master)
				);
				master_timer TIMER(
					.clk(clk),
					.n_rst(n_rst),
					.timer_active(timer_active),
					.direction(shift_direction),
					.should_nack(should_nack),
					.SDA_sync(SDA_sync),
					.SCL_sync(SCL_sync),
					.SDA_out(SDA_out_shift_register),
					.clock_div(clock_div),
					.SCL_out(SCL_out_timer),
					.shift_strobe(shift_strobe),
					.byte_complete(byte_complete),
					.ack_gen(ack_gen),
					.ack(ack_bit),
					.abort(abort)
				);
				control_buffer CONTROL_BUFFER(
					.clk(clk),
					.n_rst(n_rst),
					.u_bus_address(top_level_bASIC.I2c0.bus.bus_address),
					.u_data_direction(top_level_bASIC.I2c0.bus.data_direction),
					.u_address_mode(top_level_bASIC.I2c0.bus.address_mode),
					.u_clock_div(top_level_bASIC.I2c0.bus.clk_divider),
					.u_stretch_enabled(top_level_bASIC.I2c0.bus.en_clock_strech),
					.load_buffer(load_buffers),
					.bus_address(bus_address),
					.data_direction(data_direction),
					.address_mode(address_mode),
					.stretch_enabled(clock_stretch_enabled),
					.clock_div(clock_div)
				);
				shift_register SHIFT_REGISTER(
					.clk(clk),
					.n_rst(n_rst),
					.bus_address(bus_address),
					.tx_data(top_level_bASIC.I2c0.bus.tx_data),
					.shift_input_select(shift_input_select),
					.data_direction(data_direction),
					.shift_direction(shift_direction),
					.shift_in(SDA_sync),
					.shift_load(shift_load),
					.shift_strobe(shift_strobe),
					.shift_out(SDA_out_shift_register),
					.data_out(top_level_bASIC.I2c0.bus.rx_data_master)
				);
				start_stop_gen START_STOP_GEN(
					.clk(clk),
					.n_rst(n_rst),
					.start(start),
					.stop(stop),
					.clock_div(clock_div),
					.bus_busy(bus_busy),
					.SDA(SDA_out_start_stop_gen),
					.SCL(SCL_out_start_stop_gen),
					.done(output_wait_expired)
				);
				output_mux OUTPUT_MUX(
					.drive_select(output_select),
					.tx_SDA(SDA_out_shift_register),
					.tx_SCL(SCL_out_timer),
					.rx_SDA(!ack_gen),
					.rx_SCL(SCL_out_timer),
					.start_stop_SDA(SDA_out_start_stop_gen),
					.start_stop_SCL(SCL_out_start_stop_gen),
					.SDA_out(SDA_out),
					.SCL_out(SCL_out)
				);
				busy_tester BUSY_TESTER(
					.clk(clk),
					.n_rst(n_rst),
					.SDA_sync(SDA_sync),
					.SCL_sync(SCL_sync),
					.bus_busy(bus_busy)
				);
				byte_counter BYTE_COUNTER(
					.clk(clk),
					.n_rst(n_rst),
					.decrement(decrement_byte_counter),
					.load_buffer(load_buffers),
					.packet_length(top_level_bASIC.I2c0.bus.packet_size[5:0]),
					.zero(zero),
					.one(one)
				);
			end
			assign MASTER.clk = clk;
			assign MASTER.n_rst = n_rst;
			reg SDA_out_next;
			reg SCL_out_next;
			always @(posedge clk or negedge n_rst)
				if (!n_rst) begin
					top_level_bASIC.i2c.SDA_out = 1;
					top_level_bASIC.i2c.SCL_out = 1;
				end
				else begin
					top_level_bASIC.i2c.SDA_out = SDA_out_next;
					top_level_bASIC.i2c.SCL_out = SCL_out_next;
				end
			always @(*)
				if (bus.ms_select == 0) begin
					bus.rx_data = bus.rx_data_master;
					bus.set_transaction_complete = bus.set_transaction_complete_master;
					bus.ack_error_set = bus.ack_error_set_master;
					bus.busy = bus.busy_master;
					bus.TX_read_enable = bus.TX_read_enable_master;
					bus.RX_write_enable = bus.RX_write_enable_master;
					SDA_out_next = bus.SDA_out_master;
					SCL_out_next = bus.SCL_out_master;
				end
				else begin
					bus.rx_data = bus.rx_data_slave;
					bus.set_transaction_complete = bus.set_transaction_complete_slave;
					bus.ack_error_set = bus.ack_error_set_slave;
					bus.busy = bus.busy_slave;
					bus.TX_read_enable = bus.TX_read_enable_slave;
					bus.RX_write_enable = bus.RX_write_enable_slave;
					SDA_out_next = bus.SDA_out_slave;
					SCL_out_next = bus.SCL_out_slave;
				end
		end
		assign I2c0.clk = clk;
		assign I2c0.n_rst = rst_n;
		assign master_active = I2c0.master_active;
	endgenerate
	assign SDA_bi = (master_active ? SDA_write : 1'bz);
	assign SCL_bi = (master_active ? SCL_write : 1'bz);
	always @(posedge clk) begin
		SDA_write <= i2c.SDA_out;
		SCL_write <= i2c.SCL_out;
		if (~master_active) begin
			i2c.SDA <= SDA_bi;
			i2c.SCL <= SCL_bi;
		end
		else begin
			i2c.SDA <= 1'b1;
			i2c.SCL <= 1'b1;
		end
	end
	assign apb_i2c0If.PWDATA = ahb2apbIf_apbif.PWDATA;
	assign apb_i2c0If.PADDR = ahb2apbIf_apbif.PADDR;
	assign apb_i2c0If.PWRITE = ahb2apbIf_apbif.PWRITE;
	assign apb_i2c0If.PENABLE = ahb2apbIf_apbif.PENABLE;
	assign apb_i2c0If.PSEL = ahb2apbIf.PSEL_slave[I2C0_APB_IDX];
	assign ahb2apbIf.PRData_slave[128+:32] = apb_timer0If.PRDATA;
	reg MOSI_write;
	reg MISO_write;
	reg SS_write;
	reg SCK_write;
	generate
		if (1) begin : SPI0
			wire CLK;
			wire nRST;
			localparam CONTROL_REG = 0;
			localparam BUFFER_LEN_REG = 1;
			localparam BAUD_REG = 2;
			localparam TX_DATA_REG = 3;
			localparam RX_DATA_REG = 4;
			localparam STATUS_REG = 5;
			reg [31:0] control_reg;
			reg [31:0] control_reg_n;
			reg [31:0] status_reg;
			reg [31:0] status_reg_n;
			reg [31:0] baud_rate_reg;
			reg [31:0] baud_rate_reg_n;
			reg [31:0] byte_length_reg;
			reg [31:0] byte_length_reg_n;
			reg EN;
			wire nEN;
			localparam spi_type_pkg_NUM_REGS = 6;
			wire [5:0] r_enable;
			wire [5:0] w_enable;
			wire [5:0] r_en;
			wire [5:0] w_en;
			wire buffer_full;
			wire buffer_empty;
			wire buffer_half;
			wire OPC;
			wire MODF_flag;
			wire interrupt_out;
			wire [5:0] interrupt_clear;
			wire SCLK;
			wire SCK_unedged;
			wire SCLK_OUT;
			wire SS_OUT;
			wire shift_enable;
			wire load_enable;
			wire serial_in;
			wire serial_out;
			wire [7:0] parallel_in;
			wire [7:0] parallel_out;
			wire count_enable;
			wire cnt_clear;
			wire w_done;
			wire shifter_rst;
			wire Tx_REN;
			wire Rx_WEN;
			wire fifo_WEN;
			wire fifo_REN;
			wire [7:0] fifo_wdata;
			wire [7:0] fifo_rx_rdata;
			wire [3:0] buff_tx_numdata;
			wire [3:0] buff_rx_numdata;
			wire sck_in_posedge;
			wire sck_in_negedge;
			wire sck_out_posedge;
			wire sck_out_negedge;
			reg retime;
			reg sample;
			reg data_advance;
			wire MOSI_IN_sync;
			wire MISO_IN_sync;
			wire SS_IN_sync;
			wire SCK_IN_sync;
			localparam spi_type_pkg_WORD_W = 32;
			wire [31:0] apb_data;
			APB_SlaveInterface #(
				.NUM_REGS(spi_type_pkg_NUM_REGS),
				.ADDR_OFFSET(0)
			) apb(
				.clk(CLK),
				.n_rst(nRST),
				.PADDR(top_level_bASIC.apb_spi0If.PADDR),
				.PWDATA(top_level_bASIC.apb_spi0If.PWDATA),
				.PENABLE(top_level_bASIC.apb_spi0If.PENABLE),
				.PWRITE(top_level_bASIC.apb_spi0If.PWRITE),
				.PSEL(top_level_bASIC.apb_spi0If.PSEL),
				.PRDATA(top_level_bASIC.apb_spi0If.PRDATA),
				.read_data({status_reg, 24'b000000000000000000000000, fifo_rx_rdata, 32'b00000000000000000000000000000000, baud_rate_reg, byte_length_reg, control_reg}),
				.w_enable(w_enable),
				.r_enable(r_enable),
				.w_data(apb_data)
			);
			wire buffer_rx_full;
			wire buffer_tx_empty;
			wire buffer_rx_half;
			wire buffer_tx_half;
			interrupt_control IC(
				.CLK(CLK),
				.nRST(nRST),
				.rx_full(buffer_rx_full),
				.tx_empty(buffer_tx_empty),
				.rx_water(buffer_rx_half),
				.tx_water(buffer_tx_half),
				.transfer_complete(OPC),
				.modf(status_reg[26]),
				.clear(interrupt_clear),
				.interrupt_enable(control_reg[14-:6]),
				.interrupts(top_level_bASIC.spiif.interrupts)
			);
			clock_generator CG(
				.CLK(CLK),
				.nRST(nRST),
				.enable(control_reg[30] & count_enable),
				.polarity(control_reg[29]),
				.divider(baud_rate_reg + 32'd1),
				.SCK(SCLK)
			);
			edge_detect_spi POSEDGE_OUT(
				.clk(CLK),
				.n_rst(nRST),
				.edge_mode(1'b1),
				.d_plus(SCLK),
				.d_edge(sck_out_posedge)
			);
			edge_detect_spi NEGEDGE_OUT(
				.clk(CLK),
				.n_rst(nRST),
				.edge_mode(1'b0),
				.d_plus(SCLK),
				.d_edge(sck_out_negedge)
			);
			edge_detect_spi POSEDGE_IN(
				.clk(CLK),
				.n_rst(nRST),
				.edge_mode(1'b1),
				.d_plus(SCK_IN_sync),
				.d_edge(sck_in_posedge)
			);
			edge_detect_spi NEGEDGE_IN(
				.clk(CLK),
				.n_rst(nRST),
				.edge_mode(1'b0),
				.d_plus(SCK_IN_sync),
				.d_edge(sck_in_negedge)
			);
			always @(*) begin
				sample = 0;
				data_advance = 0;
				casez ({control_reg[29], control_reg[28]})
					0: begin
						sample = (control_reg[30] ? sck_out_posedge : sck_in_posedge);
						data_advance = (control_reg[30] ? sck_out_negedge : sck_in_negedge);
					end
					1: begin
						sample = (control_reg[30] ? sck_out_negedge : sck_in_negedge);
						data_advance = (control_reg[30] ? sck_out_posedge : sck_in_posedge);
					end
					2: begin
						sample = (control_reg[30] ? sck_out_negedge : sck_in_negedge);
						data_advance = (control_reg[30] ? sck_out_posedge : sck_in_posedge);
					end
					3: begin
						sample = (control_reg[30] ? sck_out_posedge : sck_in_posedge);
						data_advance = (control_reg[30] ? sck_out_negedge : sck_in_negedge);
					end
				endcase
			end
			wire shifter_en;
			wire TX_REN;
			flex_sr_spi #(.NUM_BITS(8)) SR(
				.clk(CLK),
				.n_rst(nRST),
				.shift_msb(control_reg[26]),
				.shift_enable(shifter_en),
				.shift_clk(sample),
				.shift_clear(shifter_rst),
				.load_enable(TX_REN),
				.mode(1'b0),
				.serial_in(serial_in),
				.parallel_in(parallel_in),
				.parallel_out(parallel_out),
				.serial_out(serial_out)
			);
			counters CNT(
				.clk(CLK),
				.n_rst(nRST),
				.clear(cnt_clear),
				.count_enable(count_enable),
				.count_clk(sample),
				.bytelen(byte_length_reg),
				.OPC(OPC),
				.w_done(w_done)
			);
			wire RX_WEN;
			controller CTRL(
				.CLK(CLK),
				.nRST(nRST),
				.mode(control_reg[30]),
				.EN(EN),
				.op_complete(OPC),
				.w_done(w_done),
				.SS_IN(SS_IN_sync),
				.SS_OUT(SS_OUT),
				.shifter_en(shifter_en),
				.shifter_load(load_enable),
				.counter_en(count_enable),
				.shifter_rst(shifter_rst),
				.counter_rst(cnt_clear),
				.TX_REN(TX_REN),
				.RX_WEN(RX_WEN)
			);
			wire buffer_tx_full;
			fifo #(
				.NUM_BITS(8),
				.buffer_size(32),
				.addr_bits(6),
				.WATER_DIR(1)
			) FIFO_TX(
				.CLK(CLK),
				.nRST(nRST),
				.WEN(w_en[TX_DATA_REG]),
				.REN(TX_REN),
				.wdata(apb_data[7:0]),
				.rdata(parallel_in),
				.numdata(buff_tx_numdata),
				.empty(buffer_tx_empty),
				.full(buffer_tx_full),
				.half(buffer_tx_half),
				.watermark((control_reg[19-:5] == 0 ? 5'd16 : control_reg[19-:5]))
			);
			wire buffer_rx_empty;
			fifo #(
				.NUM_BITS(8),
				.buffer_size(32),
				.addr_bits(6),
				.WATER_DIR(0)
			) FIFO_RX(
				.CLK(CLK),
				.nRST(nRST),
				.WEN(RX_WEN),
				.REN(r_en[RX_DATA_REG]),
				.wdata(parallel_out),
				.rdata(fifo_rx_rdata),
				.numdata(buff_rx_numdata),
				.empty(buffer_rx_empty),
				.full(buffer_rx_full),
				.half(buffer_rx_half),
				.watermark((control_reg[24-:5] == 0 ? 5'd16 : control_reg[24-:5]))
			);
			assign r_en = r_enable;
			assign w_en = w_enable;
			assign interrupt_clear = (w_enable[5] ? apb_data[5:0] : {6 {1'sb0}});
			always @(*) begin : REG_UPD
				control_reg_n = control_reg;
				status_reg_n = status_reg;
				byte_length_reg_n = byte_length_reg;
				baud_rate_reg_n = baud_rate_reg;
				if (EN)
					control_reg_n[31] = 1'b0;
				casez (w_enable)
					6'b000001: control_reg_n = apb_data;
					6'b000010: byte_length_reg_n = apb_data;
					6'b000100: baud_rate_reg_n = apb_data;
				endcase
				status_reg_n[25-:5] = buff_rx_numdata;
				status_reg_n[20-:5] = buff_tx_numdata;
				status_reg_n[28] = buffer_rx_full;
				status_reg_n[27] = buffer_tx_empty;
				status_reg_n[30] = buffer_rx_half;
				status_reg_n[29] = buffer_tx_half;
				if (((control_reg[30] == 1'b0) && control_reg[25]) && control_reg[27])
					status_reg_n[26] = 1'b1;
				else
					status_reg_n[26] = 1'b0;
				if (OPC)
					status_reg_n[31] = 1'b1;
				else if (r_enable[STATUS_REG])
					status_reg_n[31] = 1'b0;
			end
			always @(posedge CLK or negedge nRST)
				if (nRST == 0) begin
					control_reg <= 32'b00000000000000000000000000000000;
					byte_length_reg <= {32 {1'sb0}};
					baud_rate_reg <= {32 {1'sb0}};
					status_reg <= {32 {1'sb0}};
				end
				else begin
					control_reg <= control_reg_n;
					byte_length_reg <= byte_length_reg_n;
					baud_rate_reg <= baud_rate_reg_n;
					status_reg <= status_reg_n;
				end
			localparam spi_type_pkg_SPE = 31;
			always @(posedge CLK or negedge nRST)
				if (nRST == 0)
					EN <= 0;
				else if (OPC)
					EN <= 0;
				else if ((w_en[CONTROL_REG] && (apb_data[spi_type_pkg_SPE] == 1)) && (status_reg[26] == 0))
					EN <= 1;
				else if (status_reg[26] == 1)
					EN <= 0;
			always @(posedge CLK or negedge nRST)
				if (!nRST)
					retime <= 1'b0;
				else if (data_advance || control_reg[31])
					retime <= serial_out;
				else
					retime <= retime;
			sync_high SYNC_SCK(
				.clk(CLK),
				.n_rst(nRST),
				.async_in(top_level_bASIC.spiif.SCK_IN),
				.sync_out(SCK_IN_sync)
			);
			sync_high SYNC_SS(
				.clk(CLK),
				.n_rst(nRST),
				.async_in(top_level_bASIC.spiif.SS_IN),
				.sync_out(SS_IN_sync)
			);
			sync_high SYNC_MISO(
				.clk(CLK),
				.n_rst(nRST),
				.async_in(top_level_bASIC.spiif.MISO_IN),
				.sync_out(MISO_IN_sync)
			);
			sync_high SYNC_MOSI(
				.clk(CLK),
				.n_rst(nRST),
				.async_in(top_level_bASIC.spiif.MOSI_IN),
				.sync_out(MOSI_IN_sync)
			);
			assign top_level_bASIC.spiif.SS_OUT = (control_reg[27] ? SS_OUT : 1);
			assign top_level_bASIC.spiif.mode = control_reg[30];
			assign top_level_bASIC.spiif.MOSI_OUT = retime;
			assign top_level_bASIC.spiif.SCK_OUT = SCLK;
			assign top_level_bASIC.spiif.MISO_OUT = retime;
			assign serial_in = (control_reg[30] ? MISO_IN_sync : MOSI_IN_sync);
		end
		assign SPI0.CLK = clk;
		assign SPI0.nRST = rst_n;
	endgenerate
	assign apb_spi0If.PWDATA = ahb2apbIf_apbif.PWDATA;
	assign apb_spi0If.PADDR = ahb2apbIf_apbif.PADDR;
	assign apb_spi0If.PWRITE = ahb2apbIf_apbif.PWRITE;
	assign apb_spi0If.PENABLE = ahb2apbIf_apbif.PENABLE;
	assign apb_spi0If.PSEL = ahb2apbIf.PSEL_slave[SPI0_APB_IDX];
	assign ahb2apbIf.PRData_slave[96+:32] = apb_spi0If.PRDATA;
	assign SS_bi = (spiif.mode ? SS_write : 1'bz);
	assign SCK_bi = (spiif.mode ? SCK_write : 1'bz);
	assign MOSI_bi = (spiif.mode ? MOSI_write : 1'bz);
	assign MISO_bi = (spiif.mode ? 1'bz : MISO_write);
	always @(posedge clk or negedge rst_n)
		if (!rst_n)
			la_data_out <= {128 {1'sb0}};
		else
			la_data_out <= la_data_nxt;
	always @(posedge clk) begin
		MOSI_write <= spiif.MOSI_OUT;
		MISO_write <= spiif.MISO_OUT;
		SS_write <= spiif.SS_OUT;
		SCK_write <= spiif.SCK_OUT;
		if (~spiif.mode) begin
			spiif.MOSI_IN <= MOSI_bi;
			spiif.MISO_IN <= 1'b1;
			spiif.SS_IN <= SS_bi;
			spiif.SCK_IN <= SCK_bi;
		end
		else begin
			spiif.MOSI_IN <= 1'b1;
			spiif.MISO_IN <= MISO_bi;
			spiif.SS_IN <= 1'b1;
			spiif.SCK_IN <= 1'b1;
		end
	end
	localparam [31:0] READ = 2;
	localparam [31:0] WRITE = 1;
	generate
		localparam [31:0] _param_BADD8_SRAM_DEPTH = 'h7ffff;
		localparam signed [31:0] _param_BADD8_N_SRAM = 1;
		if (1) begin : sram_controller
			wire HCLK;
			wire HRESETn;
			localparam N_SRAM = _param_BADD8_N_SRAM;
			localparam INVERT_CE_EN = 0;
			localparam INVERT_BYTE_EN = 0;
			localparam SRAM_WIDTH = 4;
			localparam SRAM_DEPTH = _param_BADD8_SRAM_DEPTH;
			localparam SIZE_WORD = 2'h2;
			localparam SIZE_HALF_WORD = 2'h1;
			localparam SIZE_QUARTER_WORD = 2'h0;
			reg [3:0] quarter_word;
			reg [3:0] half_word;
			reg [3:0] word;
			reg [3:0] byte_en_in;
			reg [3:0] latched_quarter_word;
			reg [3:0] latched_half_word;
			reg [3:0] latched_word;
			reg [3:0] latched_byte_en_in;
			always @(*) begin
				quarter_word = {4 {1'sb0}};
				half_word = {4 {1'sb0}};
				word = {4 {1'sb0}};
				byte_en_in = {4 {1'sb0}};
				mcif.sram_en = 2'b01;
				latched_quarter_word = {4 {1'sb0}};
				latched_half_word = {4 {1'sb0}};
				latched_word = {4 {1'sb0}};
				latched_byte_en_in = {4 {1'sb0}};
				case (mcif.addr[1:0])
					2'h0: begin
						quarter_word = 4'h1;
						word = 4'hf;
						half_word = 4'h3;
					end
					2'h1: quarter_word = 4'h2;
					2'h2: begin
						quarter_word = 4'h4;
						half_word = 4'hc;
					end
					2'h3: quarter_word = 4'h8;
				endcase
				case (mcif.size)
					SIZE_QUARTER_WORD: byte_en_in = quarter_word;
					SIZE_HALF_WORD: byte_en_in = half_word;
					SIZE_WORD: byte_en_in = word;
				endcase
				case (mcif.latched_addr[1:0])
					2'h0: begin
						latched_quarter_word = 4'h1;
						latched_half_word = 4'h3;
						latched_word = 4'hf;
					end
					2'h1: latched_quarter_word = 4'h2;
					2'h2: begin
						latched_quarter_word = 4'h4;
						latched_half_word = 4'hc;
					end
					2'h3: latched_quarter_word = 4'h8;
				endcase
				case (mcif.latched_size)
					SIZE_QUARTER_WORD: latched_byte_en_in = latched_quarter_word;
					SIZE_HALF_WORD: latched_byte_en_in = latched_half_word;
					SIZE_WORD: latched_byte_en_in = latched_word;
				endcase
				mcif.byte_en = byte_en_in ^ mcif.INVERT_BYTE_EN;
				mcif.latched_byte_en = latched_byte_en_in ^ mcif.INVERT_BYTE_EN;
				begin : sv2v_autoblock_28
					reg signed [31:0] sram_num;
					for (sram_num = 0; sram_num < mcif.N_SRAM; sram_num = sram_num + 1)
						if (($unsigned(mcif.addr) >= $unsigned((sram_num * mcif.SRAM_DEPTH) * mcif.SRAM_WIDTH)) && ($unsigned(mcif.addr) < $unsigned(((sram_num + 1) * mcif.SRAM_DEPTH) * mcif.SRAM_WIDTH)))
							mcif.sram_en = (2'b01 << sram_num) ^ mcif.INVERT_CE_EN;
				end
			end
			reg [31:0] addr_reg;
			localparam signed [31:0] _param_FF6BD_N_SRAM = N_SRAM;
			localparam signed [31:0] _param_FF6BD_INVERT_CE_EN = INVERT_CE_EN;
			localparam signed [31:0] _param_FF6BD_INVERT_BYTE_EN = INVERT_BYTE_EN;
			localparam signed [31:0] _param_FF6BD_SRAM_WIDTH = SRAM_WIDTH;
			localparam [31:0] _param_FF6BD_SRAM_DEPTH = SRAM_DEPTH;
			if (1) begin : mcif
				localparam N_SRAM = _param_FF6BD_N_SRAM;
				localparam INVERT_CE_EN = _param_FF6BD_INVERT_CE_EN;
				localparam INVERT_BYTE_EN = _param_FF6BD_INVERT_BYTE_EN;
				localparam SRAM_WIDTH = _param_FF6BD_SRAM_WIDTH;
				localparam SRAM_DEPTH = _param_FF6BD_SRAM_DEPTH;
				wire HCLK;
				wire HRESETn;
				wire HWRITE;
				wire HSEL;
				wire HREADY;
				wire HRESP;
				wire [1:0] HTRANS;
				wire [2:0] HSIZE;
				wire [31:0] HADDR;
				wire [31:0] HWDATA;
				wire [31:0] HRDATA;
				reg [31:0] ram_wData;
				wire [31:0] ram_rData;
				wire [3:0] byte_en;
				wire [0:0] sram_en;
				reg wen;
				wire [31:0] ram_addr;
				wire sram_wait;
				wire [31:0] addr;
				reg [1:0] size;
				wire [1:0] latched_size;
				wire [1:0] read_size;
				wire [31:0] rData;
				wire [31:0] latched_data;
				wire [31:0] latched_addr;
				wire [3:0] latched_byte_en;
				wire latched_flag;
				wire [31:0] read_addr;
				wire [31:0] RAO_w_data;
				wire [31:0] RAO_r_data;
				wire [31:0] RAO_RAM_val;
				wire [11:0] RAO_addr;
				wire [11:0] RAO_RAM_addr;
				wire RAO_w_en;
			end
			localparam signed [31:0] _param_CC3E2_N_SRAM = N_SRAM;
			localparam signed [31:0] _param_CC3E2_INVERT_CE_EN = INVERT_CE_EN;
			if (1) begin : encoder
				localparam N_SRAM = _param_CC3E2_N_SRAM;
				localparam INVERT_CE_EN = _param_CC3E2_INVERT_CE_EN;
				localparam SIZE_WORD = 2'h2;
				localparam SIZE_HALF_WORD = 2'h1;
				localparam SIZE_QUARTER_WORD = 2'h0;
				wire [31:0] rData_in;
				wire [31:0] word;
				wire [15:0] half_word;
				wire [7:0] quarter_word;
				wire [3:0] byte_sel;
				wire [_param_CC3E2_N_SRAM:0] rData_sel;
				wire [3:0] lcv;
				wire [31:0] bit_mask;
				assign rData_sel = top_level_bASIC.sram_controller.mcif.sram_en ^ INVERT_CE_EN;
				assign lcv = top_level_bASIC.sram_controller.mcif.latched_byte_en ^ INVERT_CE_EN;
				assign bit_mask = {{8 {lcv[3]}}, {8 {lcv[2]}}, {8 {lcv[1]}}, {8 {lcv[0]}}};
				assign top_level_bASIC.sram_controller.mcif.rData = top_level_bASIC.sram_controller.mcif.ram_rData[0+:32];
			end
			localparam [31:0] _param_2EE2D_NUM_BYTES = 2097148;
			if (1) begin : sram_ahb_slave
				wire ram_en;
				localparam BASE_ADDRESS = 0;
				localparam NUM_BYTES = _param_2EE2D_NUM_BYTES;
				wire [31:0] sl_wdata;
				wire [31:0] sl_addr;
				reg [31:0] last_addr;
				wire sl_rprep;
				wire sl_wprep;
				wire sl_wen;
				wire sl_ren;
				wire [4:0] sl_bcount;
				wire [2:0] sl_btype;
				wire [2:0] size;
				reg [2:0] last_size;
				reg [31:0] addr;
				reg latched_flag;
				reg [2:0] latched_size;
				reg [31:0] latched_addr;
				reg [31:0] latched_data;
				reg [31:0] current;
				reg [31:0] next;
				wire latch_en;
				reg latch_clr;
				wire wen;
				reg slave_wait;
				wire next_slave_wait;
				wire n_rst;
				wire clk;
				assign clk = top_level_bASIC.sram_controller.mcif.HCLK;
				assign n_rst = top_level_bASIC.sram_controller.mcif.HRESETn;
				assign top_level_bASIC.sram_controller.mcif.ram_addr = addr & 32'hfffffffc;
				assign top_level_bASIC.sram_controller.mcif.addr = addr;
				ahb_slave #(
					.BASE_ADDRESS(BASE_ADDRESS),
					.NUMBER_ADDRESSES(NUM_BYTES)
				) AHBS0(
					.HCLK(top_level_bASIC.sram_controller.mcif.HCLK),
					.HRESETn(top_level_bASIC.sram_controller.mcif.HRESETn),
					.HMASTLOCK(1'b0),
					.HWRITE(top_level_bASIC.sram_controller.mcif.HWRITE),
					.HSEL(top_level_bASIC.sram_controller.mcif.HSEL),
					.HREADYIN(1'b1),
					.HADDR(top_level_bASIC.sram_controller.mcif.HADDR),
					.HWDATA(top_level_bASIC.sram_controller.mcif.HWDATA),
					.HTRANS(top_level_bASIC.sram_controller.mcif.HTRANS),
					.HBURST(3'b000),
					.HSIZE(top_level_bASIC.sram_controller.mcif.HSIZE),
					.HPROT(4'b0000),
					.HRDATA(top_level_bASIC.sram_controller.mcif.HRDATA),
					.HREADYOUT(top_level_bASIC.sram_controller.mcif.HREADY),
					.HRESP(top_level_bASIC.sram_controller.mcif.HRESP),
					.burst_cancel(1'b0),
					.slave_wait(top_level_bASIC.sram_controller.mcif.sram_wait),
					.rdata(top_level_bASIC.sram_controller.mcif.rData),
					.wdata(sl_wdata),
					.addr(sl_addr),
					.r_prep(sl_rprep),
					.w_prep(sl_wprep),
					.wen(sl_wen),
					.ren(sl_ren),
					.size(size),
					.burst_count(sl_bcount),
					.burst_type(sl_btype)
				);
				reg last_wen;
				reg nwen;
				always @(*)
					if (sl_rprep && !top_level_bASIC.sram_controller.mcif.sram_wait)
						next = READ;
					else if (sl_wprep && !top_level_bASIC.sram_controller.mcif.sram_wait)
						next = WRITE;
					else if (sl_wen && !top_level_bASIC.sram_controller.mcif.sram_wait)
						next = IDLE;
					else if (sl_ren && !top_level_bASIC.sram_controller.mcif.sram_wait)
						next = IDLE;
					else
						next = current;
				always @(posedge clk or negedge n_rst)
					if (~n_rst) begin
						current <= IDLE;
						slave_wait <= 1'b0;
						last_wen <= 1'b0;
					end
					else begin
						current <= next;
						slave_wait <= top_level_bASIC.sram_controller.mcif.sram_wait;
						last_wen <= nwen;
					end
				assign wen = (current == WRITE ? sl_wen : 1'b0);
				assign latch_en = (current == WRITE ? sl_rprep : 1'b0);
				assign ram_en = (((sl_rprep || sl_wprep) && ((top_level_bASIC.sram_controller.mcif.HTRANS == 2'b10) || (top_level_bASIC.sram_controller.mcif.HTRANS == 2'b11))) || latch_clr ? 1'b1 : 1'b0);
				always @(posedge clk or negedge n_rst)
					if (~n_rst) begin
						last_size <= {3 {1'sb0}};
						last_addr <= {32 {1'sb0}};
					end
					else if (sl_wprep | sl_rprep) begin
						last_size <= size;
						last_addr <= sl_addr;
					end
				always @(posedge clk or negedge n_rst)
					if (~n_rst) begin
						latched_flag <= 1'b0;
						latched_size <= {3 {1'sb0}};
						latched_addr <= {32 {1'sb0}};
						latched_data <= {32 {1'sb0}};
					end
					else if (latch_en) begin
						latched_flag <= 1'b1;
						latched_size <= last_size;
						latched_addr <= last_addr;
						latched_data <= sl_wdata;
					end
					else if (latch_clr)
						latched_flag <= 1'b0;
				assign top_level_bASIC.sram_controller.mcif.latched_data = latched_data;
				assign top_level_bASIC.sram_controller.mcif.latched_addr = latched_addr;
				assign top_level_bASIC.sram_controller.mcif.latched_flag = latched_flag;
				assign top_level_bASIC.sram_controller.mcif.latched_size = latched_size;
				assign top_level_bASIC.sram_controller.mcif.read_size = last_size;
				assign top_level_bASIC.sram_controller.mcif.read_addr = last_addr;
				always @(*) begin
					top_level_bASIC.sram_controller.mcif.wen = 1'b0;
					nwen = 1'b0;
					top_level_bASIC.sram_controller.mcif.ram_wData = {32 {1'sb0}};
					addr = 32'hbad1bad1;
					top_level_bASIC.sram_controller.mcif.size = {2 {1'sb0}};
					latch_clr = 1'b0;
					if (latched_flag) begin
						top_level_bASIC.sram_controller.mcif.ram_wData = latched_data;
						if (sl_rprep || ((current == READ) && !sl_wprep)) begin
							top_level_bASIC.sram_controller.mcif.wen = 1'b0;
							nwen = 1'b0;
							latch_clr = 1'b0;
							addr = sl_addr;
							top_level_bASIC.sram_controller.mcif.size = size;
						end
						else begin
							top_level_bASIC.sram_controller.mcif.wen = 1'b1;
							nwen = 1'b1;
							latch_clr = 1'b1;
							addr = latched_addr;
							top_level_bASIC.sram_controller.mcif.size = latched_size;
						end
					end
					else if (sl_rprep || ((current == READ) && !sl_wprep)) begin
						top_level_bASIC.sram_controller.mcif.wen = 1'b0;
						nwen = 1'b0;
						addr = sl_addr;
						top_level_bASIC.sram_controller.mcif.size = size;
					end
					else begin
						top_level_bASIC.sram_controller.mcif.wen = wen;
						nwen = wen;
						top_level_bASIC.sram_controller.mcif.ram_wData = sl_wdata;
						addr = last_addr;
						top_level_bASIC.sram_controller.mcif.size = last_size;
						latch_clr = 1'b0;
					end
				end
			end
			assign top_level_bASIC.sramIf.ram_en = sram_ahb_slave.ram_en;
			always @(posedge HCLK or negedge HRESETn)
				if (HRESETn == 1'b0)
					addr_reg <= 32'd30;
				else
					addr_reg <= mcif.ram_addr;
			assign mcif.HCLK = HCLK;
			assign mcif.HRESETn = HRESETn;
			assign mcif.HWRITE = top_level_bASIC.sramIf_ahbif.HWRITE;
			assign mcif.HSEL = top_level_bASIC.sramIf_ahbif.HSEL;
			assign mcif.HTRANS = top_level_bASIC.sramIf_ahbif.HTRANS;
			assign mcif.HSIZE = top_level_bASIC.sramIf_ahbif.HSIZE;
			assign mcif.HADDR = top_level_bASIC.sramIf_ahbif.HADDR;
			assign mcif.HWDATA = top_level_bASIC.sramIf_ahbif.HWDATA;
			assign mcif.ram_rData = top_level_bASIC.sramIf.ram_rData;
			assign mcif.sram_wait = top_level_bASIC.sramIf.sram_wait;
			assign top_level_bASIC.sramIf_ahbif.HREADYOUT = mcif.HREADY;
			assign top_level_bASIC.sramIf.wen = mcif.wen;
			assign top_level_bASIC.sramIf_ahbif.HRESP = mcif.HRESP;
			assign top_level_bASIC.sramIf_ahbif.HRDATA = mcif.HRDATA;
			assign top_level_bASIC.sramIf.ram_wData = mcif.ram_wData;
			assign top_level_bASIC.sramIf.addr = mcif.ram_addr;
			assign top_level_bASIC.sramIf.byte_en = mcif.byte_en;
		end
		assign sram_controller.HCLK = clk;
		assign sram_controller.HRESETn = rst_n;
	endgenerate
	generate
		if (1) begin : mem_blocks
			wire clk;
			wire nRST;
			wire [31:0] rdata_swapped;
			reg [31:0] rdata_mux;
			wire [31:0] wdata_reg;
			reg [31:0] rom_rdata_reg;
			reg [31:0] ram_rdata_reg;
			reg [31:0] rom_active_reg;
			reg [31:0] ram_active_reg;
			reg [31:0] sram_active_reg;
			reg sram_en_delay;
			reg wait_delay;
			reg wen_delayed;
			always @(posedge clk or negedge nRST)
				if (!nRST) begin
					rom_rdata_reg <= {32 {1'sb1}};
					ram_rdata_reg <= {32 {1'sb1}};
					rom_active_reg <= {32 {1'sb0}};
					ram_active_reg <= {32 {1'sb0}};
					sram_active_reg <= {32 {1'sb0}};
					sram_en_delay <= 1'b0;
					wait_delay <= 1'b0;
					wen_delayed <= 1'b0;
				end
				else begin
					rom_rdata_reg <= top_level_bASIC.blkif.rom_rdata;
					ram_rdata_reg <= top_level_bASIC.blkif.ram_rdata;
					rom_active_reg <= top_level_bASIC.blkif.rom_active;
					ram_active_reg <= top_level_bASIC.blkif.ram_active;
					sram_active_reg <= top_level_bASIC.blkif.sram_active;
					sram_en_delay <= top_level_bASIC.sramIf.ram_en;
					wait_delay <= sram_en_delay && top_level_bASIC.blkif.ram_wait;
					wen_delayed <= top_level_bASIC.sramIf.wen;
				end
			assign top_level_bASIC.blkif.addr = top_level_bASIC.sramIf.addr;
			assign top_level_bASIC.blkif.wen = top_level_bASIC.sramIf.wen;
			assign top_level_bASIC.blkif.ram_en = top_level_bASIC.sramIf.ram_en;
			assign top_level_bASIC.blkif.byte_en = {top_level_bASIC.sramIf.byte_en[0], top_level_bASIC.sramIf.byte_en[1], top_level_bASIC.sramIf.byte_en[2], top_level_bASIC.sramIf.byte_en[3]};
			assign wdata_reg = top_level_bASIC.sramIf.ram_wData;
			assign top_level_bASIC.sramIf.ram_rData[0+:32] = rdata_swapped;
			assign top_level_bASIC.sramIf.sram_wait = top_level_bASIC.blkif.ram_wait;
			assign top_level_bASIC.blkif.rom_wait = top_level_bASIC.blkif.ram_wait;
			always @(*) begin
				rdata_mux = ram_rdata_reg;
				if (rom_active_reg)
					rdata_mux = rom_rdata_reg;
			end
			endian_swapper_ram write_swap(
				.word_in(wdata_reg),
				.word_out(top_level_bASIC.blkif.wdata)
			);
			endian_swapper read_swap(
				.word_in(rdata_mux),
				.word_out(rdata_swapped)
			);
		end
		assign mem_blocks.clk = clk;
		assign mem_blocks.nRST = rst_n;
	endgenerate
	generate
		if (1) begin : offchip_sram
			wire clk;
			wire nRST;
			localparam ADDR_BOTTOM = 32'h00008400;
			localparam ADDR_TOP = 32'h00200000;
			localparam DELAY_MAX = 0;
			wire active;
			assign active = (top_level_bASIC.blkif.addr >= ADDR_BOTTOM) && (top_level_bASIC.blkif.addr <= ADDR_TOP);
			assign top_level_bASIC.blkif.sram_active = active;
			reg [3:0] wait_count;
			reg [31:0] old_addr;
			reg [31:0] old_wdata;
			reg old_wen;
			always @(posedge clk or negedge nRST)
				if (!nRST) begin
					wait_count <= 0;
					old_addr <= {32 {1'sb1}};
					old_wdata <= 32'hbad1bad1;
					old_wen <= 0;
				end
				else begin
					wait_count <= wait_count;
					if (wait_count > 0)
						wait_count <= wait_count - 1;
					if (((old_addr != top_level_bASIC.blkif.addr) || (old_wdata != top_level_bASIC.blkif.wdata)) || (old_wen != top_level_bASIC.blkif.wen))
						wait_count <= DELAY_MAX;
					old_addr <= top_level_bASIC.blkif.addr;
					old_wdata <= top_level_bASIC.blkif.wdata;
					old_wen <= top_level_bASIC.blkif.wen;
				end
			assign top_level_bASIC.blkif.sram_wait = wait_count != 0;
			always @(*) begin : offchip_sram_controls
				top_level_bASIC.offchip_sramif.nWE = 4'b1111;
				top_level_bASIC.offchip_sramif.nCE = 1'b0;
				top_level_bASIC.offchip_sramif.nOE = 1'b1;
				if (active) begin
					top_level_bASIC.offchip_sramif.nOE = 1'b0;
					if (top_level_bASIC.blkif.wen) begin
						top_level_bASIC.offchip_sramif.nOE = 1'b1;
						top_level_bASIC.offchip_sramif.nWE = ~top_level_bASIC.blkif.byte_en;
					end
				end
			end
			always @(*) begin : offchip_sram_data
				top_level_bASIC.offchip_sramif.external_addr = top_level_bASIC.blkif.addr >> 2;
			end
			assign top_level_bASIC.offchip_sramif.external_bidir[7:0] = (!top_level_bASIC.offchip_sramif.nWE[0] ? top_level_bASIC.blkif.wdata[7:0] : 8'bzzzzzzzz);
			assign top_level_bASIC.offchip_sramif.external_bidir[15:8] = (!top_level_bASIC.offchip_sramif.nWE[1] ? top_level_bASIC.blkif.wdata[15:8] : 8'bzzzzzzzz);
			assign top_level_bASIC.offchip_sramif.external_bidir[23:16] = (!top_level_bASIC.offchip_sramif.nWE[2] ? top_level_bASIC.blkif.wdata[23:16] : 8'bzzzzzzzz);
			assign top_level_bASIC.offchip_sramif.external_bidir[31:24] = (!top_level_bASIC.offchip_sramif.nWE[3] ? top_level_bASIC.blkif.wdata[31:24] : 8'bzzzzzzzz);
			always @(posedge clk)
				if (top_level_bASIC.blkif.wen)
					top_level_bASIC.blkif.sram_rdata <= {32 {1'sb0}};
				else begin
					top_level_bASIC.blkif.sram_rdata[7:0] <= top_level_bASIC.offchip_sramif.external_bidir[7:0];
					top_level_bASIC.blkif.sram_rdata[15:8] <= top_level_bASIC.offchip_sramif.external_bidir[15:8];
					top_level_bASIC.blkif.sram_rdata[23:16] <= top_level_bASIC.offchip_sramif.external_bidir[23:16];
					top_level_bASIC.blkif.sram_rdata[31:24] <= top_level_bASIC.offchip_sramif.external_bidir[31:24];
				end
			assign top_level_bASIC.offchip_sramif.WE = ~top_level_bASIC.offchip_sramif.nWE;
		end
		assign offchip_sram.clk = clk;
		assign offchip_sram.nRST = rst_n;
	endgenerate
	generate
		if (1) begin : ROM
			wire clk;
			wire nRST;
			localparam ROM_TOP = 16'h2000;
			localparam ROM_BOTTOM = 16'h003b;
			wire [31:0] addr;
			reg rom_wait;
			reg rom_later;
			wire nrom_wait;
			assign addr = ((top_level_bASIC.blkif.addr < 32'h00008000) && (top_level_bASIC.blkif.addr >= 32'h00000200) ? top_level_bASIC.blkif.addr >> 2 : {32 {1'sb0}});
			assign top_level_bASIC.blkif.rom_active = ((top_level_bASIC.blkif.addr < 32'h00008000) && (addr >= ROM_BOTTOM)) && !top_level_bASIC.blkif.rom_wait;
			assign nrom_wait = ((top_level_bASIC.blkif.addr < 32'h00008000) && (top_level_bASIC.blkif.addr >= 32'h00000200) ? 1'b1 : 1'b0);
			always @(posedge clk or negedge nRST)
				if (!nRST) begin
					rom_wait <= 1'b0;
					rom_later <= 1'b0;
				end
				else begin
					rom_wait <= nrom_wait;
					rom_later <= rom_wait;
				end
			always @(*)
				case (addr)
					16'h0080: top_level_bASIC.blkif.rom_rdata <= 32'h23200000;
					16'h0081: top_level_bASIC.blkif.rom_rdata <= 32'hb7000080;
					16'h0082: top_level_bASIC.blkif.rom_rdata <= 32'h93808000;
					16'h0083: top_level_bASIC.blkif.rom_rdata <= 32'h23a00000;
					16'h0084: top_level_bASIC.blkif.rom_rdata <= 32'hb7000080;
					16'h0085: top_level_bASIC.blkif.rom_rdata <= 32'h93804000;
					16'h0086: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h0087: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h0088: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h0089: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h008a: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h008b: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h008c: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h008d: top_level_bASIC.blkif.rom_rdata <= 32'h13000000;
					16'h008e: top_level_bASIC.blkif.rom_rdata <= 32'h83210000;
					16'h008f: top_level_bASIC.blkif.rom_rdata <= 32'he38e01fe;
					16'h0090: top_level_bASIC.blkif.rom_rdata <= 32'hb7000080;
					16'h0091: top_level_bASIC.blkif.rom_rdata <= 32'h93808000;
					16'h0092: top_level_bASIC.blkif.rom_rdata <= 32'h1301f00f;
					16'h0093: top_level_bASIC.blkif.rom_rdata <= 32'h23a02000;
					16'h0094: top_level_bASIC.blkif.rom_rdata <= 32'h67820100;
					16'h0095: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0096: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0097: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0098: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0099: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h009f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h00ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0100: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0101: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0102: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0103: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0104: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0105: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0106: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0107: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0108: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0109: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h010f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0110: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0111: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0112: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0113: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0114: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0115: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0116: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0117: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0118: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0119: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h011f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0120: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0121: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0122: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0123: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0124: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0125: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0126: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0127: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0128: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0129: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h012f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0130: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0131: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0132: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0133: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0134: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0135: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0136: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0137: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0138: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0139: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h013f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0140: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0141: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0142: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0143: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0144: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0145: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0146: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0147: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0148: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0149: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h014f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0150: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0151: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0152: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0153: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0154: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0155: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0156: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0157: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0158: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0159: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h015f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0160: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0161: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0162: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0163: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0164: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0165: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0166: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0167: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0168: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0169: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h016f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0170: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0171: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0172: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0173: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0174: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0175: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0176: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0177: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0178: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0179: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h017f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0180: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0181: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0182: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0183: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0184: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0185: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0186: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0187: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0188: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0189: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h018f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0190: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0191: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0192: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0193: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0194: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0195: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0196: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0197: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0198: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0199: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h019f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h01ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0200: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0201: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0202: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0203: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0204: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0205: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0206: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0207: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0208: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0209: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h020f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0210: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0211: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0212: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0213: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0214: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0215: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0216: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0217: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0218: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0219: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h021f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0220: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0221: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0222: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0223: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0224: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0225: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0226: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0227: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0228: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0229: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h022f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0230: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0231: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0232: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0233: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0234: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0235: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0236: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0237: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0238: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0239: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h023f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0240: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0241: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0242: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0243: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0244: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0245: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0246: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0247: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0248: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0249: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h024f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0250: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0251: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0252: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0253: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0254: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0255: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0256: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0257: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0258: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0259: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h025f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0260: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0261: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0262: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0263: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0264: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0265: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0266: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0267: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0268: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0269: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h026f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0270: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0271: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0272: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0273: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0274: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0275: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0276: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0277: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0278: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0279: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h027f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0280: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0281: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0282: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0283: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0284: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0285: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0286: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0287: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0288: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0289: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h028f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0290: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0291: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0292: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0293: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0294: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0295: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0296: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0297: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0298: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0299: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h029f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h02ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0300: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0301: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0302: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0303: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0304: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0305: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0306: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0307: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0308: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0309: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h030f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0310: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0311: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0312: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0313: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0314: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0315: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0316: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0317: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0318: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0319: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h031f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0320: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0321: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0322: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0323: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0324: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0325: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0326: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0327: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0328: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0329: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h032f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0330: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0331: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0332: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0333: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0334: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0335: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0336: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0337: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0338: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0339: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h033f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0340: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0341: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0342: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0343: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0344: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0345: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0346: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0347: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0348: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0349: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h034f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0350: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0351: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0352: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0353: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0354: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0355: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0356: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0357: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0358: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0359: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h035f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0360: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0361: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0362: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0363: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0364: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0365: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0366: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0367: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0368: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0369: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h036f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0370: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0371: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0372: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0373: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0374: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0375: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0376: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0377: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0378: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0379: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h037f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0380: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0381: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0382: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0383: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0384: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0385: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0386: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0387: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0388: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0389: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h038f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0390: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0391: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0392: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0393: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0394: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0395: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0396: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0397: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0398: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0399: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h039f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h03ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0400: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0401: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0402: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0403: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0404: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0405: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0406: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0407: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0408: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0409: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h040f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0410: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0411: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0412: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0413: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0414: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0415: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0416: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0417: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0418: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0419: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h041f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0420: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0421: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0422: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0423: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0424: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0425: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0426: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0427: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0428: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0429: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h042f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0430: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0431: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0432: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0433: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0434: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0435: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0436: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0437: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0438: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0439: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h043f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0440: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0441: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0442: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0443: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0444: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0445: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0446: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0447: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0448: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0449: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h044f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0450: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0451: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0452: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0453: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0454: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0455: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0456: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0457: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0458: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0459: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h045f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0460: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0461: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0462: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0463: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0464: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0465: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0466: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0467: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0468: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0469: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h046f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0470: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0471: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0472: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0473: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0474: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0475: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0476: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0477: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0478: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0479: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h047f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0480: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0481: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0482: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0483: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0484: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0485: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0486: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0487: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0488: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0489: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h048f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0490: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0491: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0492: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0493: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0494: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0495: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0496: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0497: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0498: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0499: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h049f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h04ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0500: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0501: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0502: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0503: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0504: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0505: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0506: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0507: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0508: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0509: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h050f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0510: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0511: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0512: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0513: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0514: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0515: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0516: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0517: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0518: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0519: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h051f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0520: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0521: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0522: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0523: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0524: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0525: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0526: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0527: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0528: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0529: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h052f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0530: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0531: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0532: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0533: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0534: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0535: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0536: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0537: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0538: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0539: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h053f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0540: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0541: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0542: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0543: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0544: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0545: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0546: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0547: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0548: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0549: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h054f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0550: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0551: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0552: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0553: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0554: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0555: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0556: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0557: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0558: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0559: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h055f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0560: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0561: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0562: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0563: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0564: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0565: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0566: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0567: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0568: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0569: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h056f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0570: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0571: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0572: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0573: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0574: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0575: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0576: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0577: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0578: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0579: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h057f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0580: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0581: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0582: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0583: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0584: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0585: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0586: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0587: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0588: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0589: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h058f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0590: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0591: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0592: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0593: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0594: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0595: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0596: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0597: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0598: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0599: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h059f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h05ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0600: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0601: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0602: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0603: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0604: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0605: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0606: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0607: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0608: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0609: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h060f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0610: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0611: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0612: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0613: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0614: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0615: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0616: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0617: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0618: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0619: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h061f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0620: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0621: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0622: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0623: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0624: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0625: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0626: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0627: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0628: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0629: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h062f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0630: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0631: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0632: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0633: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0634: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0635: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0636: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0637: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0638: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0639: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h063f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0640: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0641: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0642: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0643: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0644: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0645: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0646: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0647: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0648: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0649: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h064f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0650: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0651: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0652: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0653: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0654: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0655: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0656: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0657: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0658: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0659: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h065f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0660: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0661: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0662: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0663: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0664: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0665: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0666: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0667: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0668: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0669: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h066f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0670: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0671: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0672: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0673: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0674: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0675: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0676: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0677: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0678: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0679: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h067f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0680: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0681: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0682: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0683: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0684: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0685: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0686: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0687: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0688: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0689: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h068f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0690: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0691: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0692: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0693: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0694: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0695: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0696: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0697: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0698: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0699: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h069f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h06ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0700: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0701: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0702: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0703: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0704: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0705: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0706: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0707: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0708: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0709: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h070f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0710: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0711: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0712: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0713: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0714: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0715: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0716: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0717: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0718: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0719: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h071f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0720: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0721: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0722: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0723: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0724: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0725: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0726: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0727: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0728: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0729: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h072f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0730: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0731: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0732: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0733: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0734: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0735: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0736: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0737: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0738: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0739: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h073f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0740: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0741: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0742: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0743: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0744: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0745: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0746: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0747: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0748: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0749: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h074f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0750: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0751: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0752: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0753: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0754: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0755: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0756: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0757: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0758: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0759: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h075f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0760: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0761: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0762: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0763: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0764: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0765: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0766: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0767: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0768: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0769: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h076f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0770: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0771: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0772: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0773: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0774: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0775: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0776: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0777: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0778: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0779: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h077f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0780: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0781: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0782: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0783: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0784: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0785: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0786: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0787: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0788: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0789: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h078f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0790: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0791: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0792: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0793: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0794: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0795: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0796: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0797: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0798: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0799: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h079f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h07ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0800: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0801: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0802: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0803: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0804: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0805: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0806: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0807: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0808: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0809: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h080f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0810: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0811: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0812: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0813: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0814: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0815: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0816: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0817: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0818: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0819: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h081f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0820: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0821: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0822: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0823: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0824: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0825: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0826: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0827: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0828: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0829: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h082f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0830: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0831: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0832: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0833: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0834: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0835: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0836: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0837: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0838: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0839: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h083f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0840: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0841: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0842: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0843: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0844: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0845: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0846: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0847: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0848: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0849: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h084f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0850: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0851: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0852: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0853: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0854: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0855: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0856: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0857: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0858: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0859: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h085f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0860: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0861: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0862: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0863: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0864: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0865: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0866: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0867: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0868: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0869: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h086f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0870: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0871: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0872: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0873: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0874: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0875: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0876: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0877: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0878: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0879: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h087f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0880: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0881: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0882: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0883: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0884: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0885: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0886: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0887: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0888: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0889: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h088f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0890: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0891: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0892: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0893: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0894: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0895: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0896: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0897: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0898: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0899: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h089f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h08ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0900: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0901: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0902: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0903: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0904: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0905: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0906: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0907: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0908: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0909: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h090f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0910: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0911: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0912: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0913: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0914: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0915: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0916: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0917: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0918: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0919: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h091f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0920: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0921: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0922: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0923: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0924: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0925: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0926: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0927: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0928: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0929: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h092f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0930: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0931: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0932: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0933: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0934: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0935: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0936: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0937: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0938: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0939: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h093f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0940: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0941: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0942: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0943: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0944: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0945: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0946: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0947: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0948: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0949: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h094f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0950: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0951: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0952: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0953: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0954: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0955: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0956: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0957: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0958: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0959: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h095f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0960: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0961: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0962: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0963: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0964: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0965: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0966: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0967: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0968: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0969: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h096f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0970: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0971: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0972: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0973: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0974: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0975: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0976: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0977: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0978: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0979: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h097f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0980: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0981: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0982: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0983: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0984: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0985: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0986: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0987: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0988: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0989: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h098f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0990: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0991: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0992: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0993: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0994: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0995: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0996: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0997: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0998: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0999: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h099f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h09ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0a9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aa9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aaa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aaf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ab9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0abb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0abc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0abd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0abe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0abf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ac9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0acb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0acc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0acd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ace: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0acf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ad9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ada: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0adb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0adc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0add: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ade: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0adf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ae9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aeb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0af9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0afa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0afb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0afc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0afd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0afe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0aff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0b9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ba9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0baa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0baf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0be9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0beb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bf9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0bff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0c9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ca9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0caa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0caf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ccb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ccc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ccd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ccf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ce9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ceb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ced: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cf9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0cff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0d9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0da9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0daa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0daf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0db9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ddb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ddc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ddd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ddf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0de9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0deb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ded: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0def: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0df9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0dff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0e9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ea9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eaa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ead: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eaf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ebb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ebc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ebd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ebe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ebf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ec9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ecb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ecc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ecd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ece: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ecf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ed9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0edb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0edc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0edd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ede: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0edf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ee9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eeb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ef9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0efa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0efb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0efc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0efd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0efe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0eff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0f9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fa9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0faa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0faf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fe9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0feb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ff9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ffa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ffb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ffc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ffd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0ffe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h0fff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1000: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1001: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1002: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1003: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1004: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1005: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1006: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1007: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1008: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1009: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h100f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1010: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1011: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1012: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1013: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1014: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1015: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1016: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1017: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1018: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1019: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h101f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1020: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1021: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1022: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1023: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1024: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1025: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1026: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1027: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1028: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1029: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h102f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1030: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1031: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1032: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1033: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1034: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1035: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1036: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1037: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1038: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1039: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h103f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1040: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1041: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1042: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1043: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1044: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1045: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1046: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1047: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1048: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1049: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h104f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1050: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1051: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1052: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1053: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1054: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1055: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1056: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1057: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1058: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1059: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h105f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1060: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1061: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1062: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1063: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1064: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1065: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1066: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1067: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1068: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1069: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h106f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1070: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1071: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1072: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1073: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1074: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1075: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1076: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1077: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1078: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1079: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h107f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1080: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1081: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1082: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1083: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1084: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1085: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1086: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1087: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1088: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1089: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h108f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1090: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1091: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1092: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1093: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1094: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1095: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1096: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1097: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1098: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1099: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h109f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h10ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1100: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1101: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1102: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1103: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1104: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1105: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1106: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1107: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1108: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1109: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h110f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1110: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1111: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1112: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1113: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1114: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1115: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1116: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1117: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1118: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1119: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h111f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1120: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1121: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1122: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1123: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1124: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1125: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1126: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1127: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1128: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1129: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h112f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1130: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1131: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1132: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1133: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1134: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1135: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1136: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1137: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1138: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1139: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h113f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1140: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1141: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1142: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1143: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1144: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1145: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1146: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1147: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1148: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1149: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h114f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1150: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1151: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1152: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1153: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1154: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1155: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1156: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1157: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1158: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1159: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h115f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1160: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1161: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1162: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1163: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1164: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1165: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1166: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1167: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1168: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1169: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h116f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1170: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1171: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1172: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1173: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1174: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1175: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1176: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1177: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1178: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1179: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h117f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1180: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1181: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1182: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1183: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1184: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1185: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1186: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1187: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1188: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1189: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h118f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1190: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1191: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1192: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1193: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1194: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1195: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1196: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1197: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1198: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1199: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h119f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h11ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1200: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1201: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1202: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1203: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1204: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1205: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1206: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1207: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1208: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1209: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h120f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1210: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1211: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1212: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1213: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1214: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1215: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1216: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1217: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1218: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1219: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h121f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1220: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1221: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1222: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1223: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1224: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1225: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1226: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1227: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1228: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1229: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h122f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1230: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1231: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1232: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1233: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1234: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1235: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1236: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1237: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1238: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1239: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h123f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1240: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1241: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1242: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1243: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1244: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1245: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1246: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1247: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1248: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1249: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h124f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1250: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1251: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1252: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1253: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1254: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1255: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1256: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1257: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1258: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1259: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h125f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1260: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1261: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1262: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1263: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1264: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1265: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1266: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1267: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1268: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1269: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h126f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1270: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1271: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1272: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1273: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1274: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1275: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1276: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1277: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1278: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1279: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h127f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1280: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1281: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1282: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1283: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1284: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1285: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1286: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1287: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1288: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1289: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h128f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1290: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1291: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1292: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1293: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1294: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1295: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1296: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1297: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1298: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1299: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h129f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h12ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1300: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1301: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1302: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1303: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1304: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1305: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1306: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1307: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1308: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1309: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h130f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1310: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1311: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1312: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1313: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1314: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1315: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1316: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1317: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1318: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1319: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h131f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1320: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1321: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1322: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1323: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1324: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1325: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1326: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1327: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1328: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1329: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h132f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1330: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1331: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1332: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1333: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1334: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1335: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1336: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1337: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1338: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1339: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h133f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1340: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1341: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1342: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1343: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1344: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1345: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1346: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1347: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1348: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1349: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h134f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1350: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1351: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1352: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1353: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1354: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1355: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1356: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1357: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1358: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1359: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h135f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1360: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1361: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1362: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1363: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1364: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1365: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1366: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1367: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1368: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1369: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h136f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1370: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1371: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1372: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1373: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1374: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1375: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1376: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1377: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1378: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1379: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h137f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1380: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1381: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1382: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1383: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1384: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1385: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1386: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1387: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1388: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1389: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h138f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1390: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1391: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1392: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1393: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1394: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1395: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1396: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1397: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1398: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1399: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h139f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h13ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1400: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1401: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1402: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1403: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1404: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1405: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1406: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1407: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1408: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1409: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h140f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1410: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1411: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1412: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1413: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1414: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1415: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1416: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1417: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1418: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1419: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h141f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1420: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1421: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1422: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1423: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1424: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1425: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1426: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1427: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1428: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1429: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h142f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1430: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1431: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1432: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1433: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1434: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1435: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1436: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1437: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1438: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1439: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h143f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1440: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1441: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1442: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1443: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1444: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1445: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1446: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1447: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1448: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1449: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h144f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1450: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1451: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1452: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1453: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1454: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1455: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1456: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1457: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1458: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1459: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h145f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1460: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1461: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1462: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1463: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1464: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1465: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1466: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1467: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1468: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1469: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h146f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1470: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1471: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1472: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1473: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1474: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1475: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1476: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1477: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1478: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1479: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h147f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1480: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1481: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1482: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1483: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1484: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1485: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1486: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1487: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1488: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1489: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h148f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1490: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1491: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1492: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1493: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1494: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1495: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1496: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1497: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1498: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1499: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h149f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h14ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1500: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1501: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1502: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1503: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1504: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1505: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1506: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1507: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1508: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1509: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h150f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1510: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1511: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1512: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1513: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1514: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1515: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1516: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1517: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1518: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1519: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h151f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1520: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1521: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1522: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1523: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1524: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1525: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1526: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1527: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1528: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1529: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h152f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1530: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1531: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1532: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1533: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1534: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1535: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1536: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1537: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1538: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1539: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h153f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1540: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1541: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1542: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1543: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1544: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1545: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1546: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1547: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1548: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1549: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h154f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1550: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1551: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1552: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1553: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1554: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1555: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1556: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1557: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1558: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1559: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h155f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1560: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1561: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1562: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1563: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1564: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1565: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1566: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1567: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1568: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1569: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h156f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1570: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1571: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1572: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1573: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1574: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1575: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1576: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1577: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1578: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1579: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h157f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1580: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1581: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1582: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1583: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1584: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1585: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1586: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1587: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1588: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1589: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h158f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1590: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1591: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1592: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1593: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1594: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1595: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1596: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1597: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1598: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1599: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h159f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h15ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1600: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1601: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1602: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1603: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1604: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1605: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1606: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1607: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1608: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1609: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h160f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1610: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1611: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1612: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1613: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1614: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1615: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1616: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1617: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1618: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1619: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h161f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1620: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1621: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1622: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1623: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1624: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1625: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1626: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1627: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1628: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1629: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h162f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1630: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1631: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1632: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1633: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1634: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1635: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1636: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1637: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1638: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1639: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h163f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1640: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1641: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1642: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1643: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1644: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1645: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1646: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1647: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1648: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1649: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h164f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1650: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1651: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1652: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1653: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1654: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1655: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1656: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1657: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1658: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1659: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h165f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1660: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1661: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1662: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1663: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1664: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1665: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1666: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1667: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1668: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1669: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h166f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1670: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1671: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1672: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1673: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1674: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1675: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1676: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1677: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1678: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1679: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h167f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1680: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1681: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1682: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1683: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1684: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1685: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1686: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1687: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1688: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1689: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h168f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1690: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1691: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1692: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1693: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1694: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1695: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1696: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1697: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1698: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1699: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h169f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h16ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1700: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1701: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1702: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1703: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1704: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1705: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1706: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1707: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1708: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1709: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h170f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1710: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1711: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1712: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1713: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1714: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1715: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1716: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1717: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1718: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1719: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h171f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1720: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1721: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1722: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1723: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1724: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1725: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1726: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1727: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1728: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1729: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h172f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1730: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1731: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1732: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1733: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1734: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1735: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1736: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1737: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1738: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1739: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h173f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1740: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1741: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1742: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1743: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1744: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1745: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1746: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1747: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1748: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1749: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h174f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1750: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1751: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1752: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1753: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1754: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1755: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1756: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1757: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1758: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1759: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h175f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1760: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1761: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1762: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1763: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1764: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1765: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1766: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1767: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1768: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1769: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h176f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1770: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1771: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1772: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1773: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1774: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1775: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1776: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1777: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1778: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1779: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h177f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1780: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1781: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1782: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1783: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1784: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1785: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1786: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1787: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1788: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1789: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h178f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1790: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1791: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1792: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1793: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1794: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1795: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1796: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1797: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1798: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1799: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h179f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h17ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1800: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1801: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1802: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1803: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1804: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1805: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1806: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1807: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1808: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1809: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h180f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1810: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1811: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1812: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1813: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1814: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1815: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1816: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1817: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1818: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1819: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h181f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1820: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1821: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1822: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1823: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1824: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1825: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1826: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1827: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1828: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1829: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h182f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1830: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1831: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1832: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1833: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1834: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1835: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1836: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1837: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1838: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1839: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h183f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1840: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1841: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1842: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1843: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1844: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1845: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1846: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1847: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1848: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1849: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h184f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1850: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1851: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1852: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1853: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1854: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1855: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1856: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1857: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1858: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1859: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h185f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1860: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1861: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1862: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1863: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1864: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1865: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1866: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1867: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1868: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1869: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h186f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1870: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1871: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1872: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1873: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1874: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1875: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1876: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1877: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1878: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1879: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h187f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1880: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1881: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1882: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1883: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1884: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1885: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1886: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1887: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1888: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1889: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h188f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1890: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1891: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1892: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1893: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1894: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1895: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1896: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1897: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1898: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1899: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h189f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h18ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1900: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1901: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1902: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1903: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1904: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1905: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1906: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1907: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1908: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1909: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h190f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1910: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1911: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1912: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1913: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1914: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1915: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1916: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1917: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1918: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1919: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h191f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1920: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1921: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1922: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1923: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1924: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1925: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1926: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1927: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1928: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1929: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h192f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1930: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1931: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1932: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1933: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1934: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1935: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1936: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1937: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1938: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1939: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h193f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1940: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1941: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1942: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1943: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1944: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1945: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1946: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1947: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1948: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1949: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h194f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1950: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1951: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1952: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1953: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1954: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1955: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1956: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1957: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1958: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1959: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h195f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1960: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1961: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1962: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1963: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1964: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1965: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1966: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1967: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1968: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1969: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h196f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1970: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1971: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1972: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1973: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1974: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1975: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1976: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1977: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1978: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1979: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h197f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1980: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1981: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1982: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1983: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1984: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1985: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1986: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1987: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1988: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1989: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h198f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1990: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1991: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1992: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1993: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1994: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1995: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1996: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1997: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1998: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1999: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h199f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19a9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19aa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19af: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19b9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19bb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19bc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19bd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19be: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19bf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19c9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19cb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19cc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19cd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19cf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19d9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19da: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19db: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19dc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19dd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19de: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19df: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19e9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19eb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19f9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19fa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19fb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19fc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19fd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19fe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h19ff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1a9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aa9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aaa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aaf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ab9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1abb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1abc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1abd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1abe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1abf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ac9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1acb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1acc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1acd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ace: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1acf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ad9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ada: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1adb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1adc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1add: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ade: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1adf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ae9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aeb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1af9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1afa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1afb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1afc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1afd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1afe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1aff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1b9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ba9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1baa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1baf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1be9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1beb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bf9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1bff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1c9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ca9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1caa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1caf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ccb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ccc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ccd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ccf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ce9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ceb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ced: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cf9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1cff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1d9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1da9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1daa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1daf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1db9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ddb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ddc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ddd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ddf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1de9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1deb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ded: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1def: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1df9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dfa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dfb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dfc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dfd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dfe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1dff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1e9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ea9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eaa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ead: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eaf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ebb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ebc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ebd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ebe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ebf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ec9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ecb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ecc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ecd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ece: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ecf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ed9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1edb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1edc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1edd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ede: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1edf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ee9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eeb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ef9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1efa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1efb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1efc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1efd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1efe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1eff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f00: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f01: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f02: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f03: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f04: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f05: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f06: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f07: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f08: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f09: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f0f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f10: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f11: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f12: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f13: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f14: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f15: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f16: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f17: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f18: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f19: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f1f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f20: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f21: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f22: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f23: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f24: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f25: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f26: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f27: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f28: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f29: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f2f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f30: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f31: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f32: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f33: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f34: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f35: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f36: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f37: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f38: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f39: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f3f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f40: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f41: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f42: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f43: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f44: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f45: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f46: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f47: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f48: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f49: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f4f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f50: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f51: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f52: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f53: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f54: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f55: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f56: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f57: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f58: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f59: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f5f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f60: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f61: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f62: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f63: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f64: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f65: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f66: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f67: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f68: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f69: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f6f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f70: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f71: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f72: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f73: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f74: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f75: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f76: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f77: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f78: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f79: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f7f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f80: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f81: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f82: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f83: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f84: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f85: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f86: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f87: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f88: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f89: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f8f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f90: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f91: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f92: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f93: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f94: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f95: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f96: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f97: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f98: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f99: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9a: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9b: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9c: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9d: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9e: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1f9f: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fa9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1faa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fab: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fac: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fad: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fae: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1faf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fb9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fba: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fbb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fbc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fbd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fbe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fbf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fc9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fca: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fcb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fcc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fcd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fce: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fcf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fd9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fda: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fdb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fdc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fdd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fde: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fdf: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fe9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fea: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1feb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fec: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fed: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fee: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fef: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff0: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff1: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff2: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff3: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff4: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff5: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff6: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff7: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff8: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ff9: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ffa: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ffb: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ffc: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ffd: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1ffe: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					16'h1fff: top_level_bASIC.blkif.rom_rdata <= 32'h00000000;
					default: top_level_bASIC.blkif.rom_rdata <= 32'hdeadbeef;
				endcase
		end
		assign ROM.clk = clk;
		assign ROM.nRST = rst_n;
	endgenerate
	localparam [31:0] READ_BOT = 2;
	localparam [31:0] READ_TOP = 1;
	localparam [31:0] WRITE_BOT = 4;
	localparam [31:0] WRITE_TOP = 3;
	generate
		if (1) begin : RAM
			wire clk;
			wire nRST;
			localparam integer ADDRBITSIZE = 16;
			localparam integer DATABITSIZE = 32;
			localparam integer TOPROM = 16;
			localparam TOPSIZE = 3071;
			localparam RAM1_OFFSET = 512;
			localparam RAM2_OFFSET = 1024;
			localparam RAM3_OFFSET = 1536;
			localparam RAM4_OFFSET = 2048;
			localparam RAM5_OFFSET = 2560;
			localparam BOTTOMSIZE = 7;
			localparam TOP_OFFSET = 16'h2000;
			wire [15:0] bottom_addr;
			wire [15:0] top_addr;
			wire [31:0] TOPRAMdata;
			wire [31:0] BOTTOMRAMdata;
			reg bottom_wen;
			reg [15:0] addr0;
			reg [15:0] addr1;
			reg [15:0] addr2;
			reg [15:0] addr3;
			reg [15:0] addr4;
			reg [15:0] addr5;
			wire [31:0] dout_unused0;
			wire [31:0] dout_unused1;
			wire [31:0] dout_unused2;
			wire [31:0] dout_unused3;
			wire [31:0] dout_unused4;
			wire [31:0] dout_unused5;
			wire [31:0] next_dout_unused0;
			wire [31:0] next_dout_unused1;
			wire [31:0] next_dout_unused2;
			wire [31:0] next_dout_unused3;
			wire [31:0] next_dout_unused4;
			wire [31:0] next_dout_unused5;
			reg csb0;
			reg csb1;
			reg csb2;
			reg csb3;
			reg csb4;
			reg csb5;
			reg web;
			reg write_enable_delayed;
			reg [31:0] ram_rdata_sel;
			reg [31:0] next_ram_rdata_sel;
			reg [15:0] reg_addr;
			reg [31:0] ram_state;
			reg [31:0] nram_state;
			reg ram_wait;
			reg ram_later;
			reg next_ram_wait;
			wire curr_ram_wait;
			wire [191:0] ram_rdata_collection;
			wire [191:0] ram_rdata_copies;
			reg wait_source;
			reg next_wait_source;
			reg source_later;
			reg [31:0] w_data_reg;
			assign bottom_addr = top_level_bASIC.blkif.addr >> 2;
			assign top_addr = (top_level_bASIC.blkif.addr >> 2) - TOP_OFFSET;
			reg [3:0] byte_en_reg;
			always @(posedge clk or negedge nRST)
				if (!nRST) begin
					reg_addr <= {16 {1'sb1}};
					ram_wait <= 1'b1;
					wait_source <= 1'b0;
					ram_rdata_sel <= {32 {1'sb0}};
					write_enable_delayed <= 1'b0;
					ram_later <= 1'b0;
					source_later <= 1'b0;
					w_data_reg <= {32 {1'sb0}};
					ram_state <= IDLE;
					byte_en_reg <= {4 {1'sb0}};
				end
				else begin
					reg_addr <= top_level_bASIC.blkif.addr >> 2;
					ram_later <= (ram_state == READ_TOP) || (ram_state == READ_BOT);
					source_later <= wait_source;
					ram_wait <= next_ram_wait;
					wait_source <= next_wait_source;
					ram_rdata_sel <= next_ram_rdata_sel;
					write_enable_delayed <= top_level_bASIC.blkif.wen;
					w_data_reg <= top_level_bASIC.blkif.wdata & {{8 {top_level_bASIC.blkif.byte_en[3]}}, {8 {top_level_bASIC.blkif.byte_en[2]}}, {8 {top_level_bASIC.blkif.byte_en[1]}}, {8 {top_level_bASIC.blkif.byte_en[0]}}};
					ram_state <= nram_state;
					byte_en_reg <= top_level_bASIC.blkif.byte_en;
				end
			reg bot_active;
			assign top_level_bASIC.blkif.ram_active = ((ram_state == READ_TOP) || (ram_state == READ_BOT) ? 1'b1 : 1'b0);
			assign top_level_bASIC.blkif.ram_wait = ((ram_state == READ_TOP) || (ram_state == READ_BOT) ? 1'b1 : 1'b0);
			always @(*) begin
				top_level_bASIC.blkif.ram_rdata = 32'hbad1bad1;
				if (ram_state == READ_TOP)
					top_level_bASIC.blkif.ram_rdata = ram_rdata_collection[ram_rdata_sel * 32+:32];
				else if (ram_state == READ_BOT)
					top_level_bASIC.blkif.ram_rdata = BOTTOMRAMdata;
				else if (ram_state == WRITE_TOP)
					top_level_bASIC.blkif.ram_rdata = (ram_rdata_copies[ram_rdata_sel * 32+:32] & {{8 {~byte_en_reg[3]}}, {8 {~byte_en_reg[2]}}, {8 {~byte_en_reg[1]}}, {8 {~byte_en_reg[0]}}}) | w_data_reg;
				else if (ram_state == WRITE_BOT)
					top_level_bASIC.blkif.ram_rdata = (BOTTOMRAMdata & {{8 {~byte_en_reg[3]}}, {8 {~byte_en_reg[2]}}, {8 {~byte_en_reg[1]}}, {8 {~byte_en_reg[0]}}}) | w_data_reg;
				else
					top_level_bASIC.blkif.ram_rdata = 32'hbad1bad1;
			end
			always @(*) begin
				web = 1'b1;
				csb0 = 1'b1;
				csb1 = 1'b1;
				csb2 = 1'b1;
				csb3 = 1'b1;
				csb4 = 1'b1;
				csb5 = 1'b1;
				addr0 = {16 {1'sb0}};
				addr1 = {16 {1'sb0}};
				addr2 = {16 {1'sb0}};
				addr3 = {16 {1'sb0}};
				addr4 = {16 {1'sb0}};
				addr5 = {16 {1'sb0}};
				bot_active = 1'b0;
				next_ram_wait = 1'b0;
				next_ram_rdata_sel = ram_rdata_sel;
				next_wait_source = wait_source;
				bottom_wen = 1'b0;
				nram_state = ram_state;
				if (((top_level_bASIC.blkif.ram_en || (((ram_state == READ_TOP) || (ram_state == READ_BOT)) && !ram_later)) && (top_level_bASIC.blkif.addr >= 32'h00000000)) && (top_level_bASIC.blkif.addr <= 32'h00000028)) begin
					bot_active = 1'b1;
					bottom_wen = top_level_bASIC.blkif.wen;
					if (top_level_bASIC.blkif.wen)
						nram_state = WRITE_BOT;
					else
						nram_state = READ_BOT;
				end
				else if (((top_level_bASIC.blkif.ram_en || (((ram_state == READ_TOP) || (ram_state == READ_BOT)) && !ram_later)) && (top_level_bASIC.blkif.addr >= 32'h00008000)) && (top_level_bASIC.blkif.addr < 32'h00008c00)) begin
					web = ~top_level_bASIC.blkif.wen;
					if (!top_level_bASIC.blkif.wen)
						nram_state = READ_TOP;
					else
						nram_state = WRITE_TOP;
					if (top_addr < RAM1_OFFSET) begin
						csb0 = 1'b0;
						addr0 = top_addr;
						next_ram_rdata_sel = 32'd0;
					end
					else if (top_addr < RAM2_OFFSET) begin
						csb1 = 1'b0;
						addr1 = top_addr - RAM1_OFFSET;
						next_ram_rdata_sel = 32'd1;
					end
					else if (top_addr < RAM3_OFFSET) begin
						csb2 = 1'b0;
						addr2 = top_addr - RAM2_OFFSET;
						next_ram_rdata_sel = 32'd2;
					end
					else if (top_addr < RAM4_OFFSET) begin
						csb3 = 1'b0;
						addr3 = top_addr - RAM3_OFFSET;
						next_ram_rdata_sel = 32'd3;
					end
					else if (top_addr < RAM5_OFFSET) begin
						csb4 = 1'b0;
						addr4 = top_addr - RAM4_OFFSET;
						next_ram_rdata_sel = 32'd4;
					end
					else begin
						csb5 = 1'b0;
						addr5 = top_addr - RAM5_OFFSET;
						next_ram_rdata_sel = 32'd5;
					end
				end
				else
					nram_state = IDLE;
			end
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM0(
				.clk0(clk),
				.csb0(csb0),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr0[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[0+:32]),
				.clk1(clk),
				.csb1(csb0),
				.addr1(addr0[8:0]),
				.dout1(ram_rdata_copies[0+:32])
			);
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM1(
				.clk0(clk),
				.csb0(csb1),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr1[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[32+:32]),
				.clk1(clk),
				.csb1(csb1),
				.addr1(addr1[8:0]),
				.dout1(ram_rdata_copies[32+:32])
			);
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM2(
				.clk0(clk),
				.csb0(csb2),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr2[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[64+:32]),
				.clk1(clk),
				.csb1(csb2),
				.addr1(addr2[8:0]),
				.dout1(ram_rdata_copies[64+:32])
			);
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM3(
				.clk0(clk),
				.csb0(csb3),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr3[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[96+:32]),
				.clk1(clk),
				.csb1(csb3),
				.addr1(addr3[8:0]),
				.dout1(ram_rdata_copies[96+:32])
			);
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM4(
				.clk0(clk),
				.csb0(csb4),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr4[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[128+:32]),
				.clk1(clk),
				.csb1(csb4),
				.addr1(addr4[8:0]),
				.dout1(ram_rdata_copies[128+:32])
			);
			sky130_sram_2kbyte_1rw1r_32x512_8 TOPRAM5(
				.clk0(clk),
				.csb0(csb5),
				.web0(web),
				.wmask0(top_level_bASIC.blkif.byte_en),
				.addr0(addr5[8:0]),
				.din0(top_level_bASIC.blkif.wdata),
				.dout0(ram_rdata_collection[160+:32]),
				.clk1(clk),
				.csb1(csb5),
				.addr1(addr5[8:0]),
				.dout1(ram_rdata_copies[160+:32])
			);
			SOC_RAM #(
				.ADDRBIT(ADDRBITSIZE),
				.DATABIT(DATABITSIZE),
				.BOTTOMADDR(0),
				.TOPADDR(BOTTOMSIZE)
			) BOTTOMRAM(
				.clk(clk),
				.n_rst(nRST),
				.bot_active(bot_active),
				.w_data(top_level_bASIC.blkif.wdata),
				.addr(bottom_addr),
				.w_en(bottom_wen),
				.byte_en(top_level_bASIC.blkif.byte_en),
				.r_data(BOTTOMRAMdata)
			);
		end
		assign RAM.clk = clk;
		assign RAM.nRST = rst_n;
	endgenerate
	localparam [1:0] PLIC = 2;
	generate
		localparam [31:0] _param_47697_base_address = 32'he0010000;
		localparam signed [31:0] _param_47697_N_interrupts = N_INTERRUPTS;
		if (1) begin : plic_wrapper
			localparam [31:0] base_address = _param_47697_base_address;
			localparam N_interrupts = _param_47697_N_interrupts;
			wire clk;
			wire n_rst;
			wire [31:0] plicIf_hw_interrupt_requests;
			wire plicIf_interrupt_service_request;
			wire plicIf_interrupt_clear;
			if (1) begin : plicIf
				localparam N_interrupts = 32;
				wire interrupt_service_request;
				wire interrupt_clear;
				wire [31:0] hw_interrupt_requests;
				wire [31:0] addr;
				wire ren;
				wire wen;
				wire [31:0] rdata;
				wire [31:0] wdata;
				wire rambusy;
			end
			wire [31:0] wdata;
			wire [31:0] addr;
			wire r_prep;
			wire w_prep;
			wire wen;
			wire ren;
			wire [2:0] size;
			wire [4:0] burst_count;
			wire [2:0] burst_type;
			reg [31:0] prev_addr;
			reg prev_ren;
			reg prev_wen;
			assign plicIf.hw_interrupt_requests = plicIf_hw_interrupt_requests;
			assign plicIf_interrupt_service_request = plicIf.interrupt_service_request;
			assign plicIf_interrupt_clear = plicIf.interrupt_clear;
			always @(posedge clk or negedge n_rst)
				if (!n_rst) begin
					prev_addr <= {32 {1'sb0}};
					prev_ren <= 1'b0;
					prev_wen <= 1'b0;
				end
				else begin
					prev_addr <= top_level_bASIC.plic_ahb_if.HADDR;
					prev_ren <= ~top_level_bASIC.plic_ahb_if.HWRITE;
					prev_wen <= top_level_bASIC.plic_ahb_if.HWRITE;
				end
			assign plicIf.wdata = top_level_bASIC.plic_ahb_if.HWDATA;
			assign plicIf.rambusy = 1'b0;
			assign plicIf.wen = prev_wen;
			assign plicIf.ren = prev_ren;
			assign plicIf.addr = prev_addr;
			localparam NUMBER_ADDRESSES = (128 + ((1 + ((((_param_47697_N_interrupts[0] | _param_47697_N_interrupts[1]) | _param_47697_N_interrupts[2]) | _param_47697_N_interrupts[3]) | _param_47697_N_interrupts[4])) << 4)) + 12;
			localparam [31:0] _param_DD04D_base_address = base_address;
			localparam signed [31:0] _param_DD04D_N_interrupts = N_interrupts;
			if (1) begin : PLIC
				localparam N_interrupts = _param_DD04D_N_interrupts;
				localparam [31:0] base_address = _param_DD04D_base_address;
				wire clk;
				wire n_rst;
				wire [31:0] priority_addr;
				wire [31:0] pending_addr;
				wire [31:0] reserved_addr;
				wire [31:0] enable_addr;
				wire [31:0] priority_threshold_addr;
				wire [31:0] claim_complete_addr;
				wire carry_over_mod;
				wire [31:0] interrupt_requests_unmasked;
				wire [1023:0] interrupt_priority_regs;
				wire [31:0] interrupt_masks;
				wire [31:0] interrupt_requests_masked;
				wire [31:0] pending_interrupts;
				wire interrupt_claimed;
				wire [31:0] active_interrupt_ID;
				wire [31:0] active_interrupt;
				wire interrupt_priority_request;
				wire interrupt_processing;
				wire [31:0] rdata1;
				wire [31:0] rdata2;
				wire [31:0] rdata3;
				wire addr_valid1;
				wire addr_valid2;
				wire addr_valid3;
				assign carry_over_mod = (((N_interrupts[0] | N_interrupts[1]) | N_interrupts[2]) | N_interrupts[3]) | N_interrupts[4];
				assign top_level_bASIC.plic_wrapper.plicIf.interrupt_clear = interrupt_claimed;
				assign priority_addr = 33'sd3758161924;
				assign pending_addr = priority_addr + 128;
				assign enable_addr = pending_addr + ((1 + carry_over_mod) << 2);
				assign reserved_addr = enable_addr + ((1 + carry_over_mod) << 2);
				assign priority_threshold_addr = reserved_addr + 4;
				assign claim_complete_addr = priority_threshold_addr + 4;
				interrupt_enable_registers #(.N_interrupts(N_interrupts)) my_interrupt_en(
					.n_rst(n_rst),
					.clk(clk),
					.addr(top_level_bASIC.plic_wrapper.plicIf.addr),
					.wen(top_level_bASIC.plic_wrapper.plicIf.wen),
					.rdata(rdata1),
					.wdata(top_level_bASIC.plic_wrapper.plicIf.wdata),
					.addr_valid(addr_valid1),
					.interrupt_masks(interrupt_masks),
					.enable_addr(enable_addr),
					.reserved_addr(reserved_addr),
					.priority_threshold_addr(priority_threshold_addr),
					.claim_complete_addr(claim_complete_addr),
					.interrupt_priority_regs(interrupt_priority_regs)
				);
				register_mask #(.N_interrupts(N_interrupts)) my_register_mask_module(
					.interrupt_masks(interrupt_masks),
					.interrupt_requests(interrupt_requests_unmasked),
					.interrupt_requests_masked(interrupt_requests_masked)
				);
				interrupt_pending_priority_registers #(.N_interrupts(N_interrupts)) my_interrupt_pending_priority(
					.clk(clk),
					.n_rst(n_rst),
					.interrupt_requests_masked(interrupt_requests_masked),
					.pending_interrupts(pending_interrupts),
					.interrupt_priority_regs(interrupt_priority_regs),
					.active_interrupt(active_interrupt),
					.interrupt_claimed(interrupt_claimed),
					.priority_addr(priority_addr),
					.pending_addr(pending_addr),
					.enable_addr(enable_addr),
					.addr(top_level_bASIC.plic_wrapper.plicIf.addr),
					.wen(top_level_bASIC.plic_wrapper.plicIf.wen),
					.rdata(rdata2),
					.wdata(top_level_bASIC.plic_wrapper.plicIf.wdata),
					.addr_valid(addr_valid2)
				);
				interrupt_priority_resolve #(.N_INTERRUPTS(N_interrupts)) my_interrupt_priority_resolve(
					.clk(clk),
					.n_rst(n_rst),
					.interrupt_priorities(interrupt_priority_regs),
					.pending_interrupts(pending_interrupts),
					.active_interrupt(active_interrupt),
					.active_interrupt_ID(active_interrupt_ID),
					.interrupt_processing(interrupt_processing)
				);
				interrupt_request_reg #(.N_interrupts(N_interrupts)) my_interrupt_request_reg(
					.clk(clk),
					.n_rst(n_rst),
					.interrupt_requests_in(top_level_bASIC.plic_wrapper.plicIf.hw_interrupt_requests),
					.interrupt_requests(interrupt_requests_unmasked)
				);
				interrupt_claim_complete_register #(.N_interrupts(N_interrupts)) my_interrupt_claim_complete_register(
					.clk(clk),
					.n_rst(n_rst),
					.active_interrupt_ID(active_interrupt_ID),
					.active_interrupt(active_interrupt),
					.interrupt_claimed(interrupt_claimed),
					.interrupt_request_pulse(top_level_bASIC.plic_wrapper.plicIf.interrupt_service_request),
					.claim_complete_addr(claim_complete_addr),
					.addr(top_level_bASIC.plic_wrapper.plicIf.addr),
					.wen(top_level_bASIC.plic_wrapper.plicIf.wen),
					.rdata(rdata3),
					.wdata(top_level_bASIC.plic_wrapper.plicIf.wdata),
					.addr_valid(addr_valid3),
					.ren(top_level_bASIC.plic_wrapper.plicIf.ren),
					.interrupt_processing(interrupt_processing)
				);
				rdata_arbiter my_rdata(
					.rdata1(rdata1),
					.rdata2(rdata2),
					.rdata3(rdata3),
					.ren(top_level_bASIC.plic_wrapper.plicIf.ren),
					.addr_valid1(addr_valid1),
					.addr_valid2(addr_valid2),
					.addr_valid3(addr_valid3),
					.rdata(top_level_bASIC.plic_wrapper.plicIf.rdata)
				);
			end
			assign PLIC.clk = clk;
			assign PLIC.n_rst = n_rst;
			ahb_slave #(
				.BASE_ADDRESS(base_address),
				.NUMBER_ADDRESSES(NUMBER_ADDRESSES)
			) AHBS2(
				.HCLK(clk),
				.HRESETn(n_rst),
				.HMASTLOCK(top_level_bASIC.plic_ahb_if.HMASTLOCK),
				.HWRITE(top_level_bASIC.plic_ahb_if.HWRITE),
				.HSEL(top_level_bASIC.plic_ahb_if.HSEL),
				.HREADYIN(1'b1),
				.HADDR(top_level_bASIC.plic_ahb_if.HADDR),
				.HWDATA(top_level_bASIC.plic_ahb_if.HWDATA),
				.HTRANS(top_level_bASIC.plic_ahb_if.HTRANS),
				.HBURST(3'b000),
				.HSIZE(top_level_bASIC.plic_ahb_if.HSIZE),
				.HPROT(top_level_bASIC.plic_ahb_if.HPROT),
				.HRDATA(top_level_bASIC.plic_ahb_if.HRDATA),
				.HREADYOUT(top_level_bASIC.plic_ahb_if.HREADYOUT),
				.HRESP(top_level_bASIC.plic_ahb_if.HRESP),
				.burst_cancel(1'b0),
				.slave_wait(1'b0),
				.rdata(plicIf.rdata),
				.wdata(wdata),
				.addr(addr),
				.r_prep(r_prep),
				.w_prep(w_prep),
				.wen(wen),
				.ren(ren),
				.size(size),
				.burst_count(burst_count),
				.burst_type(burst_type)
			);
		end
		assign plic_wrapper.clk = clk;
		assign plic_wrapper.n_rst = rst_n;
		assign plic_wrapper.plicIf_hw_interrupt_requests = plicIf_hw_interrupt_requests;
		assign plicIf_interrupt_service_request = plic_wrapper.plicIf_interrupt_service_request;
		assign plicIf_interrupt_clear = plic_wrapper.plicIf_interrupt_clear;
	endgenerate
	localparam [1:0] CLINT = 3;
	generate
		localparam [31:0] _param_A6D19_base_address = 32'he0000000;
		if (1) begin : clint_wrapper
			localparam [31:0] base_address = _param_A6D19_base_address;
			wire clk;
			wire n_rst;
			wire clintIf_soft_int;
			wire clintIf_clear_soft_int;
			wire clintIf_timer_int;
			wire clintIf_clear_timer_int;
			if (1) begin : clintIf
				wire mtime_sel;
				wire mtimeh_sel;
				wire mtimecmp_sel;
				wire mtimecmph_sel;
				wire msip_sel;
				wire wen;
				wire ren;
				wire [31:0] wdata;
				reg [31:0] rdata;
				wire timer_int;
				wire clear_timer_int;
				wire soft_int;
				wire clear_soft_int;
				wire [31:0] addr;
				wire rambusy;
			end
			wire [31:0] wdata;
			wire [31:0] addr;
			wire r_prep;
			wire w_prep;
			wire wen;
			wire ren;
			wire [2:0] size;
			wire [4:0] burst_count;
			wire [2:0] burst_type;
			reg [31:0] prev_addr;
			reg prev_ren;
			reg prev_wen;
			assign clintIf_soft_int = clintIf.soft_int;
			assign clintIf_clear_soft_int = clintIf.clear_soft_int;
			assign clintIf_timer_int = clintIf.timer_int;
			assign clintIf_clear_timer_int = clintIf.clear_timer_int;
			always @(posedge clk or negedge n_rst)
				if (!n_rst) begin
					prev_addr <= {32 {1'sb0}};
					prev_ren <= 1'b0;
					prev_wen <= 1'b0;
				end
				else begin
					prev_addr <= top_level_bASIC.clint_ahb_if.HADDR;
					prev_ren <= ~top_level_bASIC.clint_ahb_if.HWRITE;
					prev_wen <= top_level_bASIC.clint_ahb_if.HWRITE;
				end
			assign clintIf.wdata = top_level_bASIC.clint_ahb_if.HWDATA;
			assign clintIf.rambusy = 1'b0;
			assign clintIf.wen = prev_wen;
			assign clintIf.ren = prev_ren;
			assign clintIf.addr = prev_addr;
			assign clintIf.msip_sel = clintIf.addr == base_address;
			assign clintIf.mtime_sel = clintIf.addr == 33'sd3758096388;
			assign clintIf.mtimeh_sel = clintIf.addr == 33'sd3758096392;
			assign clintIf.mtimecmp_sel = clintIf.addr == 33'sd3758096396;
			assign clintIf.mtimecmph_sel = clintIf.addr == 33'sd3758096400;
			localparam NUMBER_ADDRESSES = 20;
			if (1) begin : CLINT
				wire clk;
				wire n_rst;
				wire [31:0] mtime;
				wire [31:0] mtime_next;
				wire [31:0] mtimeh;
				wire [31:0] mtimeh_next;
				wire [31:0] mtimecmp;
				wire [31:0] mtimecmp_next;
				wire [31:0] mtimecmph;
				wire [31:0] mtimecmph_next;
				reg [31:0] msip;
				wire [31:0] msip_next;
				reg [63:0] mtimefull;
				reg [63:0] mtimefull_next;
				reg [63:0] mtimecmpfull;
				reg [63:0] mtimecmpfull_next;
				wire timer_int;
				reg prev_timer_int;
				assign mtime = mtimefull[31:0];
				assign mtimeh = mtimefull[63:32];
				assign mtimecmp = mtimecmpfull[31:0];
				assign mtimecmph = mtimecmpfull[63:32];
				assign timer_int = mtimefull >= mtimecmpfull;
				assign top_level_bASIC.clint_wrapper.clintIf.timer_int = timer_int & !prev_timer_int;
				assign top_level_bASIC.clint_wrapper.clintIf.clear_timer_int = top_level_bASIC.clint_wrapper.clintIf.wen & (top_level_bASIC.clint_wrapper.clintIf.mtimecmp_sel | top_level_bASIC.clint_wrapper.clintIf.mtimecmph_sel);
				assign top_level_bASIC.clint_wrapper.clintIf.soft_int = (top_level_bASIC.clint_wrapper.clintIf.wen & top_level_bASIC.clint_wrapper.clintIf.wdata[0]) & top_level_bASIC.clint_wrapper.clintIf.msip_sel;
				assign top_level_bASIC.clint_wrapper.clintIf.clear_soft_int = (top_level_bASIC.clint_wrapper.clintIf.wen & ~top_level_bASIC.clint_wrapper.clintIf.wdata[0]) & top_level_bASIC.clint_wrapper.clintIf.msip_sel;
				assign msip_next = (top_level_bASIC.clint_wrapper.clintIf.msip_sel & top_level_bASIC.clint_wrapper.clintIf.wen ? top_level_bASIC.clint_wrapper.clintIf.wdata : msip);
				always @(posedge clk or negedge n_rst)
					if (!n_rst) begin
						mtimefull <= {64 {1'sb0}};
						mtimecmpfull <= {64 {1'sb0}};
						msip <= {32 {1'sb0}};
						prev_timer_int <= 1'b0;
					end
					else begin
						mtimefull <= mtimefull_next;
						mtimecmpfull <= mtimecmpfull_next;
						msip <= msip_next;
						prev_timer_int <= timer_int;
					end
				always @(*) begin
					mtimefull_next = mtimefull + 1;
					if (top_level_bASIC.clint_wrapper.clintIf.mtime_sel & top_level_bASIC.clint_wrapper.clintIf.wen)
						mtimefull_next = {mtimefull[63:32], top_level_bASIC.clint_wrapper.clintIf.wdata};
					else if (top_level_bASIC.clint_wrapper.clintIf.mtimeh_sel & top_level_bASIC.clint_wrapper.clintIf.wen)
						mtimefull_next = {top_level_bASIC.clint_wrapper.clintIf.wdata, mtimefull[31:0]};
				end
				always @(*) begin
					mtimecmpfull_next = mtimecmpfull;
					if (top_level_bASIC.clint_wrapper.clintIf.mtimecmp_sel & top_level_bASIC.clint_wrapper.clintIf.wen)
						mtimecmpfull_next = {mtimecmpfull[63:32], top_level_bASIC.clint_wrapper.clintIf.wdata};
					else if (top_level_bASIC.clint_wrapper.clintIf.mtimecmph_sel & top_level_bASIC.clint_wrapper.clintIf.wen)
						mtimecmpfull_next = {top_level_bASIC.clint_wrapper.clintIf.wdata, mtimecmpfull[31:0]};
				end
				always @(*) begin
					top_level_bASIC.clint_wrapper.clintIf.rdata = 'bz;
					if (top_level_bASIC.clint_wrapper.clintIf.mtime_sel)
						top_level_bASIC.clint_wrapper.clintIf.rdata = mtime;
					else if (top_level_bASIC.clint_wrapper.clintIf.mtimeh_sel)
						top_level_bASIC.clint_wrapper.clintIf.rdata = mtimeh;
					else if (top_level_bASIC.clint_wrapper.clintIf.mtimecmp_sel)
						top_level_bASIC.clint_wrapper.clintIf.rdata = mtimecmp;
					else if (top_level_bASIC.clint_wrapper.clintIf.mtimecmph_sel)
						top_level_bASIC.clint_wrapper.clintIf.rdata = mtimecmph;
					else if (top_level_bASIC.clint_wrapper.clintIf.msip_sel)
						top_level_bASIC.clint_wrapper.clintIf.rdata = msip;
				end
			end
			assign CLINT.clk = clk;
			assign CLINT.n_rst = n_rst;
			ahb_slave #(
				.BASE_ADDRESS(base_address),
				.NUMBER_ADDRESSES(NUMBER_ADDRESSES)
			) AHBS2(
				.HCLK(clk),
				.HRESETn(n_rst),
				.HMASTLOCK(top_level_bASIC.clint_ahb_if.HMASTLOCK),
				.HWRITE(top_level_bASIC.clint_ahb_if.HWRITE),
				.HSEL(top_level_bASIC.clint_ahb_if.HSEL),
				.HREADYIN(1'b1),
				.HADDR(top_level_bASIC.clint_ahb_if.HADDR),
				.HWDATA(top_level_bASIC.clint_ahb_if.HWDATA),
				.HTRANS(top_level_bASIC.clint_ahb_if.HTRANS),
				.HBURST(3'b000),
				.HSIZE(top_level_bASIC.clint_ahb_if.HSIZE),
				.HPROT(top_level_bASIC.clint_ahb_if.HPROT),
				.HRDATA(top_level_bASIC.clint_ahb_if.HRDATA),
				.HREADYOUT(top_level_bASIC.clint_ahb_if.HREADYOUT),
				.HRESP(top_level_bASIC.clint_ahb_if.HRESP),
				.burst_cancel(1'b0),
				.slave_wait(1'b0),
				.rdata(clintIf.rdata),
				.wdata(wdata),
				.addr(addr),
				.r_prep(r_prep),
				.w_prep(w_prep),
				.wen(wen),
				.ren(ren),
				.size(size),
				.burst_count(burst_count),
				.burst_type(burst_type)
			);
		end
		assign clint_wrapper.clk = clk;
		assign clint_wrapper.n_rst = rst_n;
		assign clintIf_soft_int = clint_wrapper.clintIf_soft_int;
		assign clintIf_clear_soft_int = clint_wrapper.clintIf_clear_soft_int;
		assign clintIf_timer_int = clint_wrapper.clintIf_timer_int;
		assign clintIf_clear_timer_int = clint_wrapper.clintIf_clear_timer_int;
	endgenerate
	assign plicIf_hw_interrupt_requests = {17'b00000000000000000, i2c.interrupt, spiif.interrupts, gpioIf0.interrupt};
	assign gpio_output_en_low = ~gpioIf0.en_data;
	assign timer_output_en_low = ~pin_timer0If.output_en;
	assign SDA_output_en_low = ~master_active;
	assign SCL_output_en_low = ~master_active;
	assign SS_output_en_low = ~spiif.mode;
	assign SCK_output_en_low = ~spiif.mode;
	assign MOSI_output_en_low = ~spiif.mode;
	assign MISO_output_en_low = ~spiif.mode;
	assign pwm_w_data_0 = pwm_out;
	wire timer_sel_sub;
	assign timer_sel_sub = ~pin_timer0If.output_en;
	wire timer_sel_out;
	assign timer_sel_out = pin_timer0If.output_en;
	assign select_slave = muxed_ahb_if.HTRANS != 2'b00;
	reg [1:0] mux_sel;
	localparam [1:0] APB = 1;
	localparam [1:0] SRAM = 0;
	always @(posedge clk or negedge rst_n)
		if (!rst_n)
			mux_sel <= SRAM;
		else if (muxed_ahb_if.HREADY)
			if (muxed_ahb_if.HADDR < 32'h80000000)
				mux_sel <= SRAM;
			else if ((muxed_ahb_if.HADDR >= 32'he0000000) && (muxed_ahb_if.HADDR < 32'he0010000))
				mux_sel <= CLINT;
			else if ((muxed_ahb_if.HADDR >= 32'he0010000) && (muxed_ahb_if.HADDR < 32'hefffffff))
				mux_sel <= PLIC;
			else
				mux_sel <= APB;
	always @(*)
		if (muxed_ahb_if.HADDR < 32'h80000000) begin
			sramIf_ahbif.HSEL = select_slave;
			ahb2apbIf_ahbif.HSEL = 0;
			plic_ahb_if.HSEL = 0;
			clint_ahb_if.HSEL = 0;
		end
		else if ((muxed_ahb_if.HADDR >= 32'he0000000) && (muxed_ahb_if.HADDR < 32'he0010000)) begin
			sramIf_ahbif.HSEL = 0;
			ahb2apbIf_ahbif.HSEL = 0;
			plic_ahb_if.HSEL = 0;
			clint_ahb_if.HSEL = select_slave;
		end
		else if ((muxed_ahb_if.HADDR >= 32'he0010000) && (muxed_ahb_if.HADDR < 32'hefffffff)) begin
			sramIf_ahbif.HSEL = 0;
			ahb2apbIf_ahbif.HSEL = 0;
			plic_ahb_if.HSEL = select_slave;
			clint_ahb_if.HSEL = 0;
		end
		else begin
			sramIf_ahbif.HSEL = 0;
			ahb2apbIf_ahbif.HSEL = select_slave;
			plic_ahb_if.HSEL = 0;
			clint_ahb_if.HSEL = 0;
		end
	always @(*) begin
		la_data_nxt[31:0] = muxed_ahb_if.HADDR;
		la_data_nxt[63:32] = muxed_ahb_if.HWDATA;
		la_data_nxt[95:33] = muxed_ahb_if.HRDATA;
		la_data_nxt[96] = muxed_ahb_if.HREADY;
		la_data_nxt[98:97] = muxed_ahb_if.HRESP;
		la_data_nxt[98] = 1'b0;
		la_data_nxt[100:99] = muxed_ahb_if.HTRANS;
		la_data_nxt[101] = muxed_ahb_if.HWRITE;
		la_data_nxt[104:102] = muxed_ahb_if.HSIZE;
		la_data_nxt[107:105] = muxed_ahb_if.HBURST;
		la_data_nxt[111:108] = muxed_ahb_if.HPROT;
		la_data_nxt[112] = muxed_ahb_if.HMASTLOCK;
		la_data_nxt[113] = rst_n;
		la_data_nxt[114] = clk;
		la_data_nxt[127:115] = {13 {1'sb0}};
		if (mux_sel == SRAM) begin
			muxed_ahb_if.HREADY = sramIf_ahbif.HREADYOUT;
			la_data_nxt[96] = sramIf_ahbif.HREADYOUT;
			muxed_ahb_if.HRESP[0] = sramIf_ahbif.HRESP;
			la_data_nxt[97] = sramIf_ahbif.HRESP;
			muxed_ahb_if.HRDATA = sramIf_ahbif.HRDATA;
			la_data_nxt[95:33] = sramIf_ahbif.HRDATA;
		end
		else if (mux_sel == APB) begin
			muxed_ahb_if.HREADY = ahb2apbIf_ahbif.HREADYOUT;
			la_data_nxt[96] = ahb2apbIf_ahbif.HREADYOUT;
			muxed_ahb_if.HRESP[0] = ahb2apbIf_ahbif.HRESP;
			la_data_nxt[97] = sramIf_ahbif.HRESP;
			muxed_ahb_if.HRDATA = ahb2apbIf_ahbif.HRDATA;
			la_data_nxt[95:33] = ahb2apbIf_ahbif.HRDATA;
		end
		else if (mux_sel == PLIC) begin
			muxed_ahb_if.HREADY = plic_ahb_if.HREADYOUT;
			la_data_nxt[96] = plic_ahb_if.HREADYOUT;
			muxed_ahb_if.HRESP[0] = plic_ahb_if.HRESP;
			la_data_nxt[97] = sramIf_ahbif.HRESP;
			muxed_ahb_if.HRDATA = plic_ahb_if.HRDATA;
			la_data_nxt[95:33] = plic_ahb_if.HRDATA;
		end
		else if (mux_sel == CLINT) begin
			muxed_ahb_if.HREADY = clint_ahb_if.HREADYOUT;
			la_data_nxt[96] = clint_ahb_if.HREADYOUT;
			muxed_ahb_if.HRESP[0] = clint_ahb_if.HRESP;
			la_data_nxt[97] = sramIf_ahbif.HRESP;
			muxed_ahb_if.HRDATA = clint_ahb_if.HRDATA;
			la_data_nxt[95:33] = clint_ahb_if.HRDATA;
		end
		else begin
			muxed_ahb_if.HREADY = 1'b1;
			la_data_nxt[96] = 1'b1;
			muxed_ahb_if.HRESP[0] = 1'b1;
			la_data_nxt[97] = 1'b1;
			muxed_ahb_if.HRDATA = 32'hbad1bad1;
			la_data_nxt[95:33] = 32'hbad1bad1;
		end
	end
	wire [1:1] sv2v_tmp_CAC9A;
	assign sv2v_tmp_CAC9A = 1'b0;
	always @(*) muxed_ahb_if.HRESP[1] = sv2v_tmp_CAC9A;
	assign sramIf_ahbif.HTRANS = muxed_ahb_if.HTRANS;
	assign sramIf_ahbif.HWRITE = muxed_ahb_if.HWRITE;
	assign sramIf_ahbif.HADDR = muxed_ahb_if.HADDR;
	assign sramIf_ahbif.HWDATA = muxed_ahb_if.HWDATA;
	assign sramIf_ahbif.HSIZE = muxed_ahb_if.HSIZE;
	assign sramIf_ahbif.HBURST = muxed_ahb_if.HBURST;
	assign sramIf_ahbif.HPROT = muxed_ahb_if.HPROT;
	assign sramIf_ahbif.HMASTLOCK = muxed_ahb_if.HMASTLOCK;
	assign sramIf_ahbif.HREADY = muxed_ahb_if.HREADY;
	assign ahb2apbIf_ahbif.HTRANS = muxed_ahb_if.HTRANS;
	assign ahb2apbIf_ahbif.HWRITE = muxed_ahb_if.HWRITE;
	assign ahb2apbIf_ahbif.HADDR = muxed_ahb_if.HADDR;
	assign ahb2apbIf_ahbif.HWDATA = muxed_ahb_if.HWDATA;
	assign ahb2apbIf_ahbif.HSIZE = muxed_ahb_if.HSIZE;
	assign ahb2apbIf_ahbif.HBURST = muxed_ahb_if.HBURST;
	assign ahb2apbIf_ahbif.HPROT = muxed_ahb_if.HPROT;
	assign ahb2apbIf_ahbif.HMASTLOCK = muxed_ahb_if.HMASTLOCK;
	assign ahb2apbIf_ahbif.HREADY = muxed_ahb_if.HREADY;
	assign plic_ahb_if.HTRANS = muxed_ahb_if.HTRANS;
	assign plic_ahb_if.HWRITE = muxed_ahb_if.HWRITE;
	assign plic_ahb_if.HADDR = muxed_ahb_if.HADDR;
	assign plic_ahb_if.HWDATA = muxed_ahb_if.HWDATA;
	assign plic_ahb_if.HSIZE = muxed_ahb_if.HSIZE;
	assign plic_ahb_if.HBURST = muxed_ahb_if.HBURST;
	assign plic_ahb_if.HPROT = muxed_ahb_if.HPROT;
	assign plic_ahb_if.HMASTLOCK = muxed_ahb_if.HMASTLOCK;
	assign plic_ahb_if.HREADY = muxed_ahb_if.HREADY;
	assign clint_ahb_if.HTRANS = muxed_ahb_if.HTRANS;
	assign clint_ahb_if.HWRITE = muxed_ahb_if.HWRITE;
	assign clint_ahb_if.HADDR = muxed_ahb_if.HADDR;
	assign clint_ahb_if.HWDATA = muxed_ahb_if.HWDATA;
	assign clint_ahb_if.HSIZE = muxed_ahb_if.HSIZE;
	assign clint_ahb_if.HBURST = muxed_ahb_if.HBURST;
	assign clint_ahb_if.HPROT = muxed_ahb_if.HPROT;
	assign clint_ahb_if.HMASTLOCK = muxed_ahb_if.HMASTLOCK;
	assign clint_ahb_if.HREADY = muxed_ahb_if.HREADY;
	reset_synchronizer_n core_reset(
		.clk(clk),
		.async_reset(asyncrst_n),
		.sync_reset(sync_rst_n)
	);
	always @(*) begin
		rst_n = 1'b1;
		if (!sync_rst_n)
			rst_n = 1'b0;
		else if (wb_rst_i)
			rst_n = 1'b0;
	end
endmodule
