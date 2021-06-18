
`timescale 1ns/10ps
`define GPIO_R_IND 		0
`define GPIO_W_IND 		1
`define GPIO_EN_IND 	2

`define GPIO_DATA_ADDR 				32'h8000_1004
`define GPIO_EN_ADDR				32'h8000_1008
`define GPIO_DATA_ADDR2				32'h8000_2004
`define GPIO_EN_ADDR2				32'h8000_2008

`define COMMAND_SET_COUNT 		{1'b1,7'd2}
`define COMMAND_SET_ADDR 		{1'b1,7'd3}
`define COMMAND_READ_DATA		{1'b1,7'd4}
`define COMMAND_WRITE_DATA 		{1'b1,7'd5}
`define COMMAND_ALIVE			{1'b1,7'd6}
`define COMMAND_CORE_RST		{1'b1,7'd7}
`define COMMAND_CORE_NORM		{1'b1,7'd8}


module tb_top_level_bASIC;

parameter PERIOD = 20;
parameter S_BIT_PERIOD = 8620;

logic tx_o = 1'b1;
logic rx_i;
logic n_rst;
logic clk = 1'b1;
//wire [7:0] gpio0_in;
//wire [7:0] gpio0_out;
wire [7:0] gpio0_bidir_inout;
wire [7:0] gpio0_sel_out;
wire [7:0] gpio0_sel_sub_out;
logic [7:0] gpio_data;

logic [7:0] crc_data;

integer i;


//assign gpio0_in = 8'hAB;

assign gpio0_bidir_inout = (~gpio0_sel_out) ? 8'bZ : gpio_data;

// offchip_sram_simulator  sram_sim(clk, offchip_sramif);

wire [31:0] mem_data_pad;
logic [18:0]offchip_sramif_external_addr;
logic [3:0] offchip_sramif_WE;
logic [3:0] offchip_sramif_nWE;
wire [31:0] offchip_sramif_external_bidir;
logic offchip_sramif_nOE;


offchip_sram_if offchip_sramif(offchip_sramif_external_bidir);

assign offchip_sramif.nWE = offchip_sramif_nWE;
assign offchip_sramif.nOE = offchip_sramif_nOE;
assign offchip_sramif.external_addr = offchip_sramif_external_addr;

// Removed references to external RAM models

logic wb_clk_i;
logic wbs_stb_i;
logic wbs_cyc_i;
logic wbs_we_i;
logic [3:0] wbs_sel_i;
logic [31:0]wbs_dat_i;
logic [31:0]wbs_adr_i;
logic  wbs_ack_o;
logic [31:0] wbs_dat_o;
logic wrong_data;
logic  wb_stimulus_en;
/*
wishbone_stimulus wishbone (
	.wb_clk_i(clk),
        .wbs_stb_i(wbs_stb_i),
        .wbs_cyc_i(wbs_cyc_i),
        .wbs_we_i(wbs_we_i),
	.add_wait(1'b0),
        .wbs_sel_i(wbs_sel_i),
        .wbs_dat_i(wbs_dat_i),
        .wbs_adr_i(wbs_adr_i),
        .wbs_ack_o(wbs_ack_o),
        .wbs_dat_o(wbs_dat_o),
	.wrong_data(wrong_data),
	.wb_stimulus_en('0),
	.nRST(n_rst) );






    wb_intercon #(
        .AW(ADR_WIDTH),
        .DW(DAT_WIDTH),
        .NS(NUM_SLAVES),
        .ADR_MASK(ADR_MASK),
        .SLAVE_ADR(SLAVE_ADR)
    ) intercon (
        // Master Interface
        .wbm_adr_i(cpu_adr_o),
        .wbm_stb_i(cpu_stb_o),
        .wbm_dat_o(cpu_dat_i),
        .wbm_ack_o(cpu_ack_i),

        // Slaves Interface
        .wbs_stb_o({ sys_stb_i, spimemio_cfg_stb_i,
		mprj_stb_o, mprj_ctrl_stb_i, la_stb_i, 
		spi_master_stb_i, counter_timer1_stb_i, counter_timer0_stb_i,
		gpio_stb_i, uart_stb_i,
		spimemio_flash_stb_i, stg_ro_stb_i, stg_rw_stb_i, mem_stb_i }), 
        .wbs_dat_i({ sys_dat_o, spimemio_cfg_dat_o,
		mprj_dat_i, mprj_ctrl_dat_o, la_dat_o,
		spi_master_dat_o, counter_timer1_dat_o, counter_timer0_dat_o,
		gpio_dat_o, uart_dat_o,
		spimemio_flash_dat_o, stg_ro_dat_o ,stg_rw_dat_o, mem_dat_o }),
        .wbs_ack_i({ sys_ack_o, spimemio_cfg_ack_o,
		mprj_ack_i, mprj_ctrl_ack_o, la_ack_o,
		spi_master_ack_o, counter_timer1_ack_o, counter_timer0_ack_o,
		gpio_ack_o, uart_ack_o,
		spimemio_flash_ack_o, stg_ro_ack_o, stg_rw_ack_o, mem_ack_o })
    );
*/


top_level_bASIC bASIC(
	.wb_clk_i(clk),
	.wb_rst_i(1'b0),
	.asyncrst_n(n_rst),
	.clk_sel(1'b1),
	.user_clock2(clk),
	//debugger signals
	.uart_debug_rx(tx_o),
	.uart_debug_tx(rx_i),
	//peak at ram_addr	
	.gpio_bidir_io(gpio0_bidir_inout),
        .gpio_output_en_low(gpio0_sel_out),
        .wbs_stb_i('0),
        .wbs_cyc_i('0),
        .wbs_we_i('0),
        .wbs_sel_i('0),
        .wbs_dat_i('0),
        .wbs_adr_i('0)
 //       .wbs_ack_o(wbs_ack_o),
 //       .wbs_dat_o(wbs_dat_o)

        //.gpio_sel_out(gpio0_sel_out),
        //.gpio_sel_sub(gpio0_sel_sub_out)
        //.offchip_sramif_external_addr(offchip_sramif_external_addr),
        //.offchip_sramif_WE_out(offchip_sramif_WE),
        //.offchip_sramif_nWE_out(offchip_sramif_nWE),
        //.offchip_sramif_external_bidir(offchip_sramif_external_bidir),
        //.offchip_sramif_nOE(offchip_sramif_nOE)
);
	task send_bits(input logic [8:0]data);
	//$display("Sending %d", data[7:0]);
	begin
		for(int i =0; i <= 10; i++) begin
			//repeat (435) begin
			//	@(posedge clk);
			//end
			case (i)
				0: tx_o <= 1'b0;
				1: tx_o <= data[i-1];
				2: tx_o <= data[i-1];
				3: tx_o <= data[i-1];
				4: tx_o <= data[i-1];
				5: tx_o <= data[i-1];
				6: tx_o <= data[i-1];
				7: tx_o <= data[i-1];
				8: tx_o <= data[i-1];
				9: tx_o <= 1'b1;
			endcase 
			#(S_BIT_PERIOD);
		end
		#(S_BIT_PERIOD);
	end
	endtask

	task receive_bits(output logic [7:0] data);
	begin 
                logic start_recieving;
                start_recieving = 1'b0;
		//#(S_BIT_PERIOD*11);
		@(negedge rx_i);
		//$display("Start receiving\n");
		#(S_BIT_PERIOD);
                start_recieving = 1'b1;
		for(int i = 0; i < 8; i++) begin
			#(S_BIT_PERIOD/2);
			data[i] <= rx_i;
			#(S_BIT_PERIOD/2);
		end
                start_recieving = 1'b0;
		#(S_BIT_PERIOD);
		#(S_BIT_PERIOD);
             
	end
	endtask

	task receive_no_bits(output logic [7:0] data);
	begin /*
                logic start_recieving;
                start_recieving = 1'b0;
		//#(S_BIT_PERIOD*11);
		@(negedge rx_i);
		//$display("Start receiving\n");
		#(S_BIT_PERIOD);
                start_recieving = 1'b1;
		for(int i = 0; i < 8; i++) begin
			#(S_BIT_PERIOD/2);
			data[i] <= rx_i;
			#(S_BIT_PERIOD/2);
		end
                start_recieving = 1'b0;
		#(S_BIT_PERIOD);
		#(S_BIT_PERIOD);
               */
	end
	endtask



	task reset;
	begin
		tx_o <= 1'b1;
		n_rst <= 1'b0;
		#10;
		n_rst <= 1'b1;
	end
	endtask

	task set_count(input logic [7:0]count);
	begin
		send_bits(`COMMAND_SET_COUNT);
		send_bits(count);
		receive_no_bits(crc_data);
	end
	endtask

	task set_addr(input logic [31:0] addr);
	begin
		send_bits(`COMMAND_SET_ADDR);
		send_bits(addr[31:24]);
		send_bits(addr[23:16]);
		send_bits(addr[15:8]);
		send_bits(addr[7:0]);
		receive_no_bits(crc_data);
	end
	endtask
		
	task write32(input logic [31:0] data);
	begin
		send_bits(`COMMAND_WRITE_DATA);
		send_bits(data[31:24]);
		send_bits(data[23:16]);
		send_bits(data[15:8]);
		send_bits(data[7:0]);
		receive_no_bits(crc_data);
	end
	endtask

	task read32(output logic [31:0] rx_data);
	begin
		send_bits(`COMMAND_READ_DATA);
		//TODO: get uart data back
                
		//receive_bits(rx_data[31:24]);
                			
		rx_data[24] <= rx_i;
		#(S_BIT_PERIOD / 2);
		rx_data[25] <= rx_i;
		#(S_BIT_PERIOD);
		rx_data[26] <= rx_i;
		#(S_BIT_PERIOD);
		rx_data[27] <= rx_i;
		#(S_BIT_PERIOD);
		rx_data[28] <= rx_i;
		#(S_BIT_PERIOD);
		rx_data[29] <= rx_i;
		#(S_BIT_PERIOD);
		rx_data[30] <= rx_i;
		#(S_BIT_PERIOD)		
                rx_data[31] <= rx_i;
		receive_bits(rx_data[23:16]);
		receive_bits(rx_data[15:8]);
		receive_bits(rx_data[7:0]);

		receive_no_bits(crc_data);
	end
	endtask

	task alive;
	begin
		send_bits(`COMMAND_ALIVE);
		//debugger should return 0x00AE through UART
		receive_no_bits(crc_data);
	end
	endtask

	task core_rst;
	begin
		send_bits(`COMMAND_CORE_RST);
		#(S_BIT_PERIOD*11);
	end
	endtask

	task core_norm;
	begin
		send_bits(`COMMAND_CORE_NORM);
		#(S_BIT_PERIOD*11);
	end
	endtask

	//Clock generation
	initial begin	
		clk = 1'b1;
		forever #(PERIOD/2) clk = ~clk;
	end

  task writeSRAM(input logic [31:0] addr, input logic [31:0] data);
  begin
  		
		set_addr(addr);
		//$display("Writing to %h of the SRAM\n", addr[15:0]);
		set_count(8'h1);
		write32(data);
		$display("Wrote %h to %h of the SRAM at %t \n", data, addr[15:0], $time);
  end
  endtask
	
  task readSRAM(input logic [31:0] addr);
  begin
  		logic [31:0] data;
		set_addr(addr);
		//$display("Reading from %h of the SRAM\n", addr[15:0]);
		set_count(8'h1);
		read32(data);
		$display("Read %h from %h of the SRAM\n", data, addr[15:0]);
  end
  endtask

	task loadFile();
	integer data_file;
	logic [31:0] ram_addr;
	logic [31:0] ram_data;
	begin
		$display("Loading file!");
		ram_addr = 32'h0000_0000;
		ram_data = 32'hBAD1_C0DE;
	
		data_file = $fopen("/local/scratch/a/socet20/bit_reads.mif", "r");
		if (!data_file) begin
			return;
		end

		//while(!$feof(data_file) && (ram_addr < 32'h0000_02E0)) begin
		while(!$feof(data_file)) begin
			
			$fscanf(data_file, "%d:%h;\n", ram_addr, ram_data);
			$display("SRAM[0x%h] <= 0x%h", ram_addr, ram_data);
			writeSRAM({ram_addr,2'b0}, ram_data);

		end
	end
	endtask

	//write and read using debugger
	
	initial begin
		$display("About to Reset\n");
        gpio_data = 8'h00;
		reset();
		$display("Reset\n");
		core_rst();

  
    gpio_data = 8'h01;
    #500;
    gpio_data = 8'h00;
    #600_000;

  


/*
//    $display("Begin onchip SRAM test\n");
    for(i = 0; i < 1024; i++) begin
       //$display("Begin onchip SRAM Write\n");
        writeSRAM(32'h8000 + 4*i, i);
        #(S_BIT_PERIOD*11);
       //$display("Begin onchip SRAM Read\n");
        readSRAM(32'h8000 + 4*i);
        #(S_BIT_PERIOD*11);
    //  #((10*i*i*i) % 1234);
    end
*/    
    
    // // test upload of code to register RAM
     /*
     writeSRAM(32'h8000, 32'h800000b7);
     writeSRAM(32'h8004, 32'h00408093);
     writeSRAM(32'h8008, 32'h0ff00113);
     writeSRAM(32'h800C, 32'h0020a223);
     writeSRAM(32'h8010, 32'h0dd00113);
     writeSRAM(32'h8014, 32'h0020a023);
     writeSRAM(32'h8018, 32'h0000006f);
     writeSRAM(32'h801C, 32'h00100093);
     writeSRAM(32'h8020, 32'h00100113);
     writeSRAM(32'h8024, 32'h00000097);
     writeSRAM(32'h8028, 32'h3c20ae23);
     writeSRAM(32'h802C, 32'hfd5ff06f);
     writeSRAM('0, 32'h8000);
     */
    // // test upload of code to bottom of SRAM addr
    // writeSRAM(32'h8400, 32'h800000b7);
    // writeSRAM(32'h8404, 32'h00408093);
    // writeSRAM(32'h8408, 32'h0ff00113);
    // writeSRAM(32'h840C, 32'h0020a223);
    // writeSRAM(32'h8410, 32'h0dd00113);
    // writeSRAM(32'h8414, 32'h0020a023);
    // writeSRAM(32'h8418, 32'h0000006f);
    // writeSRAM(32'h841C, 32'h00100093);
    // writeSRAM(32'h8420, 32'h00100113);
    // writeSRAM(32'h8424, 32'h00000097);
    // writeSRAM(32'h8428, 32'h3c20ae23);
    // writeSRAM(32'h842C, 32'hfd5ff06f);
    // writeSRAM('0, 32'h8400);

 
    // test upload of code to bottom of SRAM addr
    //writeSRAM(32'h1F0400, 32'h800000b7);
    //writeSRAM(32'h1F0404, 32'h00408093);
    //writeSRAM(32'h1F0408, 32'h0ff00113);
    //writeSRAM(32'h1F040C, 32'h0020a223);
    //writeSRAM(32'h1F0410, 32'h0dd00113);
    //writeSRAM(32'h1F0414, 32'h0020a023);
    //writeSRAM(32'h1F0418, 32'h0000006f);
    //writeSRAM(32'h1F041C, 32'h00100093);
    //writeSRAM(32'h1F0420, 32'h00100113);
    //writeSRAM(32'h1F0424, 32'h00000097);
    //writeSRAM(32'h1F0428, 32'h3c20ae23);
    //writeSRAM(32'h1F042C, 32'hfd5ff06f);
    //writeSRAM('0, 32'h1F0400);
 

    end
	
endmodule




