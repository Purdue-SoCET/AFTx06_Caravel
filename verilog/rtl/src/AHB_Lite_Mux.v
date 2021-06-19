/*
MODULE NAME         	: AHB_Lite_Mux
AUTHOR                 	: Andrew Brito & Chuan Yean, Tan
LAST UPDATE        	: 7/1/13
VERSION         	: 1.0
DESCRIPTION         	: Priority mux that arbitrates between the debugger and M0-Core if the debugger
			  begins a read or write.
*/

module AHB_Lite_Mux(
		//--INPUTS--//
		input              clk,
      		input              n_Rst,

		//--DEBUGGER SIGNAL (REMOVE LATER)--//
		input		   debug_rst,

		//--MASTER 1 SIGNALS--//
		input wire 	   HWRITE_M1,
		input wire 	   [2:0] HSIZE_M1,
		input wire 	   [2:0] HBURST_M1,
		input wire 	   [1:0] HTRANS_M1,
		input wire 	   [31:0] HADDR_M1,
		input wire 	   [31:0] HWDATA_M1,        
		input wire 	   [3:0] HPROT_M1,

		output reg 	   HREADY_M1,
		output reg 	   [31:0] HRDATA_M1,
		output reg 	   [ 1:0] HRESP_M1,

		//--MASTER 2 SIGNALS--//
		input wire 	   HWRITE_M2,
		input wire 	   [2:0] HSIZE_M2,
		input wire 	   [2:0] HBURST_M2,
		input wire 	   [1:0] HTRANS_M2,
		input wire 	   [31:0] HADDR_M2,
		input wire 	   [31:0] HWDATA_M2,        
		input wire 	   [3:0] HPROT_M2,
		
		output reg 	   HREADY_M2,
		output reg 	   [31:0] HRDATA_M2,
		output reg 	   [ 1:0] HRESP_M2,

		//--AHB BUS SIGNALS--//
		input 		HREADY,
		input wire     	[31:0] HRDATA,
		input wire     	[ 1:0] HRESP,

		output reg     	HWRITE,
		output reg     	[2:0] HSIZE,
		output reg     	[2:0] HBURST,
		output reg     	[1:0] HTRANS,
		output reg     	[31:0] HADDR,
		output reg     	[31:0] HWDATA,
		output reg     	[3:0] HPROT
		);

/*
MASTER 1
Cache Registers
-HTRANS        -HADDR        -HWRITE
-HPROT         -HWDATA        -HSIZE        -HBURST
*/
reg [1:0]  HTRANS_REG_M1, next_HTRANS_M1;
reg [31:0] HADDR_REG_M1, next_ADDR_M1;
reg HWRITE_REG_M1, next_HWRITE_M1;
reg [3:0] HPROT_REG_M1, next_HPROT_M1;
reg [2:0] HSIZE_REG_M1, next_HSIZE_M1;
reg [2:0] HBURST_REG_M1, next_HBURST_M1;

/*
MASTER 2
Cache Registers
-HTRANS        -HADDR        -HWRITE
-HPROT         -HWDATA        -HSIZE        -HBURST
*/
reg [1:0]  HTRANS_REG_M2, next_HTRANS_M2;
reg [31:0] HADDR_REG_M2, next_ADDR_M2;
reg HWRITE_REG_M2, next_HWRITE_M2;
reg [3:0] HPROT_REG_M2, next_HPROT_M2;
reg [2:0] HSIZE_REG_M2, next_HSIZE_M2;
reg [2:0] HBURST_REG_M2, next_HBURST_M2;

//Variables
reg [1:0] prev_HTRANS_M2;
reg D_PHASE, next_D_PHASE;	//indicating current master data phase
reg [1:0] edge_detect;		//detect rising/falling edge 
reg valid_M1, next_valid_M1;
reg valid_M2, next_valid_M2;


//Clock State Machine
always @ (posedge clk, negedge n_Rst) begin
        if (n_Rst == 1'b0) begin
                //reset cache registers
                HTRANS_REG_M1 = 2'd0;
                HADDR_REG_M1  = 32'd0;
                HWRITE_REG_M1 = 1'd0;
                HPROT_REG_M1  = 4'b0;
                HSIZE_REG_M1  = 3'd0;
                HBURST_REG_M1 = 3'd0;
		
		HTRANS_REG_M2 = 2'd0;
                HADDR_REG_M2  = 32'd0;
                HWRITE_REG_M2 = 1'd0;
                HPROT_REG_M2  = 4'b0;
                HSIZE_REG_M2  = 3'd0;
                HBURST_REG_M2 = 3'd0;
            
		//reset registers
		D_PHASE 	= 1'b0;
		valid_M1	= 1'b0;
		valid_M2	= 1'b0;

		prev_HTRANS_M2 	= 2'b00;
        end else begin
                //next cache registers
                HADDR_REG_M1   	= next_ADDR_M1;
                HTRANS_REG_M1  	= next_HTRANS_M1;
                HWRITE_REG_M1  	= next_HWRITE_M1;
                HPROT_REG_M1   	= next_HPROT_M1;
                HSIZE_REG_M1   	= next_HSIZE_M1;
                HBURST_REG_M1  	= next_HBURST_M1;
               
		            HADDR_REG_M2   	= next_ADDR_M2;
                HTRANS_REG_M2  	= next_HTRANS_M2;
                HWRITE_REG_M2  	= next_HWRITE_M2;
                HPROT_REG_M2   	= next_HPROT_M2;
                HSIZE_REG_M2   	= next_HSIZE_M2;
                HBURST_REG_M2  	= next_HBURST_M2;

		D_PHASE		= next_D_PHASE;
		valid_M1 	= next_valid_M1;
		valid_M2 	= next_valid_M2;
		prev_HTRANS_M2 	= HTRANS_M2;
        end        
end

/*
DATA PHASE MUX for AHB Signals

- HREADY
- HRDATA
- HRESP
*/
always @ (*) begin 
	/*
		DPHASE DEFINITION: 
		
		
		DEPHASE -> '0' : CORE
		DEPHASE -> '1' : DEBUGGER
	*/
	if (debug_rst == 1'b1) begin
		HREADY_M1  = HREADY;
        	HREADY_M2 = 1'b0;
        	HRESP_M1   = HRESP;
       	 	HRESP_M2  = 2'd0;
		HWDATA	     = HWDATA_M1;
	end else if (D_PHASE) begin	
		HREADY_M1  = 1'b0;
        	HREADY_M2 = HREADY;
        	HRESP_M1   = 2'd0;
       	 	HRESP_M2  = HRESP;
		HWDATA	     = HWDATA_M2;
	end else begin 
		HREADY_M1  = HREADY;
        	HREADY_M2 = 1'b0;
        	HRESP_M1   = HRESP;
       	 	HRESP_M2  = 2'd0;
		HWDATA	     = HWDATA_M1;
	end

	//always getting the same data, direct mapping
	HRDATA_M1  = HRDATA;
        HRDATA_M2 = HRDATA;
end


/*
ADDRESS PHASE MUX LOGIC - AHB SIGNALS

-HTRANS        -HADDR        -HWRITE
-HPROT         -HSIZE        -HBURST

*/
always @ (*) begin

	if (debug_rst == 1'b1) begin
		HTRANS = HTRANS_M1;
        	HADDR  = HADDR_M1;
        	HWRITE = HWRITE_M1;
        	HPROT  = HPROT_M1;
        	HSIZE  = HSIZE_M1;
        	HBURST = HBURST_M1;
	end else if(D_PHASE == 1'b0 && valid_M2 == 1'b1) begin
		HTRANS = HTRANS_REG_M2;
        	HADDR  = HADDR_REG_M2;
        	HWRITE = HWRITE_REG_M2;
        	HPROT  = HPROT_REG_M2;
        	HSIZE  = HSIZE_REG_M2;
        	HBURST = HBURST_REG_M2;
	end else if(HTRANS_M2 != 2'b00) begin
		HTRANS = HTRANS_M2;
        	HADDR  = HADDR_M2;
        	HWRITE = HWRITE_M2;
        	HPROT  = HPROT_M2;
        	HSIZE  = HSIZE_M2;
        	HBURST = HBURST_M2;
	end else if(D_PHASE == 1'b1 && HTRANS_M2 == 2'b00 && valid_M1 == 1'b1) begin
		HTRANS = HTRANS_REG_M1;
        	HADDR  = HADDR_REG_M1;
        	HWRITE = HWRITE_REG_M1;
        	HPROT  = HPROT_REG_M1;
        	HSIZE  = HSIZE_REG_M1;
        	HBURST = HBURST_REG_M1;
	end else if(HTRANS_M2 == 2'b00) begin
		HTRANS = HTRANS_M1;
        	HADDR  = HADDR_M1;
        	HWRITE = HWRITE_M1;
        	HPROT  = HPROT_M1;
        	HSIZE  = HSIZE_M1;
        	HBURST = HBURST_M1;
	end else begin
		HTRANS = HTRANS_M1;
        	HADDR  = HADDR_M1;
        	HWRITE = HWRITE_M1;
        	HPROT  = HPROT_M1;
        	HSIZE  = HSIZE_M1;
        	HBURST = HBURST_M1;
	end

end 

/*
	D_PHASE tracker
*/
always @ (valid_M2, HTRANS_M2, HREADY, D_PHASE) begin

	if (HREADY) begin
		if ((valid_M2 == 1'b1 || HTRANS_M2 != 2'b00)) begin
			next_D_PHASE = 1'b1;
		end else begin 
			next_D_PHASE = 1'b0;
		end 
	end else begin 
		next_D_PHASE = D_PHASE;
	end

end


/*
                Detect transitions for Core->Debug->Core
      		Core -> Debug 	: rising edge HTRANS_M2	: edge_detect (01)
		Debug -> Core	: falling edge HTRANS_M2     : edge_detect (10)
		 
		 
*/
always @ (HTRANS_M2, prev_HTRANS_M2) begin
        //Core -> Debug
        if (prev_HTRANS_M2 == 2'b00 && HTRANS_M2 != 2'b00) begin         
        	edge_detect = 2'b01; 
	//Debug -> Core
        end else if (HTRANS_M2 == 2'b00 && prev_HTRANS_M2 != 2'b00) begin        
        	edge_detect = 2'b10;
	end else begin
		edge_detect = 2'b00;
	end
end


/*
	CACHE LOGIC
	--monitoring valid_reg_M1 & valid_reg_M2
*/

always @ (edge_detect, D_PHASE, HREADY, valid_M2, valid_M1,HADDR_M2,HTRANS_M2,HWRITE_M2,HPROT_M2,HSIZE_M2,HBURST_M2,HADDR_REG_M2,HTRANS_REG_M2,HWRITE_REG_M2,HPROT_REG_M2,HSIZE_REG_M2,HBURST_REG_M2, HADDR_M1,HTRANS_M1,HWRITE_M1,HPROT_M1,HSIZE_M1,HBURST_M1,HADDR_REG_M1,HTRANS_REG_M1,HWRITE_REG_M1,HPROT_REG_M1,HSIZE_REG_M1,HBURST_REG_M1) begin
	if (edge_detect == 2'b01 && D_PHASE == 1'b0 && HREADY == 1'b0 && valid_M2 == 1'b0) begin 
		next_valid_M2  = 1'b1;
                next_ADDR_M2   = HADDR_M2;
                next_HTRANS_M2 = HTRANS_M2;
                next_HWRITE_M2 = HWRITE_M2;
                next_HPROT_M2  = HPROT_M2;
                next_HSIZE_M2  = HSIZE_M2;
                next_HBURST_M2 = HBURST_M2;
	end else if (valid_M2 == 1'b1 && HREADY == 1'b1) begin 
		next_valid_M2  = 1'b0;
                next_ADDR_M2   = HADDR_M2;
                next_HTRANS_M2 = HTRANS_M2;
                next_HWRITE_M2 = HWRITE_M2;
                next_HPROT_M2  = HPROT_M2;
                next_HSIZE_M2  = HSIZE_M2;
                next_HBURST_M2 = HBURST_M2;
	end else begin 
		next_valid_M2  = valid_M2;
                next_ADDR_M2   = HADDR_REG_M2;
                next_HTRANS_M2 = HTRANS_REG_M2;
                next_HWRITE_M2 = HWRITE_REG_M2;
                next_HPROT_M2  = HPROT_REG_M2;
                next_HSIZE_M2  = HSIZE_REG_M2;
                next_HBURST_M2 = HBURST_REG_M2;
	end

	if (edge_detect == 2'b01 && D_PHASE == 1'b0 && valid_M1 == 1'b0) begin
		next_valid_M1  = 1'b1;
                next_ADDR_M1   = HADDR_M1;
                next_HTRANS_M1 = HTRANS_M1;
                next_HWRITE_M1 = HWRITE_M1;
                next_HPROT_M1  = HPROT_M1;
                next_HSIZE_M1  = HSIZE_M1;
                next_HBURST_M1 = HBURST_M1;
	end else if (D_PHASE == 1'b1 && HREADY == 1'b1 && HTRANS_M2 == 2'b00) begin
		next_valid_M1  = 1'b0;
                next_ADDR_M1   = HADDR_M1;
                next_HTRANS_M1 = HTRANS_M1;
                next_HWRITE_M1 = HWRITE_M1;
                next_HPROT_M1  = HPROT_M1;
                next_HSIZE_M1  = HSIZE_M1;
                next_HBURST_M1 = HBURST_M1;
	end else begin
		next_valid_M1  = valid_M1;
                next_ADDR_M1   = HADDR_REG_M1;
                next_HTRANS_M1 = HTRANS_REG_M1;
                next_HWRITE_M1 = HWRITE_REG_M1;
                next_HPROT_M1  = HPROT_REG_M1;
                next_HSIZE_M1  = HSIZE_REG_M1;
                next_HBURST_M1 = HBURST_REG_M1;
	end
end

endmodule
