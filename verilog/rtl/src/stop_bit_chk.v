// File name:   stop_bit_chk .v
// Created:     2/28/2013
// Author:      Chuan Yean Tan
// Version:	1.0 
// Description: Checks the stop bit
module stop_bit_chk ( 	input clk,
			input n_Rst,
			input sbc_clear,
			input sbc_enable, 
			input stop_bit,
			output wire framing_error 
			);

//additional signals to detect framing error
reg framing_error_reg;
reg next_framing_error; 

//register block
always @ (posedge clk, negedge n_Rst) begin

	if (n_Rst == 1'b0) begin
		framing_error_reg <= 1'b0;
	end else begin
		framing_error_reg <= next_framing_error;
	end
end

//framing error evaluating block
always @ (stop_bit, sbc_enable, sbc_clear, framing_error_reg) 
begin 
	next_framing_error = framing_error_reg;
	if (sbc_clear == 1'b1) begin
		next_framing_error = 1'b0;
	end else if (sbc_enable == 1'b1) begin
		if (stop_bit == 1'b1) begin
			next_framing_error = 1'b0;
		end else begin
			next_framing_error = 1'b1;
		end
	end //end sbc clear if
end//end always

assign framing_error = framing_error_reg;

endmodule

