// File name:   start_bit_det.v
// Created:     2/28/2013
// Author:      Chuan Yean Tan
// Version:	1.0 
// Description: Detects the start bit 

module start_bit_det ( 	input clk,
			input n_Rst,
			input data_in,
			output wire start_detected 
						);

reg Q_1,Q_2,Q_3;

always @ (posedge clk, negedge n_Rst) begin

	if (n_Rst == 1'b0) begin
		Q_1 <= 1'b1;
		Q_2 <= 1'b1; 
		Q_3 <= 1'b1;
	end else begin
		Q_3 <= Q_2;
		Q_2 <= Q_1;
		Q_1 <= data_in;
	end
end

//detecting a start bit
assign start_detected = ~(Q_2) & Q_3;

endmodule

