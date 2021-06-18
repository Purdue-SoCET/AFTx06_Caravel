// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

/*--------------------------------------*/

/* User project is instantiated  here   */
/*--------------------------------------*/
top_level_bASIC top_level_bASIC (
        .user_clock2(user_clock2),
	.clk_sel(io_in[0]),
        .asyncrst_n(io_in[1]),
        .uart_debug_rx(io_in[2]),
        .uart_debug_tx(io_out[0]),
        .gpio_bidir_io(io_out[8:1]),
        .gpio_output_en_low(io_oeb[8:1]),
        .pwm_w_data_0(io_out[9]),
        .timer_bidir_0(io_out[10]),
        .timer_output_en_low(io_oeb[10]),
        .SDA_bi(io_out[11]),
	.SDA_output_en_low(io_oeb[11]),
        .SCL_bi(io_out[12]),
	.SCL_output_en_low(io_oeb[12]),
        .SS_bi(io_out[13]),
	.SS_output_en_low(io_oeb[13]),
        .SCK_bi(io_out[14]),
	.SCK_output_en_low(io_oeb[14]),
        .MOSI_bi(io_out[15]),
	.MOSI_output_en_low(io_oeb[15]),
        .MISO_bi(io_out[16]),
	.MISO_output_en_low(io_oeb[16]),
	.wb_clk_i(wb_clk_i),
	.wb_rst_i(wb_rst_i),
	.wbs_stb_i(wbs_stb_i),
	.wbs_cyc_i(wbs_cyc_i),
	.wbs_we_i(wbs_we_i),
	.wbs_sel_i(wbs_sel_i),
	.wbs_dat_i(wbs_dat_i),
	.wbs_adr_i(wbs_adr_i),
	.wbs_ack_o(wbs_ack_o),
	.wbs_dat_o(wbs_dat_o),
	.la_data_out(la_data_out),
	.irq(user_irq)
 );

endmodule	// user_project_wrapper

`default_nettype wire
