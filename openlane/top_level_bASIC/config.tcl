# SPDX-FileCopyrightText: 2020 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

set script_dir [file dirname [file normalize [info script]]]

set ::env(DESIGN_NAME) top_level_bASIC

set ::env(VERILOG_FILES) "\
        $script_dir/../../caravel/verilog/rtl/defines.v \
        $script_dir/../../verilog/rtl/top_level_bASIC.v"

##Add SRAM collaterals over here
set ::env(VERILOG_FILES_BLACKBOX) "\
        $script_dir/sky130_sram_2kbyte_1rw1r_32x512_8.v"

set ::env(EXTRA_LEFS) "\
	$script_dir/sky130_sram_2kbyte_1rw1r_32x512_8.lef"

set ::env(EXTRA_GDS_FILES) "$script_dir/sky130_sram_2kbyte_1rw1r_32x512_8.gds"

set ::env(CLOCK_PORT) {g404/z}
set ::env(CLOCK_NET) {clk}
set ::env(CLOCK_PERIOD) "20"

set ::env(FP_SIZING) absolute
set ::env(DIE_AREA) "0 0 2700 3300"
set ::env(DESIGN_IS_CORE) 0

#set ::env(VDD_NETS) [list {vccd1} {vccd2} {vdda1} {vdda2}]
#set ::env(GND_NETS) [list {vssd1} {vssd2} {vssa1} {vssa2}]

set ::env(VDD_NETS) [list {vccd1} {vccd2}]
set ::env(GND_NETS) [list {vssd1} {vssd2}]

set ::env(FP_PIN_ORDER_CFG) $script_dir/pin_order_new.cfg
set ::env(MACRO_PLACEMENT_CFG) $script_dir/macro_placement.cfg; 

#set ::env(PL_BASIC_PLACEMENT) 1 ; Commented since this reduces the iterations and results in lot of detailed placement failures

# If you're going to use multiple power domains, then keep this disabled.
set ::env(RUN_CVC) 1

#Extra settings
set ::env(FP_CORE_UTIL) "30"
set ::env(FP_PDN_CORE_RING) 0
set ::env(GLB_RT_MAXLAYER) 5
set ::env(FP_PDN_CHECK_NODES) 0
set ::env(PL_TARGET_DENSITY) "0.33"
set ::env(RUN_CVC) 0
set ::env(RUN_KLAYOUT_XOR) 0
set ::env(RUN_KLAYOUT_DRC) 0
set ::env(QUIT_ON_TR_DRC) "0" 
set ::env(QUIT_ON_MAGIC_DRC) "0"
set ::env(QUIT_ON_ILLEGAL_OVERLAPS) "0"
set ::env(QUIT_ON_LVS_ERROR) "0"
set ::env(BASE_SDC_FILE) "$script_dir/top_level_bASIC.sdc"
set ::env(DIODE_INSERTION_STRATEGY) 5
set ::env(ROUTING_CORES) "8"
#set ::env(MAGIC_EXT_USE_GDS) "1"
