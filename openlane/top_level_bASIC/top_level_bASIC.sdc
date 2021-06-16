


#set_units -capacitance 1fF
#set_units -time 1ps

create_clock -name "wb_clk_i" -period 10 -waveform {0.0 5} [get_ports wb_clk_i]
create_clock -name "user_clock2" -period 10 -waveform {0.0 5} [get_ports user_clock2]
create_clock -name "ck3" -add -period 10 -waveform {0.0 5} [get_ports wb_clk_i]
create_clock -name "ck4" -add -period 10 -waveform {0.0 5} [get_ports user_clock2]
# -group [list \
  [get_clocks wb_clk_i]  \
  [get_clocks ck3] ] -group [list \
  [get_clocks user_clock2]  \
  [get_clocks ck4] ] -pin [get_pins g404/z]
set_clock_groups -physically_exclusive -group {wb_clk_i ck3} -group {user_clock2 ck4}

#set_input_delay -clock [get_clocks *] -add_delay 1 [get_ports *]
#set_output_delay -clock [get_clocks *] -add_delay 1 [get_ports *]
set_max_fanout 30 [get_pins * -filter "direction==out"]
set_max_capacitance 0.2 [get_pins *]
set_max_fanout 30 [get_ports * -filter "direction==in"]
set_max_capacitance 0.2 [get_ports *]
set_false_path -from [get_ports user_clock2] -setup
set_false_path -from [get_ports wb_clk_i] -setup
set_false_path -to [get_pins -hierarchical -filter "lib_pin_name==RESET_B" *] -setup
set_false_path -to [get_pins -hierarchical -filter "lib_pin_name==SET_B" *] -setup
#set_driving_cell -lib_cell inv_8x -pin "X" [get_ports asyncrst_n]
