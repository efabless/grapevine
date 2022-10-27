#current_design user_proj_example

set ::env(IO_PCT) "0.2"
set ::env(SYNTH_MAX_FANOUT) "5"
set ::env(SYNTH_CAP_LOAD) "33"
set ::env(SYNTH_TIMING_DERATE) 0.05
set ::env(SYNTH_CLOCK_UNCERTAINITY) 0.25
set ::env(SYNTH_CLOCK_TRANSITION) 0.15

set_max_fanout 20  [current_design]
set_max_transition 25  [current_design]

set input_delay_value 3
set output_delay_value 0
puts "\[INFO\]: Setting output delay to: $output_delay_value"
puts "\[INFO\]: Setting input delay to: $input_delay_value"

#set_clock_latency -source -early -min -rise  -1.59392 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -early -min -fall  -1.56861 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -early -max -rise  -1.59392 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -early -max -fall  -1.56861 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -late -min -rise  -1.59392 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -late -min -fall  -1.56861 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -late -max -rise  -1.59392 [get_pins {mprj/wb_clk_i}] -clock clk
#set_clock_latency -source -late -max -fall  -1.56861 [get_pins {mprj/wb_clk_i}] -clock clk

set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[1]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[2]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[3]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[4]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[5]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[6]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[7]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[8]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[9]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[10]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[11]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[12]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[13]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[14]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[15]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[16]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[17]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[18]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[19]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[20]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[21]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[22]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[23]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[24]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[25]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[25]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[26]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[27]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[28]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[29]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[30]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[31]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[32]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[33]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[34]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[35]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[36]}]
set_input_delay $input_delay_value  -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[37]}]

set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[0]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[1]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[2]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[3]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[4]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[5]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[6]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[7]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[8]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[9]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[10]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[11]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[12]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[13]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[14]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[15]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[16]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[17]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[18]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[19]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[20]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[21]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[22]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[23]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[24]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[26]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[27]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[28]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[29]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[30]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[31]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[32]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[33]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[34]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[35]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[36]}]
set_output_delay $output_delay_value -clock [get_clocks {clk}] -add_delay [get_ports {mprj_io[37]}]

set_clock_latency -source -early -min -rise  -1.59392 [get_ports {clock}] -clock clk
set_clock_latency -source -early -min -fall  -1.56861 [get_ports {clock}] -clock clk
set_clock_latency -source -early -max -rise  -1.59392 [get_ports {clock}] -clock clk
set_clock_latency -source -early -max -fall  -1.56861 [get_ports {clock}] -clock clk
set_clock_latency -source -late -min -rise  -1.59392 [get_ports {clock}] -clock clk
set_clock_latency -source -late -min -fall  -1.56861 [get_ports {clock}] -clock clk
set_clock_latency -source -late -max -rise  -1.59392 [get_ports {clock}] -clock clk
set_clock_latency -source -late -max -fall  -1.56861 [get_ports {clock}] -clock clk

# set cap_load [expr $::env(SYNTH_CAP_LOAD) / 1000.0]
# total capacitance load = 1.1pF caravel QFN pkg + 3pF pcb + 5pF fpga
set cap_load 9

puts "\[INFO\]: Setting load to: $cap_load"
set_load  $cap_load [all_outputs]

#puts "\[INFO\]: Setting timing derate to: [expr {$::env(SYNTH_TIMING_DERATE) * 10}] %"
#set_timing_derate -early [expr {1-$::env(SYNTH_TIMING_DERATE)}]
#set_timing_derate -late [expr {1+$::env(SYNTH_TIMING_DERATE)}]

#puts "\[INFO\]: Setting clock uncertainity to: $::env(SYNTH_CLOCK_UNCERTAINITY)"
set_clock_uncertainty $::env(SYNTH_CLOCK_UNCERTAINITY) [get_clocks {clock}]

#puts "\[INFO\]: Setting clock transition to: $::env(SYNTH_CLOCK_TRANSITION)"
set_clock_transition $::env(SYNTH_CLOCK_TRANSITION) [get_clocks {clock}]