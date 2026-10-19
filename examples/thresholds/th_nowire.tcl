# th.tcl with no parasitics: each load's slew is still re-expressed at its own library's thresholds
read_liberty th_a.lib
read_liberty th_b.lib
read_verilog seq_th.v
link_design seq
create_clock -name clk -period 1.0 [get_ports clk]
set_input_transition 0.02 [all_inputs]
set_load 0.005 [all_outputs]
report_checks -path_delay max -fields {slew} -digits 4
report_checks -path_delay min -fields {slew} -digits 4
