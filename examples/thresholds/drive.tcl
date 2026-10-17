# A propagated clock driven through a driving cell: the port takes the cell's slew and each
# wire from it the cell's load-dependent delay (the reference to the last digit).
read_liberty th_a.lib
read_liberty th_b.lib
read_verilog seq_th.v
link_design seq
create_clock -name clk -period 1.0 [get_ports clk]
set_propagated_clock [all_clocks]
set_driving_cell -lib_cell INV -pin Y [get_ports clk]
set_load 0.005 [all_outputs]
read_spef seq.spef
report_checks -path_delay max -fields {slew} -digits 4 -format full_clock_expanded
report_checks -path_delay min -fields {slew} -digits 4 -format full_clock_expanded
