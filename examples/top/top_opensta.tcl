# Experimental OpenSTA-subset adapter — example.
# The same 3-inverter design as top_sdc.sta, expressed as an OpenSTA TCL script.
# Run:  vyges-sta-si tcl examples/top/top_opensta.tcl
read_liberty cells.lib
read_verilog top.v
link_design  top
read_spef    top.spef       ;# parasitics -> wire load + Elmore net delay
read_sdc     top.sdc        ;# clock + I/O delays + uncertainty + derate
set_input_transition 0.02 [all_inputs]   ;# match top_sdc.sta's input_slew: 0.02

report_checks -path_delay min_max
report_wns
report_tns
