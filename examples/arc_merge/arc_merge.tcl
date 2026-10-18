# One gate, two arcs onto the same output edge: in1 -> u1/A fast, in2 -> u1/B with a slow input
# transition. The driver's slew and every wire delay from it are merged to the worse over the arcs,
# each on its own, and each input port keeps its own transition.
read_liberty arc_merge.lib
read_verilog arc_merge.v
link_design top
read_spef arc_merge.spef
create_clock -name clk -period 2
set_input_delay -clock clk 0 {in1 in2}
set_output_delay -clock clk 0 out
set_input_transition 0.02 [get_ports in1]
set_input_transition 0.35 [get_ports in2]
report_checks -path_delay max -fields {slew} -digits 6
report_checks -path_delay min -fields {slew} -digits 6
