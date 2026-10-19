# Recovery and removal are checked on the reset's rising (release) edge only: the library gives
# that edge's table and not the other's.
read_liberty one_edge.lib
read_verilog one_edge.v
link_design one_edge
create_clock -name clk -period 2.0 [get_ports clk]
set_input_delay 0.5 -clock clk [get_ports rst_n]
report_checks -path_delay max -digits 4
report_checks -path_delay min -digits 4
