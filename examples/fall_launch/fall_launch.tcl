# The two registers' checks, one clock of period 2: n1 -> r2 is captured 1.0 after it launched
# on the fall, r3 -> n4 1.0 after it launched on the rise.
read_liberty fall_launch.lib
read_verilog fall_launch.v
link_design fall_launch
create_clock -name clk -period 2.0 [get_ports clk]
report_checks -path_delay max -digits 4
report_checks -path_delay min -digits 4
