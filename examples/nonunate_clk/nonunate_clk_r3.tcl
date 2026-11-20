# r3/D alone (the other ends false-pathed), ideal clock. Its setup path is launched by the rise
# edge through cb1's inversion (n2's clock pin falls) and captured by the fall edge (r3's clock
# pin rises), so both clocks are named clk': the reference primes a clock whose pin transition is
# not its edge's (ReportPath::clkName). Its hold is captured by the rise edge on the rise: clk.
# OpenSTA (standalone 4af3c130): setup 0.7833 clk' (fall edge) 0.0000 -> clk' (rise edge) 1.0000;
# hold 0.1467 clk' (fall edge) 0.0000 -> clk (rise edge) 0.0000.
read_liberty nonunate_clk.lib
read_verilog nonunate_clk.v
link_design nonunate_clk
read_sdc nonunate_clk.sdc
set_false_path -to [get_pins n2/D]
set_false_path -to [get_ports {y y3}]
set_input_transition 0.02 [all_inputs]
set_load 0.005 [all_outputs]
report_checks -path_delay max -digits 4 -format full_clock_expanded
report_checks -path_delay min -digits 4 -format full_clock_expanded
