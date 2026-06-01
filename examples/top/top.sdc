# Same 3-inverter chain, constrained from SDC instead of inline .sta lines.
create_clock -name clk -period 1.0
set_input_delay   0.10 -clock clk [get_ports a]
set_output_delay  0.15 -clock clk [get_ports y]
set_clock_uncertainty 0.05 -setup
set_timing_derate -late 1.05
