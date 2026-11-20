create_clock -name clk -period 2 [get_ports clk]
set_clock_transition 0.08 [get_clocks clk]
set_propagated_clock [get_clocks clk]
set_input_delay -0.3 -clock clk [get_ports d]
set_output_delay -1.0 -max -clock clk [get_ports {y y3}]
set_output_delay -1.3 -min -clock clk [get_ports {y y3}]
set_timing_derate -early 0.9
set_timing_derate -late 1.1
