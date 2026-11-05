# The capture clock reaches u_b through real cells: time it through them, as a post-CTS
# flow does. With an ideal clock the skew would be invisible to the timer and the hold
# failure the simulation shows would be reported as met.
create_clock -name clk -period 6.0 [get_ports clk]
set_propagated_clock [all_clocks]
set_false_path -from [get_ports rst_n]
