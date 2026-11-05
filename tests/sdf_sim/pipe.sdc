# Chosen so pipe.v meets setup at every corner and pipe_slow.v fails it at the slow corner
# while still meeting it at the fast one. The reset input is asynchronous and released by
# the testbench mid-cycle, far from any clock edge: it is not a timed path here.
create_clock -name clk -period 4.0 [get_ports clk]
set_false_path -from [get_ports rst_n]
