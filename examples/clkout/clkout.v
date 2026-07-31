// A clock forwarded off-chip alongside the data it clocks — the shape of an SRAM
// interface. `clk_o` is an output port whose path STARTS at the clock source, so its
// launch time is a clock edge, not t=0.
module clkout (clk, d_i, q_o, clk_o);
  input clk, d_i;
  output q_o, clk_o;
  wire cb, dq;
  BUF cb0 (.A(clk), .X(cb));    // clock tree
  DFF r0  (.CLK(cb), .D(d_i), .Q(dq));
  BUF ob0 (.A(cb),  .X(clk_o)); // forwarded clock  <- the path under test
  BUF ob1 (.A(dq),  .X(q_o));   // ordinary flop-launched output
endmodule
