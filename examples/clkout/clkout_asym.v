// Same as clkout.v, but the forwarding buffer (BUFA) is rise-slow. The falling edge is
// therefore the FASTER one through the cell, so it can only be the critical edge on
// account of when it leaves — half a period after the rising one.
module clkout_asym (clk, d_i, q_o, clk_o);
  input clk, d_i;
  output q_o, clk_o;
  wire cb, dq;
  BUF cb0 (.A(clk), .X(cb));    // clock tree
  DFF r0  (.CLK(cb), .D(d_i), .Q(dq));
  BUFA ob0 (.A(cb),  .X(clk_o)); // forwarded clock  <- the path under test
  BUF ob1 (.A(dq),  .X(q_o));   // ordinary flop-launched output
endmodule
