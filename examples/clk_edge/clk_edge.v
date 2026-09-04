// One clock tree of asymmetric buffers feeding two rising-edge flops, and a plain data
// path between them. The ONLY asymmetry in the design is CLKBUF's rise-vs-fall delay,
// so any early-vs-late difference at a CLK pin can only have come from the engine
// picking a different EDGE -- there is one corner and no derating.
module clk_edge (clk_i, d_i, q_o);
  input clk_i, d_i;
  output q_o;
  wire c1, c2, dq;
  CLKBUF cb0 (.A(clk_i), .X(c1));
  CLKBUF cb1 (.A(c1),    .X(c2));
  DFF r1 (.CLK(c2), .D(d_i), .Q(dq));
  BUF  b0 (.A(dq),  .X(q_o));
endmodule
