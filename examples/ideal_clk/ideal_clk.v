// An ideal clock through two clock buffers to a falling-edge register n2 and a rising-edge
// register r3. The clock's set_clock_transition is the slew at both clock pins only if it is
// carried through cb0 and cb1. n2 -> r3, launched on the fall and captured on the rise half a
// period later, takes its CRPR credit at ck2 on the edge n2 is launched by: the clock buffers
// rise slowly and fall fast, so the rise and fall spreads differ.
module ideal_clk (clk, d, y);
  input  clk, d;
  output y;
  wire ck1, ck2, q2;
  CLKBUF cb0 (.A(clk), .Y(ck1));
  CLKBUF cb1 (.A(ck1), .Y(ck2));
  DFFN n2  (.CK(ck2), .D(d),  .Q(q2));
  BUF  g2  (.A(q2),   .Y(y));
  DFF  r3  (.CK(ck2), .D(q2));
endmodule
