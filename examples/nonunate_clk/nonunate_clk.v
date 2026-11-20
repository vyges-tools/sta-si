// A clock through a positive-unate buffer (cb0) and then a non-unate one (cb1: no timing_sense,
// no function), so both clock edges reach both transitions of ck2, at different times. The
// falling-edge register n2 and the rising-edge register r3 are each launched and captured by
// both edges.
module nonunate_clk (clk, d, y, y3);
  input  clk, d;
  output y, y3;
  wire ck1, ck2, q2, q3;
  CKBUF  cb0 (.A(clk), .Y(ck1));
  CLKBUF cb1 (.A(ck1), .Y(ck2));
  DFFN n2  (.CK(ck2), .D(d),  .Q(q2));
  BUF  g2  (.A(q2),   .Y(y));
  DFF  r3  (.CK(ck2), .D(q2), .Q(q3));
  BUF  g3  (.A(q3),   .Y(y3));
endmodule
