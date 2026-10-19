// n1 (falling edge) launches into r2 (rising); r3 (rising) launches through two buffers into
// n4 (falling); both meet at a5 into r5 (rising), the rising launch the later relative to its
// own edge. Every register on the one clock.
module fall_launch (clk, d1, d3);
  input clk, d1, d3;
  wire q1, x1, q3, x3a, x3b, y5;
  DFFN n1 (.CLK(clk), .D(d1), .Q(q1));
  BUF  b1 (.A(q1), .X(x1));
  DFF  r2 (.CLK(clk), .D(x1));
  DFF  r3 (.CLK(clk), .D(d3), .Q(q3));
  BUF  b3a (.A(q3), .X(x3a));
  BUF  b3b (.A(x3a), .X(x3b));
  DFFN n4 (.CLK(clk), .D(x3b));
  AND2 a5 (.A(x1), .B(x3b), .X(y5));
  DFF  r5 (.CLK(clk), .D(y5));
endmodule
