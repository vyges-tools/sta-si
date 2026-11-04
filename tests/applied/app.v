// Fixture for tests/sdc_applied.rs: an input into r1, a register-to-register path r1 -> r2 on
// clk, a path r1 -> r3 onto a second clock clk2, and r4 clocked by rd/Q, a divide-by-two of clk.
module app ( clk, clk2, a, y, z, w );
  input  clk, clk2, a;
  output y, z, w;
  wire na, q1, n1, gclk, nd;
  BUF ga ( .A(a),    .Y(na)   );
  DFF r1 ( .CK(clk), .D(na),  .Q(q1)   );
  INV g1 ( .A(q1),   .Y(n1)   );
  DFF r2 ( .CK(clk), .D(n1),  .Q(y)    );
  DFF r3 ( .CK(clk2), .D(q1), .Q(z)    );
  DFF rd ( .CK(clk), .D(nd),  .Q(gclk) );
  INV gd ( .A(gclk), .Y(nd)   );
  DFF r4 ( .CK(gclk), .D(q1), .Q(w)    );
endmodule
