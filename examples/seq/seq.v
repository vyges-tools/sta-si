// Two-flop ring: r1.Q -[INV g1]-> r2.D, and r2.Q (= primary output y) -> r1.D.
// Both flop D pins are launched by a flop Q, so the setup and hold paths are
// both register-to-register.
module seq ( clk, y );
  input  clk;
  output y;
  wire q1, n1;
  DFF r1 ( .CK(clk), .D(y),  .Q(q1) );
  INV g1 ( .A(q1),   .Y(n1) );
  DFF r2 ( .CK(clk), .D(n1), .Q(y)  );
endmodule
