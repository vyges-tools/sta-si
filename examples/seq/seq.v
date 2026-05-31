// Register-to-register path: r1 -> g1 -> r2, captured at r2/D.
module seq ( clk, a, y );
  input  clk, a;
  output y;
  wire   q1, n1;
  DFF r1 ( .CK(clk), .D(a),  .Q(q1) );
  INV g1 ( .A(q1),   .Y(n1) );
  DFF r2 ( .CK(clk), .D(n1), .Q(y)  );
endmodule
