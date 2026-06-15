// Three-inverter chain: a -> g1 -> g2 -> g3 -> y
module top ( a, y );
  input  a;
  output y;
  wire   n1, n2;
  INV g1 ( .A(a),  .Y(n1) );
  INV g2 ( .A(n1), .Y(n2) );
  INV g3 ( .A(n2), .Y(y)  );
endmodule
