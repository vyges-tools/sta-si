// 55nm register-to-register path on the icsprout55 PDK (RVT std cells).
// r1.Q -[INV g1]-> r2.D ; r2.Q (= output y) -> r1.D — both flop D pins are
// launched by a flop Q, exercising setup AND hold on real foundry timing.
module ics55_regreg ( clk, y );
  input  clk;
  output y;
  wire q1, n1;
  DFFQX1H7R r1 ( .CK(clk), .D(y),  .Q(q1) );
  INVX1H7R  g1 ( .A(q1),   .Y(n1) );
  DFFQX1H7R r2 ( .CK(clk), .D(n1), .Q(y)  );
endmodule
