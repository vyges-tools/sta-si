module top (in1, in2, out);
  input in1, in2;
  output out;
  wire y;
  AND2 u1 (.A(in1), .B(in2), .Y(y));
  BUF u2 (.A(y), .Y(out));
endmodule
