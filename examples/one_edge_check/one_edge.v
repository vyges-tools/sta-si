// One reset register; its reset through a buffer slower falling (0.30) than rising (0.05).
module one_edge (clk, rst_n, d, q);
  input clk, rst_n, d;
  output q;
  wire rb;
  BUF  b0 (.A(rst_n), .X(rb));
  DFFR r0 (.CLK(clk), .D(d), .RESET_B(rb), .Q(q));
endmodule
