// A flop with an async reset, plus a buffer on each of the data and reset paths so
// both arrive with real (and different) delays rather than straight off a port.
module async_rst (clk, d_i, rst_n_i, q_o);
  input clk, d_i, rst_n_i;
  output q_o;
  wire dbuf, rbuf;
  BUF b0 (.A(d_i), .X(dbuf));
  BUF b1 (.A(rst_n_i), .X(rbuf));
  DFRTP r1 (.CLK(clk), .D(dbuf), .RESET_B(rbuf), .Q(q_o));
endmodule
