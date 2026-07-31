// Two things a tie cell must not break:
//   r_tied/D  is driven straight from a tie cell -> never switches -> no check at all
//   g0        has ONE constant input and one real one -> must still time through the real one
module tie (clk, d_i, rst_n_i, q_o, q_tied_o);
  input clk, d_i, rst_n_i;
  output q_o, q_tied_o;
  wire hi, lo, dbuf, rbuf, gout;
  CONB t0 (.HI(hi), .LO(lo));
  BUF b0 (.A(d_i), .X(dbuf));
  BUF b1 (.A(rst_n_i), .X(rbuf));
  AND2 g0 (.A(dbuf), .B(hi), .X(gout));
  DFRTP r1 (.CLK(clk), .D(gout), .RESET_B(rbuf), .Q(q_o));
  DFRTP r_tied (.CLK(clk), .D(lo), .RESET_B(rbuf), .Q(q_tied_o));
endmodule
