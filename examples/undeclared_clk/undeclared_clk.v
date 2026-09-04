// Two flops, two clock trees, ONE declared clock -- the shape of fft_top, whose SDC
// creates a clock on clk_i and leaves pclk_i an ordinary data input carrying a large
// set_input_delay. r2's clock tree is DEEPER than its data path, so a capture arrival
// taken from that undeclared tree lands later than the data and manufactures a hold
// violation out of nothing. That is the failure this fixture exists to catch.
module undeclared_clk (clk_i, pclk_i, d_i, rst_n_i, pd_i, prst_n_i, q_o, pq_o);
  input clk_i, pclk_i, d_i, rst_n_i, pd_i, prst_n_i;
  output q_o, pq_o;
  wire cbuf, pbuf, pbuf2, dbuf, rbuf, pdbuf, prbuf;
  BUF b0 (.A(clk_i),    .X(cbuf));
  BUF b1 (.A(pclk_i),   .X(pbuf));
  BUF b2 (.A(pbuf),     .X(pbuf2));   // deeper than the data path below
  BUF b3 (.A(d_i),      .X(dbuf));
  BUF b4 (.A(rst_n_i),  .X(rbuf));
  BUF b5 (.A(pd_i),     .X(pdbuf));
  BUF b6 (.A(prst_n_i), .X(prbuf));
  DFRTP r1 (.CLK(cbuf),  .D(dbuf),  .RESET_B(rbuf),  .Q(q_o));
  DFRTP r2 (.CLK(pbuf2), .D(pdbuf), .RESET_B(prbuf), .Q(pq_o));
endmodule
