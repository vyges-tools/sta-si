// SPDX-License-Identifier: Apache-2.0
//
// Hold-side sample for the SDF-annotated simulation gate (see README.md), passing variant.
// Launch flop a toggles (a <= ~a); capture flop b samples it directly (b <= a) on a clock
// delayed by ONE ordinary clock buffer (clkbuf_1). That skew is smaller than a's
// clock-to-Q at every corner, so b still takes the OLD value of a: hold met.
// skew_bad.v is the same circuit with a long clock delay chain.
module skew_ok (clk, rst_n, qa, qb);
  input  clk;
  input  rst_n;
  output qa;
  output qb;

  wire a_n;
  wire ck0;

  sky130_fd_sc_hd__dfrtp_1 u_a (.CLK(clk), .D(a_n), .RESET_B(rst_n), .Q(qa));
  sky130_fd_sc_hd__inv_1   u_ia (.A(qa), .Y(a_n));
  sky130_fd_sc_hd__clkbuf_1 u_ck0 (.A(clk), .X(ck0));
  sky130_fd_sc_hd__dfrtp_1 u_b (.CLK(ck0), .D(qa), .RESET_B(rst_n), .Q(qb));
endmodule
