// SPDX-License-Identifier: Apache-2.0
//
// Hold-side sample for the SDF-annotated simulation gate (see README.md).
// Launch flop a toggles (a <= ~a); capture flop b samples it directly (b <= a), but b's
// clock arrives through a chain of 4 clock delay cells (clkdlybuf4s50_1). The data path is
// one clock-to-Q; the capture clock is late by the chain delay. When that skew exceeds
// clock-to-Q, b takes the NEW value of a in the same cycle: a hold failure, and a wrong value.
// skew_bad: 4 delay cells, more skew than clock-to-Q at every corner: hold fails.
module skew_bad (clk, rst_n, qa, qb);
  input  clk;
  input  rst_n;
  output qa;
  output qb;

  wire a_n;
  wire ck0, ck1, ck2, ck3;

  sky130_fd_sc_hd__dfrtp_1 u_a (.CLK(clk), .D(a_n), .RESET_B(rst_n), .Q(qa));
  sky130_fd_sc_hd__inv_1   u_ia (.A(qa), .Y(a_n));
  sky130_fd_sc_hd__clkdlybuf4s50_1 u_ck0 (.A(clk), .X(ck0));
  sky130_fd_sc_hd__clkdlybuf4s50_1 u_ck1 (.A(ck0), .X(ck1));
  sky130_fd_sc_hd__clkdlybuf4s50_1 u_ck2 (.A(ck1), .X(ck2));
  sky130_fd_sc_hd__clkdlybuf4s50_1 u_ck3 (.A(ck2), .X(ck3));
  sky130_fd_sc_hd__dfrtp_1 u_b (.CLK(ck3), .D(qa), .RESET_B(rst_n), .Q(qb));
endmodule
