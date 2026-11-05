// SPDX-License-Identifier: Apache-2.0
//
// Three-cell probe of what the simulator does with an SDF (see ../README.md). Every delay in
// probe.sdf is a distinct round number, so each feature shows up as a time that can only be
// produced if that feature was applied.
module probe (a, clk, q);
  input  a;
  input  clk;
  output q;
  wire n1, n2;
  sky130_fd_sc_hd__buf_1   u_buf (.A(a), .X(n1));
  sky130_fd_sc_hd__inv_1   u_inv (.A(n1), .Y(n2));
  sky130_fd_sc_hd__dfxtp_1 u_ff  (.CLK(clk), .D(n2), .Q(q));
endmodule
