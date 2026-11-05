// SPDX-License-Identifier: Apache-2.0
//
// MODIFIED copy of pipe.v: a chain of 4 delay cells (dlygate4sd3_1) inserted on the
// critical path between x4 and r. Same function, longer path: at the 4.0 ns clock of
// pipe.sdc it fails setup at the slow corner and meets it at the fast one.
//
// Register-to-register sample for the SDF-annotated simulation gate (see README.md).
// sky130_fd_sc_hd gate-level netlist, written by hand so every path is known:
//
//   c0  : toggle flop                 c0 <= ~c0
//   c1  : second counter bit          c1 <= c1 ^ c0
//   r   : capture flop at the end of the datapath
//           x1 = c0 ^ c1 ; x2 = ~x1 ; x3 = x2 ^ c1 ; x4 = buf(x3) ; r <= x4
//
// The datapath x1..x4 is the critical path. Every arc on it is sensitized every cycle
// (an XOR passes a change of either input), so both the rising and the falling arcs the
// timer reports are actually exercised by the simulation.
module pipe_slow (clk, rst_n, q0, q1, r);
  input  clk;
  input  rst_n;
  output q0;
  output q1;
  output r;

  wire c0_n, c1_d, x1, x2, x3, x4;
  wire d0, d1, d2, d3;

  sky130_fd_sc_hd__dfrtp_1 u_c0 (.CLK(clk), .D(c0_n), .RESET_B(rst_n), .Q(q0));
  sky130_fd_sc_hd__inv_1   u_i0 (.A(q0), .Y(c0_n));
  sky130_fd_sc_hd__dfrtp_1 u_c1 (.CLK(clk), .D(c1_d), .RESET_B(rst_n), .Q(q1));
  sky130_fd_sc_hd__xor2_1  u_x0 (.A(q1), .B(q0), .X(c1_d));

  sky130_fd_sc_hd__xor2_1  u_g1 (.A(q0), .B(q1), .X(x1));
  sky130_fd_sc_hd__inv_1   u_g2 (.A(x1), .Y(x2));
  sky130_fd_sc_hd__xor2_1  u_g3 (.A(x2), .B(q1), .X(x3));
  sky130_fd_sc_hd__buf_1   u_g4 (.A(x3), .X(x4));
  sky130_fd_sc_hd__dlygate4sd3_1 u_d0 (.A(x4), .X(d0));
  sky130_fd_sc_hd__dlygate4sd3_1 u_d1 (.A(d0), .X(d1));
  sky130_fd_sc_hd__dlygate4sd3_1 u_d2 (.A(d1), .X(d2));
  sky130_fd_sc_hd__dlygate4sd3_1 u_d3 (.A(d2), .X(d3));
  sky130_fd_sc_hd__dfrtp_1 u_r  (.CLK(clk), .D(d3), .RESET_B(rst_n), .Q(r));
endmodule
