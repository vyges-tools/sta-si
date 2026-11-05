// SPDX-License-Identifier: Apache-2.0
//
// Drives probe.v and prints, for each SDF feature, the time that feature would produce:
//   n1    buf rise IOPATH 1.0 after a rises at 10           -> 11.000 when IOPATH applies
//   d     inv fall IOPATH 2.2 (+0.5 +0.25 INTERCONNECT)     -> 13.200 IOPATH only, 13.950 with
//         INTERCONNECT, measured at the flop's D pin (u_ff.D)
//   qf    CLK->Q fall IOPATH 3.3 after the edge at 30       -> 33.300 when an SDF IOPATH lands
//         on the model's edge-sensitive (posedge CLK => Q) path
//   qr    edge at 53.3 with D changed at 53.1 (IOPATH) -- 0.2 ns before the edge, inside the
//         0.5 ns SETUP. An enforcing simulator reports it and/or sends Q to X; a simulator
//         that does not enforce checks just captures the new value: Q rises at 56.300.
`timescale 1ns/1ps
module tb;
  reg a = 1'b0, clk = 1'b0;
  wire q;
  probe dut (.a(a), .clk(clk), .q(q));
  real n1 = -1.0, d = -1.0, qf = -1.0, qr = -1.0;
  integer qx = 0;
  always @(dut.n1) if ($realtime > 5.0 && dut.n1 === 1'b1 && n1 < 0.0) n1 = $realtime;
  always @(dut.u_ff.D) if ($realtime > 5.0 && dut.u_ff.D === 1'b0 && d < 0.0) d = $realtime;
  always @(q) begin
    if (q === 1'b0 && qf < 0.0 && $realtime > 30.0) qf = $realtime;
    if (q === 1'b1 && qr < 0.0 && $realtime > 53.3) qr = $realtime;
    if (q === 1'bx && $realtime > 30.0) qx = qx + 1;
  end
  initial begin
`ifdef SDF_FILE
    $sdf_annotate(`SDF_FILE, dut);
`endif
    #10   a = 1'b1;
    #20   clk = 1'b1;     // t=30
    #10   clk = 1'b0;     // t=40
    #10   a = 1'b0;       // t=50: n1 falls 51.1, n2 rises 53.1
    #3.3  clk = 1'b1;     // t=53.3
    #10   clk = 1'b0;
    #10   $display("PROBE n1=%0.3f d=%0.3f qf=%0.3f qr=%0.3f qx=%0d", n1, d, qf, qr, qx);
    $finish;
  end
endmodule
