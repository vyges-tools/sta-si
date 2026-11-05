// SPDX-License-Identifier: Apache-2.0
//
// Testbench for the SDF-annotated simulation gate (see README.md).
//
// Icarus Verilog 12 does not enforce $setup/$hold/$setuphold, so a timing violation can never
// show up here as a timing-check message. It shows up FUNCTIONALLY instead: a flop whose data
// arrives after its clock edge keeps the old value, and one whose data changes before its
// (skewed) clock edge takes the new value. Either way a register holds the wrong value, and
// this testbench catches it by comparing every register, every cycle, against a behavioural
// model of the same circuit written from its function -- not from another simulation of the
// netlist, which would share the netlist's races.
//
// Defines (all set by scripts/sdf-sim-gate.sh):
//   DUT        module under test: pipe, pipe_slow, skew_ok or skew_bad
//   KIND_PIPE  or KIND_SKEW, selecting the reference model
//   PERIOD     clock period in ns -- the SDC's
//   SDF_FILE   "path" of the SDF to annotate; absent = no annotation
//
// Output, one line, parsed by the script:
//   SIM cycles=<n> mismatches=<n> launch_rise=<ns> launch_fall=<ns>
// launch_rise / launch_fall are the measured clock-to-Q of the launch flop (u_c0 or u_a, both
// clocked straight from the clk port). The script checks them against the SDF's IOPATH for
// that instance: that is the proof the annotation was actually applied in THIS run.
`timescale 1ns/1ps
module tb;
  localparam real P = `PERIOD;
  localparam real EPS = 0.010;      // registers are compared this long before each edge
  localparam integer CYCLES = 32;

  reg clk = 1'b0;
  reg rst_n = 1'b0;

`ifdef KIND_PIPE
  wire q0, q1, r;
  `DUT dut (.clk(clk), .rst_n(rst_n), .q0(q0), .q1(q1), .r(r));
  wire launch = q0;
  reg m0, m1, mr;                   // reference model state
  task model_reset; begin m0 = 0; m1 = 0; mr = 0; end endtask
  task model_step;                  // what the circuit computes on one rising edge
    reg n0, n1, nr;
    begin
      n0 = ~m0;
      n1 = m1 ^ m0;
      nr = (~(m0 ^ m1)) ^ m1;       // x1 = c0^c1 ; x2 = ~x1 ; x3 = x2^c1 ; r <= x3
      m0 = n0; m1 = n1; mr = nr;
    end
  endtask
  function integer model_diff; input dummy;
    model_diff = (q0 !== m0) + (q1 !== m1) + (r !== mr);
  endfunction
`elsif KIND_SKEW
  wire qa, qb;
  `DUT dut (.clk(clk), .rst_n(rst_n), .qa(qa), .qb(qb));
  wire launch = qa;
  reg ma, mb;
  task model_reset; begin ma = 0; mb = 0; end endtask
  task model_step;
    reg na, nb;
    begin
      na = ~ma;
      nb = ma;                      // b takes a's value from BEFORE the edge
      ma = na; mb = nb;
    end
  endtask
  function integer model_diff; input dummy;
    model_diff = (qa !== ma) + (qb !== mb);
  endfunction
`else
  initial begin $display("SIM error: define KIND_PIPE or KIND_SKEW"); $finish; end
`endif

  // clock-to-Q of the launch flop, measured from the most recent rising clk edge
  real t_edge = 0.0, launch_rise = -1.0, launch_fall = -1.0;
  always @(posedge clk) t_edge = $realtime;
  always @(launch) if (rst_n === 1'b1) begin
    if (launch === 1'b1 && launch_rise < 0.0) launch_rise = $realtime - t_edge;
    if (launch === 1'b0 && launch_fall < 0.0) launch_fall = $realtime - t_edge;
  end

  // Reset is held over edge 1 and released a quarter period after it, far from any edge: the
  // release is not a timed path in this gate (see the SDCs).
  initial #(1.25 * P) rst_n = 1'b1;

  integer k, mismatches = 0, cycles = 0, d;
  initial begin
`ifdef SDF_FILE
    $sdf_annotate(`SDF_FILE, dut);
`endif
    model_reset;
    // edge k is at k*P
    for (k = 1; k <= CYCLES; k = k + 1) begin
      #(P / 2.0) clk = 1'b0;
      #(P / 2.0 - EPS);
      // just before edge k: every register must hold what the model says
      if (k >= 2) begin
        d = model_diff(0);
        if (d != 0) begin
          mismatches = mismatches + 1;
          if (mismatches <= 4) $display("SIM mismatch before edge %0d at %0.3f", k, $realtime);
        end
        cycles = cycles + 1;
      end
      #(EPS) clk = 1'b1;
      if (k >= 2) model_step;
    end
    $display("SIM cycles=%0d mismatches=%0d launch_rise=%0.4f launch_fall=%0.4f",
             cycles, mismatches, launch_rise, launch_fall);
    $finish;
  end
endmodule
