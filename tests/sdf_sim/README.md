# SDF-annotated simulation gate

`scripts/sdf-sim-gate.sh` checks the timer against a simulation that uses the SDF the timer
wrote. For each sample netlist here and each corner of `sky130_fd_sc_hd` (ss_100C_1v60,
tt_025C_1v80, ff_n40C_1v95) it runs `vyges-sta-si` with the sample's SDC, writes the SDF, simulates
the netlist with that SDF at the SDC's clock period, and requires that **the timer and the
simulation agree**: setup and hold slack both >= 0 if and only if every register matches a
behavioural model at every cycle.

It is a box gate, not part of `cargo test`: it needs a sky130 PDK install (`PDK_ROOT`, default
`~/.ciel/sky130A`) and Icarus Verilog, and CI carries neither. With either missing it prints
`SKIP` and exits 77. A skip is not a pass.

```text
./scripts/sdf-sim-gate.sh                                   # the gate
./scripts/sdf-sim-gate.sh case pipe_slow ss_100C_1v60       # one case (0 agree, 1 disagree, 2 inconclusive)
```

## What the simulator does with an SDF

The gate's first stage proves this every run with `probe/`, three cells and an SDF whose delays
are distinct round numbers, so each feature shows up as a time it alone can produce. On the box
(Icarus Verilog 12.0):

| SDF feature | Icarus 12 | evidence in the probe |
|---|---|---|
| `IOPATH` on a combinational cell, rise and fall separately | applied | buf rise 1.0 ns: `n1` at 11.000 |
| `IOPATH CLK Q` onto the model's edge-sensitive `(posedge CLK => Q)` path | applied | fall 3.3 ns: `Q` at 33.300 |
| `INTERCONNECT` | **ignored**, with `SDF WARNING: INTERCONNECT not supported` | D arrives at 13.200, not 13.950 |
| `SETUP` / `HOLD` / `$setuphold` | **not enforced** | D changes 0.2 ns before the edge inside a 0.5 ns setup: captured, no message, no X |
| `$setuphold` delayed signals (`D_delayed`, `CLK_delayed`) | **left undriven** | with the PDK model as shipped the flop's Q never leaves X |

What follows from that:

- **The failure is functional.** With no timing checks, a late data arrival shows up only as a
  register holding the wrong value. `tb.v` compares every register before every clock edge with a
  behavioural model written from the circuit's function — not with a zero-delay simulation of
  the same netlist, which would share its races (a skewed clock in zero delay still races
  through delta cycles).
- **The flop models are patched in the work directory.** Each `wire X_delayed;` in the PDK's
  Verilog gets `assign X_delayed = X;`, which is what a simulator that drives delayed signals does
  when no negative-limit shift applies. The PDK is read from `PDK_ROOT`; nothing from it is copied
  into this repository.
- **No SPEF.** Icarus drops `INTERCONNECT`, so with parasitics the simulation would leave out a
  delay the timer counts. The samples are timed without SPEF: the delays are still the
  Liberty's, at the timer's propagated slews and pin-cap loads. `INTERCONNECT` is not covered by
  this gate.
- **There is a band where they may honestly differ.** The timer requires data to settle a setup
  time before the edge (and stay a hold time after it); a simulator without checks only sees
  which side of the edge the data falls. A slack within the largest `|SETUP|`/`|HOLD|` in the SDF
  is therefore reported `INCONCLUSIVE` and fails the gate — the sample needs retuning, not
  a verdict. Every sample's slacks sit well outside it.
- **Annotation is proved per run.** The testbench measures the launch flop's clock-to-Q; the
  script requires it to equal that instance's `IOPATH CLK Q` in the SDF the simulation was given
  (1 ps tolerance, the simulator's precision). A simulation that silently applied nothing fails
  as `NOT-ANNOTATED` instead of agreeing with a passing timer.

## Samples

| netlist | what it is | SDC |
|---|---|---|
| `pipe.v` | 2-bit counter (`c0` toggles, `c1 <= c1 ^ c0`) feeding a datapath `xor2 -> inv -> xor2 -> buf` into flop `r`; every arc on it is sensitized every cycle | `pipe.sdc`, 4.0 ns |
| `pipe_slow.v` | `pipe.v` with four `dlygate4sd3_1` inserted on the critical path before `r` | `pipe.sdc` |
| `skew_ok.v` | toggle flop `a` into flop `b`, `b`'s clock through one `clkbuf_1` | `skew.sdc`, 6.0 ns, propagated clock |
| `skew_bad.v` | the same with `b`'s clock through four `clkdlybuf4s50_1`: skew exceeds clock-to-Q | `skew.sdc` |

`skew.sdc` sets `set_propagated_clock`: the capture clock goes through real cells and has to be
timed through them, as after CTS. With an ideal clock the timer reports `skew_bad`'s hold as met
while the simulation fails it — the gate reports that as `DISAGREE`, which is the point.

The gate pins the outcomes the samples are built for, so the agreement table has real failures
in it: `pipe` meets timing at every corner; `pipe_slow` fails setup at ss and meets it at ff
(tt is asserted only to agree); `skew_ok` meets timing; `skew_bad` fails hold at every corner.

## The SDF against a reference timer's

Agreement of slack sign cannot see a delay that is wrong by less than a sample's margin. Where
OpenSTA (`sta`, or `REF_STA`) is installed, the gate also has it write an SDF for every case, from
the same netlist, Liberty and SDC, and compares every IOPATH and timing check with ours
(`sdf_compare.py`, 1 ps tolerance). This is the stage that found the writer's per-edge errors:
before the fixes it reported 44 differences over 48 entries on `pipe` at ss (inverter and xor
fall delays 10-25 % low, one setup/hold value for both data edges, no RECOVERY/REMOVAL), and
none after them. Without OpenSTA the stage prints `NOT RUN` and the final PASS line says so.

Known gaps, listed rather than failed:
- `WIDTH`: the shared Liberty reader does not keep `min_pulse_width`, so no WIDTH is written.
- `IOPATH RESET_B Q`: the reference writes the clear arc with a 0 delay; ours omits it. Both
  annotate the same 0.

## Teeth

The agreement check must exit nonzero, for the expected reason, on a wrong SDF:

| mutation | case | expected |
|---|---|---|
| every delay zeroed | `pipe_slow` ss | `DISAGREE` (timer violated, simulation clean) |
| the ff corner's SDF in the ss check | `pipe_slow` ss | `DISAGREE` |
| launch flop clock-to-Q zeroed | `skew_ok` tt | `DISAGREE` (timer met, simulation fails hold) |
| instance names that match nothing | `pipe` ss | `NOT-ANNOTATED` |

## Not covered

- `INTERCONNECT` (ignored by Icarus 12; see above).
- The timing checks themselves (`SETUP`/`HOLD` values in the SDF): never enforced by the
  simulator, so only their sign-off consequence is observable, through the band above.
- The asynchronous reset: released mid-cycle and declared a false path, so neither its
  recovery/removal checks nor its `RESET_B -> Q` delay is exercised.
