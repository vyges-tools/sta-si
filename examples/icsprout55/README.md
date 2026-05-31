# icsprout55 — 55nm register-to-register timing

First **sub-100nm** `vyges-sta-si` example: a register-to-register path built from
real [icsprout55](https://github.com/openecos-projects/icsprout55-pdk) (open 55nm,
Apache-2.0) standard cells — `DFFQX1H7R` flops and an `INVX1H7R` between them —
analyzed for setup **and** hold against the foundry's NLDM Liberty.

```sh
vyges-sta-si run regreg.sta        # flat OCV
vyges-sta-si run regreg_pocv.sta   # POCV (statistical) OCV
vyges-sta-si run mcmm.sta          # multi-corner sign-off (ss / tt / ff)
```

Typical corner (tt, 1.2 V, 25 °C), 2 ns clock:

| run  | setup WNS | hold WHS |
| ---- | --------- | -------- |
| flat | 1.8273 ns | 0.1113 ns |
| POCV (σ 6%, 3σ) | 1.8015 ns | **0.0906 ns** |

A 3-sigma statistical band trims both margins — the concrete reason advanced-node
sign-off needs OCV, not a flat derate. The launch path is a real `DFFQX1H7R` CK→Q
arc; cell delays *and* setup/hold constraints are bilinear interpolations of the
foundry's NLDM tables (constraints correlated against OpenSTA).

## Multi-corner sign-off (MCMM)

`mcmm.sta` lists three scenarios — the slow (ss), typical (tt), and fast (ff)
corners, each its own extracted lib — and reports the worst setup and worst hold
across them:

```
  scenario      WNS setup     WHS hold   verdict
  ss_slow         1.6931        0.1800   MET
  tt_typ          1.8273        0.1113   MET
  ff_fast         1.8746        0.0750   MET

  worst setup: 1.6931 ns  (ss_slow)   [MET]
  worst hold:  0.0750 ns  (ff_fast)   [MET]
```

This is the textbook corner split: **setup binds at the slow corner** (slowest
paths) and **hold binds at the fast corner** (fastest data) — exactly what MCMM is
for, signing each check off against whichever corner is worst for it. (Corner files
are extracted 2-cell libs; point each scenario's `lib:` at the full per-corner
library from the PDK release for a real block.)

## The Liberty here

`ics55_LLSC_H7CR_inv_dff.lib` contains **only the two cells used**, extracted from
the full 78 MB typical-corner library (7545 timing arcs) so this example is
self-contained. To work with the real PDK, fetch the full per-corner libraries
from the PDK's GitHub releases:

```sh
git clone https://github.com/openecos-projects/icsprout55-pdk
cd icsprout55-pdk && make download unzip   # pulls *_liberty.tar.bz2 release assets
# -> IP/STD_cell/ics55_LLSC_H7C_V1p10C100/ics55_LLSC_H7CR/liberty/*.lib
```

Point `lib:` in the `.sta` at any of those corner files and the same netlist runs
unchanged. The standard-cell library ships NLDM Liberty for all corners; the IO
library (`ICsprout_55LLULP1233_IO`) ships NLDM Liberty too (pad cells use `inout`
pins — those need bidirectional-pin handling that this engine doesn't model yet).
