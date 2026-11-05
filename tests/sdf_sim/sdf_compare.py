#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compare an SDF we wrote against a reference timer's SDF for the same design and corner.

    sdf_compare.py OURS.sdf REF.sdf [--tol NS] [--known KIND]...

Exit 0 when every delay and every timing check in the reference is in ours with the same
value (within --tol, default 0.001 ns), 1 otherwise. Differences are printed one per line.
--known KIND (e.g. WIDTH) names a check kind this writer is known not to emit: reference
entries of that kind missing from ours are counted and printed as "known", not failed. Ours
writing one would still be compared.

What is compared, and how:
  * IOPATH, per (instance, from pin, to pin). A cell with several timing groups on one pin pair
    (sky130's xor2: one positive- and one negative-unate group per input) writes several
    IOPATHs for it, in each writer's own arc order, so they are compared as a sorted list of
    (rise, fall) pairs. A value is the triple's MAX field (the late, setup-side delay; ours is
    one corner per file and writes min = typ = max). An empty "()" is None.
  * TIMINGCHECK, per (instance, check, data port spec, clock port spec), e.g.
    (u_c0, SETUP, "(negedge D)", "(posedge CLK)"). Compared by MAX field as well.
  * INTERCONNECT is not compared: without SPEF the reference writes every net at 0.
  * A reference IOPATH whose values are all zero or empty (sky130's RESET_B -> Q clear arc,
    which the reference times as zero by default) is optional: its absence is reported as a
    note, not a difference, because annotating a 0 and annotating nothing are the same delay.
"""
import re
import sys


def parse(path):
    text = open(path).read()
    iopath, checks = {}, {}
    inst = None
    for line in text.splitlines():
        m = re.search(r"\(INSTANCE\s*([^)]*)\)", line)
        if m:
            inst = m.group(1).strip()
            continue
        m = re.search(r"\(IOPATH\s+(\S+)\s+(\S+)\s+(.*)$", line)
        if m:
            vals = [last(v) for v in re.findall(r"\(([^()]*)\)", m.group(3))]
            if len(vals) == 1:  # one triple: rise and fall share it
                vals = vals * 2
            iopath.setdefault((inst, m.group(1), m.group(2)), []).append(tuple(vals[:2]))
            continue
        m = re.search(r"\((SETUP|HOLD|RECOVERY|REMOVAL|WIDTH|PERIOD)\s+(.*)$", line)
        if m:
            kind, rest = m.group(1), m.group(2)
            ports = []
            # a port spec is either a bare name or "(posedge NAME)" / "(negedge NAME)"
            for pm in re.finditer(r"\((?:posedge|negedge)\s+[^()\s]+\)|[A-Za-z_][\w\[\]/.]*", rest):
                ports.append(pm.group(0))
                if len(ports) == (1 if kind in ("WIDTH", "PERIOD") else 2):
                    break
            tail = rest[pm.end():]
            val = re.search(r"\(([^()]*)\)", tail)
            checks[(inst, kind) + tuple(ports)] = last(val.group(1)) if val else None
    return iopath, checks


def last(triple):
    """The MAX field of an SDF triple ('a:b:c', 'a::c' or a bare 'v'); None for '()'."""
    fields = [f for f in triple.split(":") if f.strip() != ""]
    return float(fields[-1]) if fields else None


def close(a, b, tol):
    if a is None or b is None:
        return a is None and b is None
    return abs(a - b) <= tol


def main():
    args = sys.argv[1:]
    tol = 0.001
    known = set()
    while "--tol" in args or "--known" in args:
        for flag in ("--tol", "--known"):
            if flag in args:
                i = args.index(flag)
                if flag == "--tol":
                    tol = float(args[i + 1])
                else:
                    known.add(args[i + 1])
                del args[i:i + 2]
    if len(args) != 2:
        print(__doc__)
        return 2
    ours_io, ours_ck = parse(args[0])
    ref_io, ref_ck = parse(args[1])
    bad = 0
    gaps = 0
    for key in sorted(ref_io, key=str):
        ref = ref_io[key]
        if key not in ours_io:
            if all(v in (None, 0.0) for pair in ref for v in pair):
                print(f"note  IOPATH {' '.join(key)}: reference writes only zero/empty {ref}, ours omits it")
            else:
                print(f"DIFF  IOPATH {' '.join(key)}: missing; reference {ref}")
                bad += 1
            continue
        ours = ours_io[key]
        srt = lambda xs: sorted(xs, key=lambda p: tuple(-1 if v is None else v for v in p))
        if len(ours) != len(ref) or not all(
            close(a, b, tol) for po, pr in zip(srt(ours), srt(ref)) for a, b in zip(po, pr)
        ):
            print(f"DIFF  IOPATH {' '.join(key)}: ours {srt(ours)} reference {srt(ref)}")
            bad += 1
    for key in sorted(set(ours_io) - set(ref_io), key=str):
        print(f"DIFF  IOPATH {' '.join(key)}: ours only {ours_io[key]}")
        bad += 1
    for key in sorted(ref_ck, key=str):
        if key not in ours_ck and key[1] in known:
            print(f"known {' '.join(key)}: not written by this writer; reference {ref_ck[key]}")
            gaps += 1
        elif key not in ours_ck:
            print(f"DIFF  {' '.join(key)}: missing; reference {ref_ck[key]}")
            bad += 1
        elif not close(ours_ck[key], ref_ck[key], tol):
            print(f"DIFF  {' '.join(key)}: ours {ours_ck[key]} reference {ref_ck[key]}")
            bad += 1
    for key in sorted(set(ours_ck) - set(ref_ck), key=str):
        print(f"DIFF  {' '.join(key)}: ours only {ours_ck[key]}")
        bad += 1
    n = sum(len(v) for v in ref_io.values()) + len(ref_ck)
    print(f"{'SAME' if bad == 0 else 'DIFFER'}: {bad} difference(s) over {n} reference entries"
          f" ({gaps} known gap(s){': ' + ', '.join(sorted(known)) if known else ''})")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
