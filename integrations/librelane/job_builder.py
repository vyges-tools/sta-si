"""Pure, dependency-free helpers that turn LibreLane flow paths into vyges-sta-si
`.sta` job files.

Kept deliberately free of any `librelane` import so it can be unit-tested without the
LibreLane runtime (see ``test_job_builder.py``). The LibreLane Step
(``vyges_sta_si_step.py``) is the thin shell that gathers paths from State/config and
calls these.

EXPERIMENTAL — Layer 2 of the OpenSTA integration (see
``../../docs/opensta-integration.md``).
"""

from typing import List, Optional, Sequence


def sta_scenario(
    *,
    design: str,
    netlist: str,
    libs: Sequence[str],
    sdc: Optional[str] = None,
    spef: Optional[str] = None,
    clock_port: str = "",
    clock_period: float = 0.0,
    miller: float = 2.0,
) -> str:
    """One corner's `.sta` job.

    The clock comes from the SDC when one is given (LibreLane always has a signoff
    SDC with `create_clock`), so no `clock:` line is emitted in that case — it would
    only conflict. A `clock:` line is emitted only as a fallback when no SDC is
    available.
    """
    if not netlist:
        raise ValueError("netlist is required")
    if not libs:
        raise ValueError("at least one liberty file is required")
    lines: List[str] = [f"design: {design}", f"netlist: {netlist}"]
    lines.append("lib: " + ", ".join(libs))
    if spef:
        lines.append(f"spef: {spef}")
    if sdc:
        lines.append(f"sdc: {sdc}")
    elif clock_port and clock_period > 0:
        lines.append(f"clock: {clock_port} {clock_period}")
    else:
        raise ValueError("need an SDC (with create_clock) or a clock_port+clock_period")
    lines.append(f"miller: {miller}")  # SI on (2.0 = worst-case late coupling)
    return "\n".join(lines) + "\n"


def mcmm_master(*, design: str, scenario_files: Sequence[str]) -> str:
    """An MCMM master `.sta` that lists per-corner scenario files; the engine runs
    all and reports the worst setup/hold across them."""
    if not scenario_files:
        raise ValueError("no scenarios")
    return f"design: {design}\nscenarios: {', '.join(scenario_files)}\n"


# Metric keys are namespaced `vyges__…` so they sit *beside* OpenSTA's
# `timing__setup__ws__corner:<c>` rather than clobbering them — the SI second
# opinion is then just a comparison of the two.
def metric_keys(corner: str):
    c = corner
    return {
        "setup_ws": f"vyges__timing__setup__ws__corner:{c}",
        "setup_tns": f"vyges__timing__setup__tns__corner:{c}",
        "hold_ws": f"vyges__timing__hold__ws__corner:{c}",
        "hold_tns": f"vyges__timing__hold__tns__corner:{c}",
    }


def metrics_from_json(corner: str, data: dict) -> dict:
    """Map a `vyges-sta-si run --json` result for one corner to LibreLane metrics."""
    k = metric_keys(corner)
    out = {}
    if data.get("endpoints", 0) > 0 and data.get("wns_ns") is not None:
        out[k["setup_ws"]] = data["wns_ns"]
        out[k["setup_tns"]] = data.get("tns_ns")
    if data.get("hold_endpoints", 0) > 0 and data.get("whs_ns") is not None:
        out[k["hold_ws"]] = data["whs_ns"]
        out[k["hold_tns"]] = data.get("ths_ns")
    return out


def worst_across_corners(metrics: dict) -> dict:
    """Add overall worst (min slack) setup/hold across the per-corner metrics."""
    out = {}
    setup = [v for k, v in metrics.items() if "setup__ws__corner:" in k and v is not None]
    hold = [v for k, v in metrics.items() if "hold__ws__corner:" in k and v is not None]
    if setup:
        out["vyges__timing__setup__ws"] = min(setup)
    if hold:
        out["vyges__timing__hold__ws"] = min(hold)
    return out
