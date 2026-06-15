"""LibreLane plugin Step — run ``vyges-sta-si`` as a second-opinion timer in a
LibreLane / OpenROAD flow.

EXPERIMENTAL — Layer 2 of the OpenSTA integration (see
``../../docs/opensta-integration.md``). Unlike the ``vyges-sta-si tcl`` adapter (Layer 1),
this does **no TCL parsing** — it consumes the structured **State** the flow already has
(netlist, per-corner SDC/SPEF, and the per-corner Liberty from config), runs the engine per
corner, and writes ``vyges__…`` metrics *beside* OpenSTA's. The point is the comparison: run
this right after ``OpenROAD.STAPostPNR`` and the **SI + statistical-OCV** delta is simply
``vyges__timing__setup__ws__corner:<c>`` vs the OpenSTA ``timing__setup__ws__corner:<c>``
already in the run — no double work, no flow change, no replacement of the signoff timer.

Usage (a custom flow that appends this step) is in ``README.md``.
"""

import os
import json
from decimal import Decimal
from typing import List, Tuple

from librelane.steps import Step
from librelane.config import Variable
from librelane.state import State, DesignFormat
from librelane.steps.step import ViewsUpdate, MetricsUpdate

from job_builder import sta_scenario, metrics_from_json, worst_across_corners


@Step.factory.register()
class VygesStaSi(Step):
    """Static timing with signal integrity (Vyges), run per corner as a second opinion."""

    id = "Vyges.StaSi"
    name = "Vyges STA-SI (second opinion)"
    long_name = "Vyges static timing analysis with signal integrity"

    # We only *require* the netlist; SDC/SPEF are read opportunistically from State.
    inputs = [DesignFormat.NETLIST]
    outputs = []  # emits metrics + a report, does not mutate design views

    config_vars = [
        Variable(
            "VYGES_STA_SI_BIN",
            str,
            "Path to (or name on $PATH of) the `vyges-sta-si` binary.",
            default="vyges-sta-si",
        ),
        Variable(
            "VYGES_STA_SI_MILLER",
            Decimal,
            "Crosstalk Miller coupling factor passed to the engine "
            "(2.0 = worst-case late coupling; 1.0 disables SI).",
            default=Decimal("2.0"),
        ),
    ]

    def run(self, state_in: State, **kwargs) -> Tuple[ViewsUpdate, MetricsUpdate]:
        binary = self.config["VYGES_STA_SI_BIN"] or "vyges-sta-si"
        miller = float(self.config["VYGES_STA_SI_MILLER"])
        design = self.config["DESIGN_NAME"]

        netlist = state_in[DesignFormat.NETLIST]
        if netlist is None:
            raise Exception("VygesStaSi: no NETLIST in incoming state")
        netlist = str(netlist)

        # SDC: prefer the one the flow propagated; fall back to the configured signoff SDC.
        sdc = state_in.get(DesignFormat.SDC)
        if sdc is None:
            sdc = self.config["SIGNOFF_SDC_FILE"] or self.config["PNR_SDC_FILE"]
        sdc = str(sdc) if sdc else None

        # SPEF in State is a dict keyed by corner (or None pre-extraction).
        spef_by_corner = state_in.get(DesignFormat.SPEF)
        if not isinstance(spef_by_corner, dict):
            spef_by_corner = {}

        corners: List[str] = self.config["STA_CORNERS"] or [self.config["DEFAULT_CORNER"]]

        metrics: MetricsUpdate = {}
        for corner in corners:
            # Per-corner Liberty files (index 1 of the categorized tuple, as OpenROADStep uses).
            _, libs, _, _ = self.toolbox.get_timing_files_categorized(self.config, corner)
            libs = [str(p) for p in (libs or [])]
            if not libs:
                self.warn(f"VygesStaSi: no liberty files for corner {corner!r}; skipping")
                continue
            spef = spef_by_corner.get(corner)
            spef = str(spef) if spef else None

            job_text = sta_scenario(
                design=design,
                netlist=netlist,
                libs=libs,
                sdc=sdc,
                spef=spef,
                clock_port=self.config["CLOCK_PORT"] or "",
                clock_period=float(self.config["CLOCK_PERIOD"] or 0.0),
                miller=miller,
            )
            job_path = os.path.join(self.step_dir, f"{corner}.sta")
            with open(job_path, "w") as f:
                f.write(job_text)

            out_json = os.path.join(self.step_dir, f"{corner}.json")
            # `check=False`: a timing violation is a normal result here (we record it as a
            # metric), not a step failure — the flow's own checkers decide pass/fail.
            self.run_subprocess(
                [binary, "run", job_path, "--json", "-o", out_json],
                log_to=os.path.join(self.step_dir, f"{corner}.log"),
                check=False,
            )
            try:
                with open(out_json) as f:
                    data = json.load(f)
            except (OSError, json.JSONDecodeError) as e:
                self.warn(f"VygesStaSi: corner {corner!r} produced no parseable result: {e}")
                continue
            metrics.update(metrics_from_json(corner, data))

        metrics.update(worst_across_corners(metrics))
        return {}, metrics
