"""Example LibreLane flow: the Classic RTL→GDSII flow with ``Vyges.StaSi`` appended
as a trailing second-opinion timing step.

Appending at the end is safe: by then the State has the post-PNR netlist, the
per-corner SPEF, and the signoff SDC — everything the step reads. It writes
``vyges__…`` metrics next to OpenSTA's, so the SI/OCV delta is a metric comparison.

Run (with this directory on PYTHONPATH so the plugin modules import):

    export PYTHONPATH="$PWD:$PYTHONPATH"
    librelane --flow ClassicWithVyges <your-config.json>

EXPERIMENTAL — see ../../docs/opensta-integration.md.
"""

from librelane.flows import Flow, SequentialFlow

import vyges_sta_si_step  # noqa: F401 — importing registers the Vyges.StaSi step


@Flow.factory.register()
class ClassicWithVyges(SequentialFlow):
    """The Classic flow + a trailing Vyges STA-SI second-opinion step."""

    Steps = list(Flow.factory.get("Classic").Steps) + [vyges_sta_si_step.VygesStaSi]
