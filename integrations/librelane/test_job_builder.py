"""Standalone tests for the pure job-builder core — no LibreLane needed.

  python3 test_job_builder.py
"""

from job_builder import sta_scenario, mcmm_master, metrics_from_json, worst_across_corners


def test_scenario_with_sdc():
    t = sta_scenario(
        design="top", netlist="top.nl.v", libs=["a.lib", "b.lib"],
        sdc="top.sdc", spef="ss.spef", miller=2.0,
    )
    assert "design: top" in t
    assert "netlist: top.nl.v" in t
    assert "lib: a.lib, b.lib" in t
    assert "sdc: top.sdc" in t
    assert "spef: ss.spef" in t
    assert "clock:" not in t  # the SDC supplies the clock
    assert "miller: 2.0" in t


def test_scenario_fallback_clock():
    t = sta_scenario(design="top", netlist="t.v", libs=["a.lib"], clock_port="clk", clock_period=5.0)
    assert "clock: clk 5.0" in t
    assert "sdc:" not in t


def test_requires_clock_source():
    try:
        sta_scenario(design="t", netlist="t.v", libs=["a.lib"])
        raise AssertionError("expected ValueError (no SDC and no clock)")
    except ValueError:
        pass


def test_requires_netlist_and_libs():
    for bad in (dict(design="t", netlist="", libs=["a.lib"], sdc="t.sdc"),
                dict(design="t", netlist="t.v", libs=[], sdc="t.sdc")):
        try:
            sta_scenario(**bad)
            raise AssertionError("expected ValueError")
        except ValueError:
            pass


def test_mcmm_master():
    t = mcmm_master(design="top", scenario_files=["ss.sta", "tt.sta", "ff.sta"])
    assert "design: top" in t
    assert "scenarios: ss.sta, tt.sta, ff.sta" in t


def test_metrics_mapping():
    data = {"endpoints": 3, "wns_ns": -0.05, "tns_ns": -0.2,
            "hold_endpoints": 3, "whs_ns": 0.1, "ths_ns": 0.0}
    m = metrics_from_json("ss_100C_1v60", data)
    assert m["vyges__timing__setup__ws__corner:ss_100C_1v60"] == -0.05
    assert m["vyges__timing__setup__tns__corner:ss_100C_1v60"] == -0.2
    assert m["vyges__timing__hold__ws__corner:ss_100C_1v60"] == 0.1


def test_metrics_skips_missing():
    # no setup endpoints + null wns → no setup metrics emitted
    m = metrics_from_json("ff", {"endpoints": 0, "wns_ns": None, "hold_endpoints": 0})
    assert m == {}


def test_worst_across_corners():
    metrics = {
        "vyges__timing__setup__ws__corner:ss": -0.05,
        "vyges__timing__setup__ws__corner:ff": 0.20,
        "vyges__timing__hold__ws__corner:ss": 0.10,
        "vyges__timing__hold__ws__corner:ff": -0.02,
    }
    w = worst_across_corners(metrics)
    assert w["vyges__timing__setup__ws"] == -0.05  # worst (min) setup
    assert w["vyges__timing__hold__ws"] == -0.02   # worst (min) hold


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in tests:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(tests)} passed")
