"""Step 4: a +/-12 V supply limit, integrator windup, and back-calculation anti-windup,
seen by python-control's nonlinear I/O systems, ngspice and OpenModelica."""
import numpy as np
import pytest

from _tools import needs_ngspice, needs_omc
from mct import compare, design, spice
from mct.params import STEP, T_WINDUP, V_SUPPLY

T = design.grid(T_WINDUP)


def os_pct(t_src, w_src):
    return design.metrics(T, compare.on_grid(t_src, w_src, T), STEP)["overshoot_pct"]


def test_nonlinear_model_without_a_limit_is_the_linear_model(pc_nonlinear):
    t, w, _ = pc_nonlinear["linear"]
    _, y = design.step(design.closed_loop(), STEP, t)
    assert np.max(np.abs(w - y)) < 1e-6


def test_the_linear_loop_asks_for_more_than_the_supply(pc_nonlinear):
    _, _, v = pc_nonlinear["linear"]
    assert v.max() == pytest.approx(21.2, abs=0.1)  # 20 V at t = 0, more while the integrator fills
    assert v.max() > V_SUPPLY


def test_windup_overshoot_and_the_fix(pc_nonlinear):
    lin = design.metrics(*pc_nonlinear["linear"][:2], STEP)
    sat = design.metrics(*pc_nonlinear["saturated"][:2], STEP)
    aw = design.metrics(*pc_nonlinear["antiwindup"][:2], STEP)
    assert lin["overshoot_pct"] == pytest.approx(1.71, abs=0.01)
    assert sat["overshoot_pct"] == pytest.approx(14.86, abs=0.05)  # windup: 8.7x the linear overshoot
    assert aw["overshoot_pct"] < 0.1
    assert sat["settling_s"] == pytest.approx(2.91, abs=0.02)
    assert aw["settling_s"] == pytest.approx(1.05, abs=0.02)
    for case in ("saturated", "antiwindup"):
        assert np.max(np.abs(pc_nonlinear[case][2])) <= V_SUPPLY + 1e-9


@needs_ngspice
@pytest.mark.parametrize("mode", ["saturated", "antiwindup"])
def test_ngspice_windup_matches_python_control(mode, pc_nonlinear):
    t, w, va = spice.run_pyspice(mode)
    tp, wp, _ = pc_nonlinear[mode]
    a = compare.agreement(T, wp, compare.on_grid(t, w, T), STEP)
    assert a["max_err_pct"] < 1.0
    assert os_pct(t, w) == pytest.approx(os_pct(tp, wp), abs=0.2)
    assert np.max(np.abs(va)) <= V_SUPPLY + 1e-6


@needs_omc
@pytest.mark.parametrize("case", ["saturated", "antiwindup"])
def test_modelica_windup_matches_python_control(case, om_results, pc_nonlinear):
    t, w, v = om_results[case]
    tp, wp, _ = pc_nonlinear[case]
    a = compare.agreement(T, wp, compare.on_grid(t, w, T), STEP)
    assert a["max_err_pct"] < 1.0
    assert os_pct(t, w) == pytest.approx(os_pct(tp, wp), abs=0.2)
    assert np.max(np.abs(v)) <= V_SUPPLY + 1e-6


@needs_omc
def test_limpid_without_tracking_winds_up_like_pi_plus_limiter(om_results):
    t1, w1, _ = om_results["limpid_no_tracking"]
    t2, w2, _ = om_results["saturated"]
    a = compare.agreement(T, compare.on_grid(t2, w2, T), compare.on_grid(t1, w1, T), STEP)
    assert a["max_err_pct"] < 0.1
