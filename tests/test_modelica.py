"""Step 3: the loop as a physical model (Modelica Standard Library), simulated by omc."""
import numpy as np
import pytest

from _tools import needs_omc
from mct import compare, design
from mct.params import STEP


@needs_omc
def test_modelica_linear_loop_matches_python_control(om_results, pc_linear):
    t_om, w_om, v_om = om_results["linear"]
    t, y = pc_linear
    assert t_om[-1] == pytest.approx(t[-1])
    a = compare.agreement(t, y, compare.on_grid(t_om, w_om, t), STEP)
    assert a["max_err_pct"] < 1.0
    assert a["max_err_pct"] < 0.05  # measured: see results/step3_modelica.txt


@needs_omc
def test_modelica_controller_output_matches_python_control(om_results, pc_linear):
    """The PI block's output (armature voltage) agrees too, not just the speed."""
    t_om, w_om, v_om = om_results["linear"]
    t, y = pc_linear
    # v = kp*e + ki*integral(e), from python-control's speed waveform
    e = STEP - y
    v_pc = 20 * e + 40 * np.concatenate([[0], np.cumsum((e[1:] + e[:-1]) / 2 * np.diff(t))])
    a = compare.agreement(t, v_pc, compare.on_grid(t_om, v_om, t), 10.0)  # % of the 10 V final
    assert a["max_err_pct"] < 1.0


@needs_omc
def test_modelica_meets_the_same_specs(om_results):
    t_om, w_om, _ = om_results["linear"]
    t = design.grid()
    m = design.metrics(t, compare.on_grid(t_om, w_om, t), STEP)
    assert m["overshoot_pct"] == pytest.approx(1.71, abs=0.05)
    assert m["settling_s"] == pytest.approx(0.583, abs=0.005)


@needs_omc
def test_looser_solver_tolerance_costs_accuracy_but_stays_inside_1_percent(om_results, pc_linear):
    t, y = pc_linear
    errs = {}
    for case in ("linear_tol1e-4", "linear_tol1e-6", "linear"):
        tm, wm, _ = om_results[case]
        errs[case] = compare.agreement(t, y, compare.on_grid(tm, wm, t), STEP)["max_err_pct"]
    assert errs["linear"] < errs["linear_tol1e-4"]
    assert max(errs.values()) < 1.0
