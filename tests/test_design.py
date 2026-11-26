"""Step 1: the plant, the PI design and its discretisation, in python-control."""
import numpy as np
import pytest
from scipy.optimize import brentq

from mct import design as d
from mct.params import CONTROLLER, MOTOR, SPECS, STEP


def test_plant_matches_the_hand_derivation():
    m = MOTOR
    p = d.plant()
    # DC gain K / (b R + K^2) and the roots of (J s + b)(L s + R) + K^2
    assert d.open_loop_summary()["dc_gain_rad_s_per_V"] == pytest.approx(m.K / (m.b * m.R + m.K**2))
    roots = np.sort(np.roots([m.J * m.L, m.J * m.R + m.b * m.L, m.b * m.R + m.K**2]).real)
    assert np.allclose(np.sort(np.real(d.ct.poles(p))), roots)
    assert roots == pytest.approx([-9.9975, -2.0025], abs=1e-4)


def test_closed_loop_meets_the_ctms_specs():
    t, y = d.step(d.closed_loop())
    m = d.metrics(t, y, STEP)
    assert m["overshoot_pct"] < SPECS["overshoot_pct"]
    assert m["settling_s"] < SPECS["settling_s"]
    assert m["steady_state_error_pct"] < SPECS["steady_state_error_pct"]
    assert m["overshoot_pct"] == pytest.approx(1.71, abs=0.01)
    assert m["settling_s"] == pytest.approx(0.583, abs=0.002)


def test_pi_zero_sits_on_the_slow_motor_pole():
    zero = -CONTROLLER.ki / CONTROLLER.kp
    assert zero == pytest.approx(-2.0)
    assert d.open_loop_summary()["poles"][1] == pytest.approx(zero, abs=0.01)


def test_margins_and_an_independent_phase_margin():
    mg = d.margins(d.loop())
    assert not np.isfinite(mg["gain_margin"])  # the phase never reaches -180 degrees
    assert mg["phase_margin_deg"] == pytest.approx(69.5, abs=0.1)

    # the same phase margin from the formula, without python-control
    m, c = MOTOR, CONTROLLER

    def L(w):
        s = 1j * w
        return (c.kp + c.ki / s) * m.K / ((m.J * s + m.b) * (m.L * s + m.R) + m.K**2)

    wc = brentq(lambda w: abs(L(w)) - 1, 0.1, 100)
    pm = 180 + np.degrees(np.angle(L(wc)))
    assert wc == pytest.approx(mg["gain_crossover_rad_s"], rel=1e-3)
    assert pm == pytest.approx(mg["phase_margin_deg"], abs=0.05)


def test_tustin_pi_at_1khz():
    cd, _, _ = d.discrete_loop(1e-3)
    num = np.squeeze(cd.num[0][0])
    den = np.squeeze(cd.den[0][0])
    # Tustin: s -> (2/T)(z-1)/(z+1) turns kp + ki/s into
    # ((kp + ki T/2) z + (ki T/2 - kp)) / (z - 1)
    assert num == pytest.approx([20.02, -19.98])
    assert den == pytest.approx([1.0, -1.0])


def test_discrete_1khz_tracks_continuous():
    r = d.discrete_vs_continuous(1e-3)
    assert r["max_diff_pct"] < 0.2
    assert abs(r["d_overshoot_pct"] - 1.71) < 0.1
    assert 50 < r["d_gain_margin_db"] < 60  # finite now: sampling adds delay


def test_slower_sampling_erodes_the_margins():
    rows = [d.discrete_vs_continuous(ts) for ts in (1e-3, 1e-2, 5e-2, 1e-1)]
    gms = [r["d_gain_margin_db"] for r in rows]
    pms = [r["d_phase_margin_deg"] for r in rows]
    assert gms == sorted(gms, reverse=True)
    assert pms == sorted(pms, reverse=True)
    assert rows[-1]["d_overshoot_pct"] > SPECS["overshoot_pct"]  # 10 Hz breaks the spec


def test_discrete_gain_margin_against_an_independent_sweep():
    for ts in (1e-3, 1e-2, 1e-1):
        _, _, ld = d.discrete_loop(ts)
        w = np.logspace(-1, np.log10(np.pi / ts), 200_001)
        z = np.exp(1j * w * ts)
        L = np.polyval(np.squeeze(ld.num[0][0]), z) / np.polyval(np.squeeze(ld.den[0][0]), z)
        phase = np.unwrap(np.angle(L))
        k = np.where(np.diff(np.sign(phase + np.pi)))[0][0]  # first -180 degree crossing
        mg = d.margins(ld)
        assert w[k] == pytest.approx(mg["phase_crossover_rad_s"], rel=1e-3)
        assert 1 / abs(L[k]) == pytest.approx(mg["gain_margin"], rel=1e-3)
