"""Step 1, python-control: the plant, the PI design, margins, step metrics, discretisation."""
from __future__ import annotations

import warnings

import numpy as np
import control as ct

from .params import CONTROLLER, DT, MOTOR, STEP, T_END, Motor, PI

S = ct.tf("s")


def plant(m: Motor = MOTOR) -> ct.TransferFunction:
    """Speed / armature voltage, rad/s per V.

    J dw/dt + b w = K i   and   L di/dt + R i = v - K w   give
    W(s)/V(s) = K / ((J s + b)(L s + R) + K^2).
    """
    return m.K / ((m.J * S + m.b) * (m.L * S + m.R) + m.K**2)


def plant_ss(m: Motor = MOTOR) -> ct.StateSpace:
    """The same plant as named states (current i, speed w), for the nonlinear model."""
    a = [[-m.R / m.L, -m.K / m.L], [m.K / m.J, -m.b / m.J]]
    return ct.ss(a, [[1 / m.L], [0]], [[0, 1]], [[0]],
                 inputs="v", outputs="w", states=["i", "w"], name="motor")


def pi_tf(c: PI = CONTROLLER) -> ct.TransferFunction:
    return c.kp + c.ki / S


def loop(m: Motor = MOTOR, c: PI = CONTROLLER) -> ct.TransferFunction:
    return pi_tf(c) * plant(m)


def closed_loop(m: Motor = MOTOR, c: PI = CONTROLLER) -> ct.TransferFunction:
    """Setpoint to speed, unity feedback."""
    return ct.feedback(loop(m, c), 1)


def grid(t_end: float = T_END, dt: float = DT) -> np.ndarray:
    return np.round(np.arange(0.0, t_end + dt / 2, dt), 12)


def step(sys, step: float = STEP, t: np.ndarray | None = None):
    """Step response of an LTI system on the comparison grid: (t, y)."""
    t = grid() if t is None else t
    r = ct.step_response(sys, T=t)
    return r.time, step * np.squeeze(r.outputs)


def metrics(t: np.ndarray, y: np.ndarray, final: float) -> dict:
    """Step metrics from a waveform, the same way for every tool (python-control's step_info)."""
    info = ct.step_info(y, T=t, yfinal=final, SettlingTimeThreshold=0.02)
    return {
        "overshoot_pct": float(info["Overshoot"]),
        "settling_s": float(info["SettlingTime"]),
        "rise_s": float(info["RiseTime"]),
        "peak": float(info["Peak"]),
        "peak_s": float(info["PeakTime"]),
        "final": float(y[-1]),
        "steady_state_error_pct": abs(final - float(y[-1])) / final * 100,
    }


def margins(L) -> dict:
    """Gain margin (as a ratio and in dB), phase margin, and the two crossover frequencies."""
    with warnings.catch_warnings():
        # For the discrete loops python-control warns that it falls back from the
        # polynomial method to a frequency sweep; tests/test_design.py checks the
        # result against an independent sweep.
        warnings.simplefilter("ignore", UserWarning)
        gm, pm, wcg, wcp = ct.margin(L)
    gm = float(gm)
    return {
        "gain_margin": gm,
        "gain_margin_db": 20 * np.log10(gm) if np.isfinite(gm) else float("inf"),
        "phase_margin_deg": float(pm),
        "phase_crossover_rad_s": float(wcg),
        "gain_crossover_rad_s": float(wcp),
    }


def open_loop_summary(m: Motor = MOTOR) -> dict:
    p = plant(m)
    poles = np.sort(np.real(ct.poles(p)))
    t = grid(4.0)
    tt, y = step(p, 1.0, t)
    info = ct.step_info(y, T=tt, SettlingTimeThreshold=0.02)
    return {
        "poles": [float(x) for x in poles],
        "time_constants_s": [float(-1 / x) for x in poles],
        "dc_gain_rad_s_per_V": float(ct.dcgain(p)),
        "volts_for_1_rad_s": float(1 / ct.dcgain(p)),
        "rise_s": float(info["RiseTime"]),
        "settling_s": float(info["SettlingTime"]),
    }


def discrete_loop(ts: float, m: Motor = MOTOR, c: PI = CONTROLLER):
    """PI by Tustin, plant by zero-order hold (what a sampled controller with a DAC sees)."""
    cd = ct.c2d(pi_tf(c), ts, method="tustin")
    pd = ct.c2d(plant(m), ts, method="zoh")
    return cd, pd, cd * pd


def discrete_vs_continuous(ts: float, m: Motor = MOTOR, c: PI = CONTROLLER) -> dict:
    """Discrete closed-loop step against the continuous one, at the sample instants."""
    cd, pd, ld = discrete_loop(ts, m, c)
    cld = ct.feedback(ld, 1)
    n = int(round(T_END / ts))
    t = np.arange(n + 1) * ts
    _, yd = step(cld, STEP, t)
    _, yc = step(closed_loop(m, c), STEP, t)
    out = {"ts": ts, "max_diff_pct": float(np.max(np.abs(yd - yc)) / STEP * 100)}
    out.update({f"d_{k}": v for k, v in metrics(t, yd, STEP).items()})
    out.update({f"d_{k}": v for k, v in margins(ld).items()})
    out["pi_num"] = [float(x) for x in np.squeeze(cd.num[0][0])]
    out["pi_den"] = [float(x) for x in np.squeeze(cd.den[0][0])]
    return out
