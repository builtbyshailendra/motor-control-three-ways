"""Step 4, python-control's nonlinear I/O systems: the same loop with a supply limit.

The linear transfer functions of step 1 cannot represent saturation: u = kp*e + I
can be any number of volts. Here the PI is an `nlsys` whose output is clipped to
+/- vmax, interconnected with the linear motor by signal name.

Anti-windup is back-calculation, the scheme Modelica's LimPID uses:
    dI/dt = ki*e + (u - v)/Tt,   v = kp*e + I,   u = clip(v)
While the output is not saturated u = v and the extra term is zero.
"""
from __future__ import annotations

import numpy as np
import control as ct

from .design import grid, plant_ss
from .params import CONTROLLER, MOTOR, STEP, T_WINDUP, TT, V_SUPPLY, Motor, PI


def pi_saturated(c: PI = CONTROLLER, vmax: float | None = V_SUPPLY,
                 tt: float | None = None) -> ct.NonlinearIOSystem:
    """PI with an output limit; tt=None means no anti-windup."""

    def clip(v):
        return v if vmax is None else min(max(v, -vmax), vmax)

    def updfcn(t, x, u, params):
        r, w = u
        e = r - w
        v = c.kp * e + x[0]
        dx = c.ki * e
        if tt is not None:
            dx += (clip(v) - v) / tt
        return [dx]

    def outfcn(t, x, u, params):
        r, w = u
        return [clip(c.kp * (r - w) + x[0])]

    return ct.nlsys(updfcn, outfcn, inputs=["r", "w"], outputs=["v"], states=["I"],
                    name="pi")


def loop(c: PI = CONTROLLER, m: Motor = MOTOR, vmax: float | None = V_SUPPLY,
         tt: float | None = None) -> ct.InterconnectedSystem:
    """Motor and controller connected by signal names: pi.v -> motor.v, motor.w -> pi.w."""
    return ct.interconnect([plant_ss(m), pi_saturated(c, vmax, tt)],
                           inputs="r", outputs=["w", "v"], name="speed_loop")


def simulate(vmax: float | None = V_SUPPLY, tt: float | None = None, step: float = STEP,
             t: np.ndarray | None = None, c: PI = CONTROLLER, m: Motor = MOTOR):
    """Setpoint step at t = 0 from rest: (t, speed rad/s, applied voltage V)."""
    t = grid(T_WINDUP) if t is None else t
    sys = loop(c, m, vmax, tt)
    # The clip makes the right-hand side non-smooth: keep steps short and tolerances tight.
    resp = ct.input_output_response(
        sys, t, step * np.ones_like(t), X0=0,
        solve_ivp_method="RK45",
        solve_ivp_kwargs={"max_step": 1e-3, "rtol": 1e-8, "atol": 1e-10})
    return np.asarray(resp.time), np.asarray(resp.outputs[0]), np.asarray(resp.outputs[1])


CASES = {
    "linear": dict(vmax=None, tt=None),
    "saturated": dict(vmax=V_SUPPLY, tt=None),
    "antiwindup": dict(vmax=V_SUPPLY, tt=TT),
}
