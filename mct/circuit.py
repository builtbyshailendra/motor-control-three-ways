"""Step 2, PySpice: the same loop as a circuit for ngspice.

Mechanical side as an electrical analog (force-current analogy):
    speed w        -> voltage of node `w` (1 V = 1 rad/s)
    torque K*i     -> current K*i injected into node `w` (CCCS, F element)
    inertia J      -> capacitor J farads from `w` to ground   (C dV/dt = I)
    friction b     -> resistor 1/b ohms from `w` to ground     (I = V/R = b*w)
Armature: R and L in series with a back-EMF source E = K*w (VCVS from node `w`).
A zero-volt source `Vsense` in the armature measures i for the torque source.

Controller, op-amps as ideal high-gain VCVS (gain 1e6), all 10 kohm unless noted:
    tach  unity buffer, so the controller does not load the speed node
    U1    difference amplifier            e  = r - w
    U2    inverting integrator, Ri Ci     xn = -(1/(Ri Ci)) * integral(e)       Ri Ci = 1/ki
    U3    inverting amplifier             pn = -(Rp2/Rp1) * e                   Rp2/Rp1 = kp
    U4    inverting summer                v  = -(pn + xn) = kp*e + ki*integral(e)
    power stage: u = v (linear) or u = clip(v, +/-12 V) (B source)
Anti-windup: an ideal difference amplifier makes d = u - v and a resistor Rt feeds it
into U2's summing node, so dI/dt = ki*e + (u - v)/(Rt Ci): back-calculation with
Tt = Rt*Ci, the same law as Modelica's LimPID and mct.nonlinear.
"""
from __future__ import annotations

from PySpice.Spice.Netlist import Circuit

from .params import CONTROLLER, MOTOR, STEP, T_END, T_WINDUP, TT, V_SUPPLY, Motor, PI

MODES = ("linear", "saturated", "antiwindup")
R0 = 10e3  # ohm, the base resistor of every op-amp stage
OPAMP_GAIN = 1e6
RISE = 1e-6  # s, the setpoint source's rise time (an ideal step can't be a SPICE source)


def component_values(c: PI = CONTROLLER, tt: float = TT) -> dict:
    ci = 1 / (c.ki * R0)
    return {"Ri": R0, "Ci": ci, "Rp1": R0, "Rp2": c.kp * R0, "Rt": tt / ci}


def build(mode: str = "linear", step: float = STEP, m: Motor = MOTOR, c: PI = CONTROLLER,
          vmax: float = V_SUPPLY, tt: float = TT, opamp_gain: float = OPAMP_GAIN) -> Circuit:
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    v = component_values(c, tt)
    gnd = 0
    cir = Circuit(f"DC motor speed loop, {mode}")

    def opamp(name, plus, minus, out):
        cir.VCVS(name, out, gnd, plus, minus, opamp_gain)

    # setpoint: 0 -> step volts (= rad/s) in RISE seconds at t = 0
    cir.PieceWiseLinearVoltageSource("ref", "r", gnd, values=[(0, 0), (RISE, step)])
    cir.VCVS("tach", "ws", gnd, "w", gnd, 1)

    # U1: e = r - ws
    cir.R("e1", "r", "u1p", R0)
    cir.R("e2", "u1p", gnd, R0)
    cir.R("e3", "ws", "u1n", R0)
    cir.R("e4", "u1n", "e", R0)
    opamp("u1", "u1p", "u1n", "e")

    # U2: integrator
    cir.R("i", "e", "u2n", v["Ri"])
    cir.C("i", "u2n", "xn", v["Ci"])
    opamp("u2", gnd, "u2n", "xn")

    # U3: proportional
    cir.R("p1", "e", "u3n", v["Rp1"])
    cir.R("p2", "u3n", "pn", v["Rp2"])
    opamp("u3", gnd, "u3n", "pn")

    # U4: summer, v = kp*e + I
    cir.R("s1", "pn", "u4n", R0)
    cir.R("s2", "xn", "u4n", R0)
    cir.R("s3", "u4n", "v", R0)
    opamp("u4", gnd, "u4n", "v")

    # power stage
    if mode == "linear":
        cir.VCVS("pwr", "va", gnd, "v", gnd, 1)
    else:
        cir.BehavioralSource("pwr", "va", gnd,
                             voltage_expression=f"min(max(V(v),{-vmax:g}),{vmax:g})")
    if mode == "antiwindup":
        cir.VCVS("trk", "d", gnd, "va", "v", 1)  # d = u - v
        cir.R("t", "d", "u2n", v["Rt"])

    # motor, electrical side
    cir.R("a", "va", "a1", m.R)
    cir.L("a", "a1", "a2", m.L)
    cir.V("sense", "a2", "a3", 0)
    cir.VCVS("emf", "a3", gnd, "w", gnd, m.K)
    # motor, mechanical side as its electrical analog
    cir.CCCS("torque", gnd, "w", "Vsense", m.K)
    cir.C("J", "w", gnd, m.J)
    cir.R("b", "w", gnd, 1 / m.b)
    return cir


def netlist(mode: str = "linear", **kw) -> str:
    """The element lines PySpice generates (title first, no .end), with LF line ends
    on every OS so the committed netlists are the same on Windows and Linux."""
    text = str(build(mode, **kw)).replace("\r\n", "\n").rstrip()
    if text.endswith(".end"):
        text = text[: -len(".end")].rstrip()
    return text + "\n"


TRAN = {"step": 1e-3, "max_step": 1e-3}
STOP = {"linear": T_END, "saturated": T_WINDUP, "antiwindup": T_WINDUP}
OPTIONS = {"reltol": 1e-4}


def batch_deck(mode: str = "linear", out: str = "out.txt", stop: float | None = None, **kw) -> str:
    """A deck for `ngspice -b`: the PySpice netlist plus .tran and a .control block
    that writes time, speed and supply voltage to a text file."""
    body = netlist(mode, **kw).rstrip()
    stop = STOP[mode] if stop is None else stop
    opts = " ".join(f"{k}={v:g}" for k, v in OPTIONS.items())
    return "\n".join([
        body,
        f".options {opts}",
        f".tran {TRAN['step']:g} {stop:g} 0 {TRAN['max_step']:g}",
        ".control",
        "run",
        "set wr_singlescale",
        "set wr_vecnames",
        "option numdgt=12",
        f"wrdata {out} v(w) v(va)",
        "quit",
        ".endc",
        ".end",
        "",
    ])
