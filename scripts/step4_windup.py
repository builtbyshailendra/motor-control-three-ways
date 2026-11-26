"""Step 4: a +/-12 V supply limit, integrator windup and back-calculation anti-windup.
python-control's nonlinear I/O model always runs; ngspice and OpenModelica when installed.

    python scripts/step4_windup.py      -> results/step4_windup.txt, results/windup.png
"""
import _setup  # noqa: F401
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

from mct import compare, design, modelica, nonlinear, report, spice
from mct.params import NI, STEP, T_WINDUP, TT, V_SUPPLY

CASE_LABEL = {
    "linear": "no supply limit (linear)",
    "saturated": f"+/-{V_SUPPLY:g} V, no anti-windup",
    "antiwindup": f"+/-{V_SUPPLY:g} V, back-calculation (Tt = {TT:g} s)",
    "limpid_no_tracking": "+/-12 V, LimPID with Ni = 1e9 (tracking off)",
}
CASE_COLOR = {"linear": "#8a8984", "saturated": "#eb6834", "antiwindup": "#6250d6"}
TOOL_DASH = {"python-control": "-", "ngspice": (0, (6, 3)), "OpenModelica": (0, (1, 2))}


def main():
    t = design.grid(T_WINDUP)
    runs = {}  # (tool, case) -> (speed on grid, voltage on grid)
    for case, kw in nonlinear.CASES.items():
        _, w, v = nonlinear.simulate(**kw, t=t)
        runs["python-control", case] = (w, v)
    if spice.have_ngspice():
        for case in ("saturated", "antiwindup"):
            ts, ws, vs = spice.run_pyspice(case)
            runs["ngspice", case] = (compare.on_grid(ts, ws, t), compare.on_grid(ts, vs, t))
    if modelica.have_omc():
        modelica.ensure()
        for case in ("saturated", "antiwindup", "limpid_no_tracking"):
            tm, wm, vm = modelica.waveform(case)
            runs["OpenModelica", case] = (compare.on_grid(tm, wm, t), compare.on_grid(tm, vm, t))

    out = report.header(f"Step 4: supply limit, windup and anti-windup ({STEP:g} rad/s step, "
                        f"{T_WINDUP:g} s simulated)")
    out += [
        f"The step needs 10.01 V at steady state; the PI asks for 20 V at t = 0 and up to "
        f"{runs['python-control', 'linear'][1].max():.2f} V without a limit.",
        f"Anti-windup: dI/dt = ki*e + (u - v)/Tt, Tt = Ni*Ti = {NI:g} * 0.5 s = {TT:g} s "
        "(LimPID's default Ni)",
        "",
        f"{'tool':<15} {'case':<46} {'overshoot':>9} {'settling':>9} {'peak at':>8} {'max |u|':>8} "
        f"{'vs python-control':>18}",
    ]
    for (tool, case), (w, v) in runs.items():
        m = design.metrics(t, w, STEP)
        ref_case = "saturated" if case == "limpid_no_tracking" else case
        ref = runs["python-control", ref_case][0]
        agree = "" if tool == "python-control" else \
            f"{compare.agreement(t, ref, w, STEP)['max_err_pct']:.1e} % max"
        out.append(f"{tool:<15} {CASE_LABEL[case]:<46} {m['overshoot_pct']:>7.2f} % {m['settling_s']:>7.3f} s "
                   f"{m['peak_s']:>6.3f} s {np.max(np.abs(v)):>6.2f} V {agree:>18}")
    out += ["", "(settling: last time outside the 2 % band; overshoot 0 and a peak at the end mean the "
            "speed rose monotonically; vs python-control: max |difference| from python-control's "
            "nonlinear run of the same case, % of the setpoint)", "",
            "Choosing Tt (python-control nonlinear model, same step):",
            f"{'Tt':>8} {'overshoot':>10} {'settling':>9}"]
    for tt in (0.05, 0.1, 0.25, 0.45, 1.0, 2.0, None):
        _, w, _ = nonlinear.simulate(vmax=V_SUPPLY, tt=tt, t=t)
        m = design.metrics(t, w, STEP)
        label = "none" if tt is None else f"{tt:g} s"
        out.append(f"{label:>8} {m['overshoot_pct']:>8.2f} % {m['settling_s']:>7.3f} s")
    report.write("step4_windup.txt", out)
    plot(t, runs)


def plot(t, runs):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 6), sharex=True, dpi=110)
    for (tool, case), (w, v) in runs.items():
        if case not in CASE_COLOR:
            continue
        kw = dict(color=CASE_COLOR[case], ls=TOOL_DASH[tool], lw=2.5 if tool == "python-control" else 2)
        a1.plot(t, w, **kw)
        a2.plot(t, v, **kw)
    a1.axhline(STEP, color=report.MUTED, lw=1)
    a2.axhline(V_SUPPLY, color=report.MUTED, lw=1)
    a1.set_ylabel("speed (rad/s)", color=report.INK)
    a2.set_ylabel("armature voltage (V)", color=report.INK)
    a2.set_xlabel("time (s)", color=report.INK)
    a1.set_title(f"{STEP:g} rad/s step with a {V_SUPPLY:g} V supply: windup and the fix",
                 color=report.INK, loc="left")
    cases = [Line2D([], [], color=CASE_COLOR[c], lw=2.5, label=CASE_LABEL[c]) for c in CASE_COLOR]
    tools = [Line2D([], [], color=report.MUTED, ls=d, lw=2, label=tool) for tool, d in TOOL_DASH.items()]
    a1.legend(handles=cases, frameon=False, loc="lower right", fontsize=8.5)
    a2.legend(handles=tools, frameon=False, loc="upper right", fontsize=8.5)
    a2.set_ylim(-2, 23)
    for ax in (a1, a2):
        report.style(ax)
    fig.tight_layout()
    fig.savefig(report.RESULTS / "windup.png")
    print("-> results/windup.png")


if __name__ == "__main__":
    main()
