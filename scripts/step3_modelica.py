"""Step 3: the loop as a Modelica model, simulated by omc, against python-control.
Also draws the three-way plot when ngspice is available.

    python scripts/step3_modelica.py    -> results/step3_modelica.txt, results/three_way_step.png
"""
import _setup  # noqa: F401
import time

import matplotlib.pyplot as plt
import numpy as np

from mct import compare, design, modelica, report, spice
from mct.params import STEP, T_END


def main():
    t, y = design.step(design.closed_loop(), STEP, design.grid(T_END))
    t0 = time.perf_counter()
    modelica.ensure()
    dt = time.perf_counter() - t0
    tm, wm, vm = modelica.waveform("linear")
    a = compare.agreement(t, y, compare.on_grid(tm, wm, t), STEP)
    m = design.metrics(t, compare.on_grid(tm, wm, t), STEP)
    e = STEP - y
    v_pc = 20 * e + 40 * np.concatenate([[0], np.cumsum((e[1:] + e[:-1]) / 2 * np.diff(t))])
    av = compare.agreement(t, v_pc, compare.on_grid(tm, vm, t), 10.0)
    out = report.header("Step 3: physical model in OpenModelica (omc, Modelica Standard Library "
                        f"{modelica.msl_version()})")
    out += [
        "Model: MotorControl.SpeedLoop (Resistor, Inductor, RotationalEMF, Inertia, Damper, "
        "SpeedSensor; Step, Feedback, PI)",
        "Solver: DASSL (omc default), tolerance 1e-8, 3000 output intervals (1 ms)",
        f"Rows in the linear CSV: {len(tm)}",
        "",
        f"Speed against python-control: max error {a['max_err_pct']:.5f} % of the setpoint at {a['at_s']:.3f} s, "
        f"rms {a['rms_err_pct']:.5f} %",
        f"Armature voltage against python-control's kp*e + ki*integral(e): max error {av['max_err_pct']:.5f} % "
        "of 10 V",
        f"Overshoot {m['overshoot_pct']:.3f} %, settling {m['settling_s']:.3f} s, rise {m['rise_s']:.3f} s",
        "",
        "Solver tolerance against agreement (linear loop):",
        f"{'tolerance':>10} {'max error':>12} {'rms error':>12}",
    ]
    for tol, case in (("1e-4", "linear_tol1e-4"), ("1e-6", "linear_tol1e-6"), ("1e-8", "linear")):
        tt, wt, _ = modelica.waveform(case)
        at = compare.agreement(t, y, compare.on_grid(tt, wt, t), STEP)
        out.append(f"{tol:>10} {at['max_err_pct']:>10.5f} % {at['rms_err_pct']:>10.5f} %")
    out += ["", "omc time per run, from its SimulationResult records (compile = generated C to an executable):",
            f"{'run':<28} {'compile':>8} {'simulate':>9} {'total':>7}"]
    for run, tm_ in modelica.timings().items():
        out.append(f"{run:<28} {tm_.get('timeCompile', 0):>6.2f} s {tm_.get('timeSimulation', 0):>7.3f} s "
                   f"{tm_.get('timeTotal', 0):>5.2f} s")
    if dt > 1:
        out.append(f"(omc wall time for the whole script, including Docker start-up in CI: {dt:.1f} s)")
    report.write("step3_modelica.txt", out)
    if spice.have_ngspice():
        ts, ws, _ = spice.run_pyspice("linear")
        plot(t, y, compare.on_grid(ts, ws, t), compare.on_grid(tm, wm, t))


def plot(t, y, ws, wm):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 5.6), sharex=True, dpi=110,
                                 gridspec_kw={"height_ratios": [2, 1]})
    c = report.COLORS
    a1.plot(t, y, color=c["python-control"], lw=2.5, label="python-control (transfer function)")
    a1.plot(t, ws, color=c["ngspice"], lw=2, ls=(0, (6, 3)), label="ngspice (op-amp circuit, PySpice)")
    a1.plot(t, wm, color=c["OpenModelica"], lw=2, ls=(0, (1, 2)), label="OpenModelica (MSL physical model)")
    a1.set_ylabel("speed (rad/s)", color=report.INK)
    a1.set_title("1 rad/s setpoint step, PI kp=20, ki=40", color=report.INK, loc="left")
    a1.legend(frameon=False, loc="lower right")
    a2.plot(t, (ws - y) / STEP * 100, color=c["ngspice"], lw=2, label="ngspice")
    a2.plot(t, (wm - y) / STEP * 100, color=c["OpenModelica"], lw=2, label="OpenModelica")
    a2.axhline(0, color=report.MUTED, lw=1)
    a2.set_ylabel("difference from\npython-control (%)", color=report.INK)
    a2.set_xlabel("time (s)", color=report.INK)
    a2.legend(frameon=False, loc="upper right")
    for ax in (a1, a2):
        report.style(ax)
    fig.tight_layout()
    fig.savefig(report.RESULTS / "three_way_step.png")
    print("-> results/three_way_step.png")


if __name__ == "__main__":
    main()
