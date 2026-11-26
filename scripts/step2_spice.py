"""Step 2: the loop as an op-amp circuit, PySpice -> ngspice, against python-control.

    python scripts/step2_spice.py       -> results/step2_spice.txt  (needs ngspice)
"""
import _setup  # noqa: F401
import time

import PySpice

from mct import circuit, compare, design, report, spice
from mct.params import STEP, T_END


def main():
    t, y = design.step(design.closed_loop(), STEP, design.grid(T_END))
    out = report.header(f"Step 2: op-amp circuit in ngspice via PySpice {PySpice.__version__}")
    v = circuit.component_values()
    out += [
        "Controller parts: Ri = 10 kohm, Ci = {:.2f} uF (Ri*Ci = {:.0f} ms = 1/ki), "
        "Rp2/Rp1 = {:.0f}k/10k (kp = {:.0f})".format(v["Ci"] * 1e6, v["Ri"] * v["Ci"] * 1e3,
                                                   v["Rp2"] / 1e3, v["Rp2"] / v["Rp1"]),
        f"Op-amps: ideal VCVS, gain {circuit.OPAMP_GAIN:g}; setpoint rises in {circuit.RISE*1e6:g} us",
        f"Transient: step {circuit.TRAN['step']:g} s, max step {circuit.TRAN['max_step']:g} s, "
        f"options {circuit.OPTIONS}",
        f"Netlist: {len(circuit.netlist('linear').splitlines()) - 1} elements (netlists/linear.cir)",
        "",
    ]
    runs = {}
    for name, fn in (("PySpice shared library", spice.run_pyspice),
                     ("ngspice -b on the same deck", spice.run_batch)):
        t0 = time.perf_counter()
        ts, ws, vs = fn("linear")
        dt = time.perf_counter() - t0
        runs[name] = (ts, ws)
        a = compare.agreement(t, y, compare.on_grid(ts, ws, t), STEP)
        m = design.metrics(t, compare.on_grid(ts, ws, t), STEP)
        out += [
            f"{name}: {len(ts)} time points, {dt:.2f} s wall time",
            f"  against python-control: max error {a['max_err_pct']:.4f} % of the setpoint at {a['at_s']:.3f} s, "
            f"rms {a['rms_err_pct']:.4f} %, at 3 s {a['final_err_pct']:.5f} %",
            f"  overshoot {m['overshoot_pct']:.3f} %, settling {m['settling_s']:.3f} s, rise {m['rise_s']:.3f} s "
            f"(python-control: {design.metrics(t, y, STEP)['overshoot_pct']:.3f} %, "
            f"{design.metrics(t, y, STEP)['settling_s']:.3f} s, {design.metrics(t, y, STEP)['rise_s']:.3f} s)",
        ]
    (t1, w1), (t2, w2) = runs.values()
    a = compare.agreement(t, compare.on_grid(t1, w1, t), compare.on_grid(t2, w2, t), STEP)
    out += ["", f"PySpice vs ngspice -b: max difference {a['max_err_pct']:.2e} % of the setpoint",
            f"Linear solver, shared library (stderr notice PySpice must not treat as an error): "
            f"{'; '.join(spice.notes) or '(none printed)'}",
            f"Linear solver, ngspice -b: {spice.batch_solver_line('linear')}"]
    out += ["", "What limits the agreement (PySpice, linear loop):",
            f"{'op-amp gain':>12} {'reltol':>8} {'max error':>10} {'error at 3 s':>13} {'10.01 V / gain':>15}"]
    for gain, opts in ((1e3, None), (1e4, None), (1e5, None), (1e6, None), (1e8, None), (1e6, {}),
                       (1e6, {"reltol": 1e-6})):
        ts, ws, _ = spice.run_pyspice("linear", opamp_gain=gain, options=opts)
        a = compare.agreement(t, y, compare.on_grid(ts, ws, t), STEP)
        rt = (circuit.OPTIONS if opts is None else opts).get("reltol", 1e-3)
        out.append(f"{gain:>12.0e} {rt:>8.0e} {a['max_err_pct']:>8.4f} % {a['final_err_pct']:>11.5f} % "
                   f"{10.01 / gain / STEP * 100:>13.5f} %")
    out += ["(reltol 1e-3 is ngspice's default. An op-amp integrator with open-loop gain A is a leaky",
            " integrator with DC gain A, so the loop settles with error = 10.01 V / A.)"]
    report.write("step2_spice.txt", out)


if __name__ == "__main__":
    main()
