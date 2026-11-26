"""Step 1: plant, PI design, margins, step metrics, discretisation (python-control only).

    python scripts/step1_design.py      -> results/step1_design.txt, results/loop_bode.png
"""
import _setup  # noqa: F401
import control as ct
import matplotlib.pyplot as plt
import numpy as np

from mct import design as d
from mct import report
from mct.params import CONTROLLER, MOTOR, SPECS, STEP


def main():
    m, c = MOTOR, CONTROLLER
    ol = d.open_loop_summary()
    out = report.header("Step 1: plant, PI design and discretisation (python-control "
                        f"{ct.__version__})")
    out += [
        f"Motor (CTMS): J={m.J} kg m^2, b={m.b} N m s, K={m.K} N m/A, R={m.R} ohm, L={m.L} H",
        f"Plant W(s)/V(s) = {m.K} / ({m.J*m.L:g} s^2 + {m.J*m.R + m.b*m.L:g} s + {m.b*m.R + m.K**2:g})",
        f"Open-loop poles: {ol['poles'][0]:.4f}, {ol['poles'][1]:.4f} rad/s "
        f"(time constants {ol['time_constants_s'][0]*1000:.1f} ms, {ol['time_constants_s'][1]*1000:.1f} ms)",
        f"DC gain: {ol['dc_gain_rad_s_per_V']:.5f} rad/s per V, so 1 rad/s needs {ol['volts_for_1_rad_s']:.2f} V",
        f"Open-loop 1 V step: rise (10-90 %) {ol['rise_s']:.3f} s, settling (2 %) {ol['settling_s']:.3f} s",
        "",
        f"PI: kp={c.kp} V per rad/s, ki={c.ki} V per rad (Ti = {c.ti} s); zero at s = {-c.ki/c.kp:g}, "
        f"on the slow motor pole",
    ]
    L = d.loop()
    cl = d.closed_loop()
    mg = d.margins(L)
    poles = sorted(ct.poles(cl), key=lambda p: p.real)
    out += [
        "Closed-loop poles: " + ", ".join(f"{p.real:.4f}{p.imag:+.4f}j" for p in poles),
        f"Gain margin: {'infinite (phase never reaches -180 deg)' if not np.isfinite(mg['gain_margin']) else mg['gain_margin_db']}",
        f"Phase margin: {mg['phase_margin_deg']:.2f} deg at {mg['gain_crossover_rad_s']:.3f} rad/s",
    ]
    t, y = d.step(cl)
    met = d.metrics(t, y, STEP)
    out += [
        "",
        f"Closed-loop {STEP} rad/s step on a 1 ms grid (python-control step_info, 2 % band):",
        f"  overshoot {met['overshoot_pct']:.2f} %  (spec < {SPECS['overshoot_pct']} %)",
        f"  settling  {met['settling_s']:.3f} s  (spec < {SPECS['settling_s']} s)",
        f"  rise      {met['rise_s']:.3f} s, peak {met['peak']:.4f} rad/s at {met['peak_s']:.3f} s",
        f"  error at 3 s {met['steady_state_error_pct']:.4f} %  (spec < {SPECS['steady_state_error_pct']} %; "
        "the integrator makes it zero in the limit)",
        "",
        "Discretised: PI by Tustin, plant by zero-order hold; closed-loop step against the continuous one",
        f"{'sample time':>12} {'PI numerator':>18} {'max diff':>9} {'overshoot':>10} {'settling':>9} "
        f"{'gain margin':>12} {'phase margin':>13}",
    ]
    for ts in (1e-3, 1e-2, 5e-2, 1e-1):
        r = d.discrete_vs_continuous(ts)
        num = f"{r['pi_num'][0]:.3f} z {r['pi_num'][1]:+.3f}"
        out.append(f"{ts*1000:>9g} ms {num:>18} {r['max_diff_pct']:>7.3f} % {r['d_overshoot_pct']:>8.2f} % "
                   f"{r['d_settling_s']:>7.3f} s {r['d_gain_margin_db']:>8.1f} dB {r['d_phase_margin_deg']:>9.2f} deg")
    out.append("(PI denominator z - 1 in every case; the gain margin is finite once sampled)")
    report.write("step1_design.txt", out)
    bode(L, mg)


def bode(L, mg):
    w = np.logspace(-1, 3, 800)
    resp = ct.frequency_response(L, w)
    mag_db = 20 * np.log10(np.abs(np.squeeze(resp.complex)))
    phase = np.degrees(np.unwrap(np.angle(np.squeeze(resp.complex))))
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 5.2), sharex=True, dpi=110)
    blue = report.COLORS["python-control"]
    a1.semilogx(w, mag_db, color=blue, lw=2)
    a1.axhline(0, color=report.MUTED, lw=1)
    a1.set_ylabel("magnitude (dB)", color=report.INK)
    a1.set_title("Loop gain L(s) = PI(s) P(s)", color=report.INK, loc="left")
    a2.semilogx(w, phase, color=blue, lw=2)
    a2.axhline(-180, color=report.MUTED, lw=1)
    wc, pm = mg["gain_crossover_rad_s"], mg["phase_margin_deg"]
    for ax in (a1, a2):
        ax.axvline(wc, color=report.MUTED, lw=1, ls="--")
        report.style(ax)
    a2.annotate(f"phase margin {pm:.1f} deg\nat {wc:.2f} rad/s", xy=(wc, -180 + pm), xytext=(wc * 3, -120),
                color=report.INK, arrowprops=dict(arrowstyle="-", color=report.MUTED))
    a2.set_ylabel("phase (deg)", color=report.INK)
    a2.set_xlabel("frequency (rad/s)", color=report.INK)
    a2.set_ylim(-190, -80)
    fig.tight_layout()
    fig.savefig(report.RESULTS / "loop_bode.png")
    print("-> results/loop_bode.png")


if __name__ == "__main__":
    main()
