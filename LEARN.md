# Learning notes: motor-control-three-ways

## 1. The loop on one page

A DC motor turns voltage into speed through two coupled equations:

    electrical   L di/dt = v - R i - K w        (back-EMF K w opposes the supply)
    mechanical   J dw/dt = K i - b w            (torque K i against viscous friction b w)

Laplace-transform both and eliminate i: W(s)/V(s) = K / ((J s + b)(L s + R) + K²). With the CTMS numbers that is 0.01 / (0.005 s² + 0.06 s + 0.1001): poles at −2.0 and −10.0 rad/s, and 10.01 V for 1 rad/s.

A **PI controller** sets v = kp·e + ki·∫e, with e = setpoint − speed. The P part reacts now; the I part keeps adding until the error is zero, which is why the steady-state error goes to zero. Here kp = 20, ki = 40: the PI's zero (s = −ki/kp = −2) sits on the slow motor pole and cancels it, leaving a loop of about 40 / (s(s + 10)), a second-order closed loop with damping 0.79.

The same equations, three ways:
- **python-control:** the transfer functions themselves.
- **ngspice:** a circuit whose node equations are the same equations (speed is a voltage, torque a current, inertia a capacitor).
- **OpenModelica:** physical components (resistor, inductor, EMF, inertia, damper) connected by ports; the tool derives the equations.

If all three agree to 0.001 %, each tool and each model is doing what I think it is.

## 2. python-control

**What it is:** a Python library for linear (and some nonlinear) control-system analysis, in the spirit of MATLAB's Control System Toolbox. Version 0.10.2 here.

**The concepts that matter:**
- **LTI systems:** `tf` (transfer functions) and `ss` (state space). You can multiply them (series), and `feedback(L, 1)` closes a unity-feedback loop: L / (1 + L).
- **Time response and metrics:** `step_response(sys, T)` returns a response object; `step_info` gives overshoot, rise time (10–90 %), settling time (2 % band) and peak, either for a system or for a waveform plus its time vector.
- **Stability margins:** `margin(L)` returns gain margin, phase margin and the two crossover frequencies. Phase margin is how much extra lag the loop tolerates at the frequency where |L| = 1; gain margin is how much extra gain it tolerates where the phase is −180°.
- **Discretisation:** `c2d(sys, Ts, method)`. A digital controller uses **Tustin** (s → (2/T)(z−1)/(z+1), keeps stability, warps frequency a little); the plant it drives through a DAC is the **zero-order hold** discretisation. Sampling adds about half a sample of delay, so the gain margin becomes finite.
- **Nonlinear I/O systems:** `nlsys(updfcn, outfcn, inputs, outputs, states)` wraps any Python dynamics; `interconnect` wires systems together by signal name; `input_output_response` integrates with scipy's `solve_ivp`.

**Where it is used:** `mct/design.py` (plant, PI, `step_info`, `margin`, `c2d`), `mct/nonlinear.py` (saturated PI as an `nlsys`), and as the reference every other tool is compared with.

**Pitfalls met:**
- `step_info(sys)` picks its own time grid, so the settling time came out 0.604 s instead of 0.583 s on a 1 ms grid. Measure everything on the same grid.
- For the discrete loops python-control warns that it falls back to a frequency sweep for margins. Fine, but check it once: a test sweeps L(e^jωT) itself and agrees within 0.1 %.
- A gain margin of `inf` is not a bug: the continuous loop's phase only approaches −180°.
- `input_output_response` returns `NamedSignal` arrays; `round()` on them failed, so the code converts with `np.asarray`.
- A clipped output makes the right-hand side non-smooth: keep `max_step` short (1 ms) and tolerances tight, or the solver steps over the corner.

## 3. PySpice

**What it is:** a Python front end for SPICE simulators. You build a `Circuit` object in Python, it writes the netlist, runs ngspice (as a shared library or a subprocess) and returns the waveforms as arrays. Version 1.5 here.

**The concepts that matter:**
- **Element methods map to SPICE letters:** `cir.R("a", n1, n2, 1.0)` becomes `Ra n1 n2 1.0`; `VCVS` is E, `CCCS` is F, `BehavioralSource` is B, `PieceWiseLinearVoltageSource` is a PWL V source. A CCCS names its controlling source by its full name (`"Vsense"`).
- **`str(circuit)` is the netlist.** That makes the circuit reviewable and testable without a simulator (the tests compare `netlists/*.cir` with the generator).
- **Simulators:** `circuit.simulator(simulator="ngspice-shared")` loads libngspice into Python; `transient(step_time, end_time, max_time)` runs `.tran`; the analysis gives `analysis.time` and `analysis["node"]`.
- **Parameterised circuits:** because the circuit is Python, a sweep is a loop (`build(c=PI(kp, 2*kp))`).

**Where it is used:** `mct/circuit.py` (building), `mct/spice.py` (running, both ways).

**Pitfalls met:**
- PySpice 1.5 treats any ngspice stderr line not starting with `Warning:` as an error. ngspice 42 prints `Using SPARSE 1.3 as Direct Linear Solver` on stderr, so every run "failed" after succeeding. Fixed by a subclass that tolerates exactly that line.
- On Linux it dlopens `libngspice.so`; Ubuntu's `libngspice0` only has `libngspice.so.0`. Set `NGSPICE_LIBRARY_PATH`.
- On Windows it looks for a bundled `Spice64_dll/dll-vs/ngspice.dll` (downloaded by a post-install command this project doesn't run), so locally only netlist generation works.
- On Windows `str(circuit)` uses CRLF line ends and has no `.end`.
- It logs `Unsupported Ngspice version 42` and uses its newest table; the answers matched `ngspice -b` exactly, so harmless here.

## 4. ngspice

**What it is:** the open-source SPICE circuit simulator (descended from Berkeley SPICE3). It writes the circuit as modified nodal analysis equations (Kirchhoff's current law at every node, plus one equation per voltage source) and integrates them in time.

**The concepts that matter:**
- **Operating point, then transient:** `.tran` first solves the DC operating point (capacitors open, inductors shorted), then steps in time with an adaptive step. `max_time` caps the step; `reltol` sets the accuracy target (default 10⁻³).
- **Controlled sources are the building blocks of models:** E (voltage-controlled voltage source: an ideal op-amp or the back-EMF), F (current-controlled current source: torque = K·i), G (voltage-controlled current source), B (behavioural: any expression, like `min(max(V(v),-12),12)`).
- **Analogs:** a capacitor is an integrator (C dV/dt = I), so inertia becomes a capacitor and speed a voltage; friction becomes a resistor. Any linear ODE can be built this way.
- **Sign conventions:** a current source's positive current flows from its first node through the source to its second, so `Ftorque 0 w ...` pushes current into node `w`. A voltage source's current is positive into its + node, so `Vsense` reads the armature current.
- **Batch mode:** `ngspice -b deck.cir` with a `.control` block (`run`, `wrdata file v(w)`) needs no Python at all.

**Where it is used:** the decks in `netlists/`, run in CI through PySpice and through `ngspice -b`.

**Pitfalls met:**
- An ideal step can't be a source: the setpoint rises in 1 µs.
- Op-amps modelled as gain-10⁶ VCVS are not ideal enough to ignore: an op-amp integrator is a leaky integrator with DC gain A, so the loop settles 10.01 V / A low (0.001 % at A = 10⁶, 1 % at A = 10³). Changing reltol from 10⁻³ to 10⁻⁶ changed nothing: the model, not the solver, set the error.
- Hard rails on high-gain op-amps (±15 V at gain 10⁶) stopped the transient: `Timestep too small; ... trouble with node "u4n"`. Gear integration and a smooth tanh limit didn't help; gain 10⁴ did.
- The integrator op-amp has no DC feedback path; the operating point still solves because the whole loop closes through the motor at DC.

## 5. OpenModelica and the Modelica Standard Library

**What it is:** Modelica is an equation-based language for physical systems: components declare equations, connections declare that potentials are equal and flows sum to zero, and the compiler sorts it all into a solvable system. OpenModelica's compiler `omc` turns a model into C, compiles it, and runs it with a DAE solver (DASSL by default). The Modelica Standard Library (MSL) has the components: electrical, rotational mechanics, blocks.

**The concepts that matter:**
- **Acausal connectors:** an electrical pin has voltage (potential) and current (flow); a rotational flange has angle and torque. `connect(emf.flange, inertia.flange_a)` means "same angle, torques sum to zero". You never write the motor equations by hand; `RotationalEMF` couples the two domains (v = k·w, τ = k·i).
- **Causal blocks** (`Blocks.Continuous.PI`, `Math.Feedback`, `Nonlinear.Limiter`, `Continuous.LimPID`) carry signals with a direction, for the controller.
- **Initialisation:** states need start values; `fixed = true` makes them initial equations (inductor current 0, rotor at rest), `initType = InitialState` does the same for a controller's integrator.
- **Scripting:** a `.mos` file is a sequence of omc commands: `loadModel`, `loadFile`, `simulate(Model, stopTime, numberOfIntervals, tolerance, outputFormat = "csv", variableFilter = ...)`, `getErrorString()` after each call to see what went wrong.
- **LimPID anti-windup:** the integrator input gets an extra (limited output − unlimited output)/(k·Ni) term, which is back-calculation with tracking time Ni·Ti.

**Where it is used:** `modelica/MotorControl.mo`, `modelica/simulate.mos`, run by `mct/modelica.py`.

**Pitfalls met:**
- The `-minimal` Docker image has no MSL; `installPackage(Modelica, "4.1.0")` fetches it.
- In MSL 4 the EMF is `RotationalEMF` (MSL 3.2.3 called it `EMF`) and units live in `Modelica.Units.SI`.
- `variableFilter` is a regular expression and the .mos string needs `"\\."` for a literal dot.
- Running omc in Docker: mount the workspace at the same absolute path so relative paths in the .mos work, run as the runner's user so files aren't owned by root, and give it a writable HOME for the package manager.
- The linear CSV has 3,002 rows for 3,001 output instants, so don't assume one row per interval. The reader keeps the last row for any repeated time before interpolating.
- Tolerance matters: at 10⁻⁴ the speed was 0.016 % off, at the default 10⁻⁶ 0.00023 %, at 10⁻⁸ 0.00001 %.

## 6. The finding: integrator windup

A 1 rad/s step needs 10.01 V forever, but the PI asks for 20 V at once (kp·e with e = 1) and up to 21.18 V as the integral fills. With a 12 V supply, the motor gets 12 V while the integrator keeps integrating an error the actuator can't remove. By the time the speed reaches the setpoint, the integral holds far more than 10 V, so the output stays at 12 V past the setpoint: 14.86 % overshoot, 2.909 s to settle, against 1.71 % and 0.583 s without a limit.

The linear transfer function can't show this: it has no idea there is a supply. Every tool that models the limit (python-control `nlsys`, ngspice B source, Modelica Limiter) shows the same numbers within 0.003 %.

**Back-calculation** feeds the difference between what was asked and what was delivered back into the integrator: dI/dt = ki·e + (u − v)/Tt. While saturated the integrator is pulled towards the value that just saturates the output, so it is ready the moment the speed arrives. With Tt = 0.45 s (LimPID's default, 0.9·Ti): 0 % overshoot, 1.047 s. Too short a Tt (0.05 s) keeps the integrator nearly empty, and the speed creeps the last few percent: 1.895 s. Too long (2 s) and it is nearly no anti-windup: 9.66 %.

Other fixes, not built here: conditional integration (stop integrating while saturated), clamping the integrator, or rails on the integrator op-amp itself (an exploration measured 12.44 % at ±15 V).

## 7. What broke while building

1. PySpice reported every ngspice 42 run as failed because of a solver notice on stderr.
2. PySpice wanted `libngspice.so`; Ubuntu ships `libngspice.so.0`.
3. PySpice netlists on Windows: CRLF and no `.end`.
4. The minimal OpenModelica image had no MSL.
5. `step_info` on a system gave a different settling time from the same response on a 1 ms grid.
6. The circuit settled 0.001 % low: finite op-amp gain, not solver tolerance.
7. Faster anti-windup tracking made settling slower.
8. Op-amp rails at gain 10⁶ made ngspice give up ("Timestep too small").

## 8. Explain it without notes

1. Derive the motor's transfer function from the two equations. Where do the two poles come from?
2. Why does a PI give zero steady-state error to a step, and a P controller not?
3. Why put the PI's zero on the slow motor pole? What does it cost when a load torque arrives?
4. What do gain margin and phase margin measure? Why is the gain margin infinite here but 54 dB at 1 kHz sampling?
5. Tustin versus zero-order hold: which one discretises the controller, which one the plant, and why?
6. How do inertia, friction and torque become circuit elements? Which node voltage is the speed?
7. What are E, F, G and B elements in SPICE? Which one is the op-amp, the back-EMF, the torque?
8. Why does an op-amp integrator with gain A leave an error of 10.01 V / A? How would you check it?
9. What does "acausal" mean in Modelica? What flows through a rotational flange?
10. What does omc do between `simulate(...)` and the CSV appearing?
11. Explain integrator windup with this motor's numbers. Why can't the linear model see it?
12. How does back-calculation work, and what happens if Tt is too short or too long?
13. The three tools agree to 0.001 %. What does that prove, and what doesn't it prove?

## 9. Your turn

One exercise per tool, 20–40 minutes each. Exercises 2–4 need ngspice or omc, which only run in CI: work on a branch and let CI run them.

```bash
git switch -c exercise-<name>          # one branch per exercise
# edit, then
git commit -am "exercise: <name>" && git push -u origin exercise-<name>
gh run watch                            # pick the run; about 2 minutes
gh run view --log | grep -A20 "<your step name>"
gh run download <run-id> -n results     # your results/ and logs
```
Add your own step to `.github/workflows/ci.yml` after "measurements", for example `- run: python scripts/my_exercise.py`. Delete the branch when done (`git push origin --delete exercise-<name>`).

### Exercise 1: python-control, retune the PI and read the margins (local, 20 min)

1. Run the design for other gains without touching the files:
   ```bash
   python -c "from mct import design as d; from mct.params import PI; c = PI(40, 80); t, y = d.step(d.closed_loop(c=c)); print(d.metrics(t, y, 1.0)); print(d.margins(d.loop(c=c)))"
   ```
2. Keep the zero on the motor pole (ki = 2·kp) and raise kp. Find the largest kp that keeps overshoot under 5 % (bisect by hand, or `scipy.optimize.brentq`).
3. Then move the zero: kp = 20, ki = 100.
4. Plot the step responses on one chart and the loop Bode plots on another.

Expected (python-control, measured locally while writing this):

| kp, ki | Overshoot | Settling | Rise | Phase margin |
|---|---|---|---|---|
| 20, 40 (the design) | 1.71 % | 0.583 s | 0.384 s | 69.5° at 3.75 rad/s |
| 26, 52 | 4.84 % | 0.830 s | 0.293 s | 64.8° at 4.71 rad/s |
| 26.29, 52.58 | 5.00 % (the limit) | 0.826 s | 0.289 s | 64.6° |
| 40, 80 | 12.01 % | 0.655 s | 0.196 s | 56.4° at 6.66 rad/s |
| 20, 100 | 30.18 % | 1.598 s | 0.246 s | 40.8° at 4.80 rad/s |

What to notice: with the zero on −2 the closed-loop poles' real part stays at −5, so more gain only adds overshoot and makes the 2 % settling time worse (the overshoot now leaves the band). Moving the zero away un-cancels the slow pole and the margins fall. To be faster, the PI alone is not enough: try adding a D term (CTMS uses kp = 100, ki = 200, kd = 10) and compare.

### Exercise 2: PySpice, drive the netlist from a parameter sweep (CI, 30 min)

1. Write `scripts/sweep_kp.py`: for kp in 10, 20, 40, run `spice.run_pyspice("linear", c=PI(kp, 2 * kp))` and `design.step(design.closed_loop(c=PI(kp, 2 * kp)))`, put both on the 1 ms grid with `compare.on_grid`, and print the overshoot of each and the max difference.
2. Print `circuit.netlist("linear", c=PI(40, 80))` locally first and find the line that changed (`Rp2`). Which resistor would you change on a real board, and what about ki?
3. Add the script as a CI step, push, read the log.

Expected (measured in CI while writing this exercise):

| kp | ngspice overshoot | python-control overshoot | Max difference |
|---|---|---|---|
| 10 | 0.000 % | 0.000 % | 0.0009 % |
| 20 | 1.707 % | 1.707 % | 0.0013 % |
| 40 | 12.008 % | 12.008 % | 0.0028 % |

Stretch: sweep the op-amp gain the same way and reproduce the 10.01 V / A table in `results/step2_spice.txt`.

### Exercise 3: ngspice, a real op-amp instead of an ideal one (CI, 40 min)

1. In `mct/circuit.py`, give `build()` an option for a single-pole op-amp model and use it in `opamp()` instead of the VCVS: a VCCS `G` (gm = 1 mS, controlled by v+ − v−) into an internal node, a resistor A0/gm and a capacitor from that node to ground (pole at GBW/A0), and a unity VCVS buffer from the internal node to the output. That gives DC gain A0 and gain-bandwidth GBW.
2. Run the linear loop with A0 = 10⁵ and GBW = 1 MHz, 1 kHz, 100 Hz, 10 Hz. Compare with python-control.
3. Optional: give the op-amps ±15 V rails with a B source, `V = min(max(A*(V(p)-V(n)),-15),15)`, and run the saturated (windup) case.

Expected (measured in CI while writing this exercise, dev-branch runs 45818725645 and 45819130464):

| A0 = 10⁵, GBW | Max error vs python-control | Overshoot | Settling |
|---|---|---|---|
| 1 MHz | 0.011 % (just the DC gain: 10.01 V / A0) | 1.704 % | 0.583 s |
| 1 kHz | 0.86 % | 1.789 % | 0.575 s |
| 100 Hz | 8.7 % | 4.71 % | 0.872 s |
| 10 Hz | 50 % | 40.3 % | does not settle in 3 s |

The loop crosses over at 0.6 Hz, yet a 1 kHz op-amp already costs almost 1 %: the P stage has gain 20, so its own bandwidth is only GBW/21, and a step asks every stage for a fast edge. With rails: at gain 10⁶ ngspice stops with `Timestep too small ... trouble with node "u4n"`; drop the gain to 10⁴ and it runs, with the windup overshoot down from 14.86 % to 12.44 % (the integrator op-amp can't wind past its rail). Explain both before looking anything up.

### Exercise 4: OpenModelica, a load torque step (CI, 30 min)

1. In `modelica/MotorControl.mo`, add a model `MotorWithLoad` that `extends Motor` and adds `Modelica.Mechanics.Rotational.Sources.TorqueStep load(stepTorque = -tauLoad, offsetTorque = 0, startTime = 1)` with `connect(load.flange, inertia.flange_b)` and a parameter `tauLoad = 0.01` (N·m). A negative step torque brakes the rotor.
2. Copy `SpeedLoop` to `SpeedLoopLoad` using `MotorWithLoad motor;`. Add a `simulate(MotorControl.SpeedLoopLoad, stopTime = 3.0, ...)` line to `simulate.mos` and `"load": "SpeedLoopLoad"` to `MODELS` in `mct/modelica.py`.
3. Write a script or test that reads it with `modelica.waveform("load")` and prints the lowest speed after t = 1 s, when it is back within 2 % of 1 rad/s, and the final armature voltage. Check the same thing in python-control: the transfer function from load torque to speed is −Pd/(1 + C·P) with Pd = (L s + R)/((J s + b)(L s + R) + K²).
4. Then put the same load (0.05 N·m) on `SpeedLoopAntiWindup`.

Expected:
- 0.01 N·m (python-control prediction, computed locally): the speed dips to 0.936 rad/s (6.4 %) at 1.18 s and is back within 2 % at 1.50 s; the armature voltage settles 1.00 V higher, at 11.01 V (Td·R/K more). Modelica should match within 0.01 %. The recovery runs on the slow −2 rad/s pole the PI zero cancelled for setpoint steps: cancelled for the setpoint, not for the load.
- 0.05 N·m with the 12 V supply (hand calculation): holding 1 rad/s would need 15.01 V, so the speed settles at (12·K/R − 0.05)/(b + K²/R) = 0.699 rad/s. With LimPID the unlimited controller output settles Tt·ki·e = 5.4 V above the limit instead of growing; with PI + Limiter the integral keeps climbing at ki·e = 12 V/s. Check the unlimited outputs `pid.limiter.u` and `pi.y` in the CSV (add them to `variableFilter`).
