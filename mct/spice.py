"""Running the circuits in ngspice, two ways.

1. PySpice -> ngspice shared library (libngspice.so), the normal PySpice route.
2. `ngspice -b` on the deck PySpice generated, reading the `wrdata` text file back.

Both are only available where ngspice is installed (CI's Ubuntu runner). Running both
and comparing them checks the interface as well as the circuit.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

from .circuit import OPTIONS, STOP, TRAN, batch_deck, build


def have_ngspice() -> bool:
    return shutil.which("ngspice") is not None


# ngspice 42 prints which linear solver it uses on stderr. PySpice 1.5 treats every
# stderr line that doesn't start with "Warning:" as an error and raises
# NgSpiceCommandError after a successful `run`. These lines are information only.
BENIGN_STDERR = ("Using SPARSE 1.3 as Direct Linear Solver", "Using KLU as Direct Linear Solver")
_shared = None
notes: list[str] = []  # the benign stderr lines seen, for the results file


def shared():
    """One NgSpiceShared for the process, tolerant of the solver notice above."""
    global _shared
    if _shared is None:
        from PySpice.Spice.NgSpice.Shared import NgSpiceCommandError, NgSpiceShared

        class TolerantNgSpiceShared(NgSpiceShared):
            def exec_command(self, command, join_lines=True):
                try:
                    return super().exec_command(command, join_lines)
                except NgSpiceCommandError:
                    real = [x for x in self._stderr
                            if x.strip() and not x.strip().startswith(BENIGN_STDERR)]
                    if real or self._error_in_stdout:
                        raise
                    notes.extend(x.strip() for x in self._stderr if x.strip() not in notes)
                    return self.stdout if join_lines else self._stdout

        _shared = TolerantNgSpiceShared(ngspice_id=0, send_data=False)
    return _shared


def run_pyspice(mode: str = "linear", stop: float | None = None, options: dict | None = None, **kw):
    """(t, speed, supply voltage) from PySpice's NgSpiceShared. Extra keywords go to
    circuit.build (for example opamp_gain); `options` replaces the .options line."""
    stop = STOP[mode] if stop is None else stop
    cir = build(mode, **kw)
    sim = cir.simulator(temperature=25, nominal_temperature=25, simulator="ngspice-shared",
                        ngspice_shared=shared())
    if options is None:
        options = OPTIONS
    if options:
        sim.options(**options)
    an = sim.transient(step_time=TRAN["step"], end_time=stop, max_time=TRAN["max_step"])
    return (np.asarray(an.time, dtype=float), np.asarray(an["w"], dtype=float),
            np.asarray(an["va"], dtype=float))


def run_batch(mode: str = "linear", workdir: Path | str | None = None,
              stop: float | None = None, **kw):
    """(t, speed, supply voltage) from `ngspice -b` on the generated deck."""
    workdir = Path(workdir or os.environ.get("MCT_BUILD", "build")) / "ngspice" / mode
    workdir.mkdir(parents=True, exist_ok=True)
    (workdir / "deck.cir").write_text(batch_deck(mode, out="out.txt", stop=stop, **kw),
                                      encoding="ascii")
    out = workdir / "out.txt"
    out.unlink(missing_ok=True)
    proc = subprocess.run(["ngspice", "-b", "deck.cir"], cwd=workdir, capture_output=True,
                          text=True, timeout=300)
    (workdir / "ngspice.log").write_text(proc.stdout + proc.stderr, encoding="utf-8")
    if proc.returncode != 0 or not out.exists():
        raise RuntimeError(f"ngspice -b failed ({proc.returncode}):\n{proc.stdout[-2000:]}"
                           f"\n{proc.stderr[-2000:]}")
    data = np.loadtxt(out, skiprows=1)
    return data[:, 0], data[:, 1], data[:, 2]


def batch_solver_line(mode: str = "linear") -> str:
    """The linear-solver line from the last `ngspice -b` log, if it printed one."""
    log = Path(os.environ.get("MCT_BUILD", "build")) / "ngspice" / mode / "ngspice.log"
    for line in log.read_text(encoding="utf-8").splitlines():
        if "Linear Solver" in line:
            return line.strip(" *")
    return "(no solver line)"
