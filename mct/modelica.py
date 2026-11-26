"""Step 3, OpenModelica: run modelica/simulate.mos with omc and read the CSVs back.

omc compiles each model to C, builds an executable and runs it (DASSL by default),
so this needs a full OpenModelica install. Locally there is none; CI runs omc from
the official Docker image through a small wrapper script on the PATH.
"""
from __future__ import annotations

import csv
import shutil
import subprocess
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
BUILD = REPO / "build" / "modelica"  # simulate.mos expects to run two levels below the repo
SCRIPT = REPO / "modelica" / "simulate.mos"

MODELS = {
    "linear": "SpeedLoop",
    "saturated": "SpeedLoopSaturated",
    "antiwindup": "SpeedLoopAntiWindup",
    "limpid_no_tracking": "SpeedLoopLimPIDNoTracking",
    "linear_tol1e-6": "SpeedLoop_tol1e-6",  # the same linear model at looser solver tolerances
    "linear_tol1e-4": "SpeedLoop_tol1e-4",
}


def have_omc() -> bool:
    return shutil.which("omc") is not None


def run_all(timeout: float = 900) -> str:
    """Run every simulation in simulate.mos; return omc's output (also in build/modelica/omc.log)."""
    BUILD.mkdir(parents=True, exist_ok=True)
    for f in BUILD.glob("*_res.csv"):
        f.unlink()
    proc = subprocess.run(["omc", "../../modelica/simulate.mos"], cwd=BUILD,
                          capture_output=True, text=True, timeout=timeout)
    log = proc.stdout + proc.stderr
    (BUILD / "omc.log").write_text(log, encoding="utf-8")
    missing = [m for m in MODELS.values() if not result_path(m).exists()]
    if proc.returncode != 0 or missing:
        raise RuntimeError(f"omc failed (exit {proc.returncode}), no result for {missing}:\n{log[-4000:]}")
    return log


def ensure() -> None:
    """Run omc unless every CSV exists and is newer than the model and the script."""
    newest_source = max(SCRIPT.stat().st_mtime, (REPO / "modelica" / "MotorControl.mo").stat().st_mtime)
    paths = [result_path(m) for m in MODELS.values()]
    if not all(p.exists() and p.stat().st_mtime > newest_source for p in paths):
        run_all()


def timings() -> dict[str, dict[str, float]]:
    """timeCompile, timeSimulation and timeTotal per run, from the SimulationResult records."""
    out, current = {}, None
    for line in (BUILD / "omc.log").read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("resultFile ="):
            current = Path(line.split('"')[1]).name.removeprefix("MotorControl.").removesuffix("_res.csv")
            out[current] = {}
        elif current and line.startswith(("timeCompile", "timeSimulation", "timeTotal")):
            key, value = line.rstrip(",").split("=")
            out[current][key.strip()] = float(value)
    return out


def msl_version() -> str:
    """The Modelica Standard Library version simulate.mos printed."""
    for line in (BUILD / "omc.log").read_text(encoding="utf-8").splitlines():
        if line.startswith("Modelica Standard Library"):
            return line.split()[-1]
    return "?"


def result_path(model: str) -> Path:
    return BUILD / f"MotorControl.{model}_res.csv"


def read(model: str) -> dict[str, np.ndarray]:
    """One CSV written by simulate(..., outputFormat="csv") as {column: array}."""
    with open(result_path(model), newline="") as f:
        rows = list(csv.reader(f))
    header = [h.strip().strip('"') for h in rows[0]]
    data = np.array([[float(x) for x in r] for r in rows[1:] if r], dtype=float)
    return {h: data[:, i] for i, h in enumerate(header)}


def waveform(case: str):
    """(t, speed rad/s, armature voltage V) for one case."""
    d = read(MODELS[case])
    return d["time"], d["motor.w"], d["motor.v"]
