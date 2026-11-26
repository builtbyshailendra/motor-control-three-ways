"""Comparing waveforms from different tools on one time grid."""
from __future__ import annotations

import numpy as np


def on_grid(t_src, y_src, t):
    """Linear interpolation of a solver's own time points onto the comparison grid.

    Solvers with events (Modelica) write two rows at the same instant; keep the last.
    """
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    keep = np.r_[t_src[1:] != t_src[:-1], True]
    return np.interp(t, t_src[keep], y_src[keep])


def agreement(t, y_ref, y_tool, scale: float) -> dict:
    """Error of y_tool against y_ref, both on grid t, in percent of `scale` (the setpoint)."""
    err = np.asarray(y_tool) - np.asarray(y_ref)
    k = int(np.argmax(np.abs(err)))
    return {
        "max_err_pct": float(abs(err[k]) / scale * 100),
        "at_s": float(t[k]),
        "rms_err_pct": float(np.sqrt(np.mean(err**2)) / scale * 100),
        "final_err_pct": float(abs(err[-1]) / scale * 100),
    }
