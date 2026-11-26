"""Small helpers for the results files and plots."""
from __future__ import annotations

import os
import platform
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
RESULTS = REPO / "results"

# The reference categorical palette (validated light-mode steps): slot 1 blue, 2 orange, 3 aqua.
COLORS = {"python-control": "#2a78d6", "ngspice": "#eb6834", "OpenModelica": "#1baf7a"}
INK = "#0b0b0b"
MUTED = "#52514e"
GRID = "#e4e3df"


def where() -> str:
    """Where a result came from: CI run or local machine."""
    if os.environ.get("GITHUB_ACTIONS") == "true":
        run = os.environ.get("GITHUB_RUN_ID", "?")
        sha = os.environ.get("GITHUB_SHA", "?")[:7]
        return f"GitHub Actions run {run} (commit {sha}), {platform.platform()}"
    return f"local, {platform.platform()}"


def header(title: str) -> list[str]:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    return [title, "=" * len(title), f"Measured: {stamp}, {where()}", ""]


def write(name: str, lines: list[str]) -> Path:
    RESULTS.mkdir(exist_ok=True)
    path = RESULTS / name
    path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"-> {path.relative_to(REPO)}")
    return path


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
