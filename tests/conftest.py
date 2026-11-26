"""Shared, expensive results: each solver runs once per test session."""
import pytest

from mct import design, modelica
from mct.params import STEP, T_END


@pytest.fixture(scope="session")
def pc_linear():
    """python-control's closed-loop step on the 3 s grid: (t, speed)."""
    return design.step(design.closed_loop(), STEP, design.grid(T_END))


@pytest.fixture(scope="session")
def om_results():
    """Every OpenModelica simulation, run once: {case: (t, speed, voltage)}."""
    modelica.run_all()
    return {case: modelica.waveform(case) for case in modelica.MODELS}
