"""The numbers every tool uses: motor, controller, supply, step.

Motor: the DC motor speed example of "Control Tutorials for MATLAB and Simulink"
(CTMS, University of Michigan, Carnegie Mellon and Detroit Mercy), section
"DC Motor Speed: System Modeling":
https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling

The design specs come from the same tutorial's "PID Controller Design" page:
settling time under 2 s, overshoot under 5 %, steady-state error under 1 %.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Motor:
    J: float = 0.01  # rotor inertia, kg m^2
    b: float = 0.1  # viscous friction, N m s/rad
    K: float = 0.01  # torque constant N m/A, equal to the back-EMF constant V s/rad
    R: float = 1.0  # armature resistance, ohm
    L: float = 0.5  # armature inductance, H


@dataclass(frozen=True)
class PI:
    """u = kp*e + ki*integral(e), e = setpoint - speed."""

    kp: float = 20.0  # V per rad/s
    ki: float = 40.0  # V per rad

    @property
    def ti(self) -> float:
        """Integral time Ti = kp/ki, s (Modelica's PI calls it T, LimPID calls it Ti)."""
        return self.kp / self.ki


MOTOR = Motor()
CONTROLLER = PI()

STEP = 1.0  # setpoint step, rad/s (needs 10 V at steady state)
T_END = 3.0  # simulated time for the linear comparison, s
T_WINDUP = 5.0  # simulated time for the windup study, s (recovery takes about 3 s)
DT = 1e-3  # comparison grid, s

V_SUPPLY = 12.0  # supply limit for the windup study, +/- V
NI = 0.9  # Modelica LimPID's default anti-windup factor
TT = NI * CONTROLLER.ti  # back-calculation (tracking) time constant, s = 0.45 s

SPECS = {"overshoot_pct": 5.0, "settling_s": 2.0, "steady_state_error_pct": 1.0}
