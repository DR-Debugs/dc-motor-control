"""Brushed DC motor model.

States: x = [omega (rad/s), current (A)]
Input:  u = voltage (V)

    J * d(omega)/dt = Kt*i - b*omega
    L * di/dt       = V - R*i - Ke*omega
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class MotorParams:
    J: float = 0.01    # rotor inertia (kg*m^2)
    b: float = 0.1     # viscous friction (N*m*s)
    Kt: float = 0.01   # torque constant (N*m/A)
    Ke: float = 0.01   # back-EMF constant (V*s/rad)
    R: float = 1.0     # armature resistance (ohm)
    L: float = 0.5     # armature inductance (H)
    V_max: float = 24.0  # supply voltage limit (V)


def dynamics(x, u, p: MotorParams):
    """Return dx/dt for the motor."""
    omega, i = x
    domega = (p.Kt * i - p.b * omega) / p.J
    di = (u - p.R * i - p.Ke * omega) / p.L
    return np.array([domega, di])


def state_space(p: MotorParams):
    """Return (A, B, C) matrices, which you'll need for LQR."""
    A = np.array([[-p.b / p.J, p.Kt / p.J],
                  [-p.Ke / p.L, -p.R / p.L]])
    B = np.array([[0.0], [1.0 / p.L]])
    C = np.array([[1.0, 0.0]])
    return A, B, C
