"""LQR speed controller with integral action."""
import numpy as np
from scipy.linalg import solve_continuous_are


def augment_with_integrator(A, B, C):
    """Add an integral-of-error state: xi_dot = r - y."""
    n = A.shape[0]
    A_aug = np.block([[A, np.zeros((n, 1))],
                      [-C, np.zeros((1, 1))]])
    B_aug = np.vstack([B, [[0.0]]])
    return A_aug, B_aug


def is_controllable(A, B):
    n = A.shape[0]
    ctrb = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(n)])
    return np.linalg.matrix_rank(ctrb) == n


def lqr_gain(A, B, Q, R):
    """Solve the continuous algebraic Riccati equation and return K."""
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)

class LQR:
    def __init__(self, K, dt, u_min, u_max):
        self.K = K
        self.dt = dt
        self.u_min, self.u_max = u_min, u_max
        self.xi = 0.0  # integral of speed error

    def update(self, setpoint, x):
        """x = [omega, current] (full-state feedback)."""
        z = np.array([x[0], x[1], self.xi])
        u_unsat = float(-self.K[0] @ z)
        u = min(max(u_unsat, self.u_min), self.u_max)

        # Anti-windup, same idea as the PID
        if u == u_unsat:
            self.xi += (setpoint - x[0]) * self.dt
        return u