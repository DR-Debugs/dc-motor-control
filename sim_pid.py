"""Simulate closed-loop PID speed control of a DC motor and plot the result."""
import numpy as np
import matplotlib.pyplot as plt

from src.motor_model import MotorParams, dynamics
from src.pid import PID

DT = 0.001        # control loop period (s), i.e. 1 kHz
T_END = 3.0       # simulation length (s)
SETPOINT = 1.0    # target speed (rad/s)


def rk4_step(x, u, p, dt):
    k1 = dynamics(x, u, p)
    k2 = dynamics(x + 0.5 * dt * k1, u, p)
    k3 = dynamics(x + 0.5 * dt * k2, u, p)
    k4 = dynamics(x + dt * k3, u, p)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def main():
    p = MotorParams()
    pid = PID(kp=100, ki=200, kd=10, dt=DT, u_min=-p.V_max, u_max=p.V_max)

    t = np.arange(0, T_END, DT)
    x = np.zeros(2)
    omega_log, u_log = [], []

    for _ in t:
        u = pid.update(SETPOINT, x[0])
        x = rk4_step(x, u, p, DT)
        omega_log.append(x[0])
        u_log.append(u)

    omega_log = np.array(omega_log)
    final = omega_log[-1]
    overshoot = max(0.0, (omega_log.max() - SETPOINT) / SETPOINT * 100)
    print(f"Final speed: {final:.4f} rad/s | Overshoot: {overshoot:.1f}%")

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax1.plot(t, omega_log, label="speed")
    ax1.axhline(SETPOINT, ls="--", c="gray", label="setpoint")
    ax1.set_ylabel("Speed (rad/s)")
    ax1.set_title("DC Motor PID Speed Control")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(t, u_log, c="tab:orange")
    ax2.set_ylabel("Voltage (V)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True)
    fig.tight_layout()
    fig.savefig("results/pid_step_response.png", dpi=150)
    print("Saved results/pid_step_response.png")


if __name__ == "__main__":
    main()
