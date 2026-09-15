"""Compare PID and LQR speed control on the same motor."""
import numpy as np
import matplotlib.pyplot as plt

from src.motor_model import MotorParams, state_space
from src.pid import PID
from src.lqr import LQR, augment_with_integrator, lqr_gain
from sim_pid import rk4_step, DT, T_END, SETPOINT


def run(controller, p, uses_full_state):
    t = np.arange(0, T_END, DT)
    x = np.zeros(2)
    w_log, u_log = [], []
    for _ in t:
        if uses_full_state:
            u = controller.update(SETPOINT, x)
        else:
            u = controller.update(SETPOINT, x[0])
        x = rk4_step(x, u, p, DT)
        w_log.append(x[0])
        u_log.append(u)
    return t, np.array(w_log), np.array(u_log)


def metrics(t, w):
    rise = t[np.argmax(w >= 0.9 * SETPOINT)]
    overshoot = max(0.0, (w.max() - SETPOINT) / SETPOINT * 100)
    return rise, overshoot


def main():
    p = MotorParams()

    pid = PID(kp=100, ki=200, kd=10, dt=DT, u_min=-p.V_max, u_max=p.V_max)

    A, B, C = state_space(p)
    A_aug, B_aug = augment_with_integrator(A, B, C)
    Q = np.diag([1.0, 0.0, 1.0])   # [speed, current, integral error]
    R = np.array([[1.0]])          # cost on voltage
    K = lqr_gain(A_aug, B_aug, Q, R)
    lqr = LQR(K, DT, -p.V_max, p.V_max)

    t, w_pid, u_pid = run(pid, p, uses_full_state=False)
    _, w_lqr, u_lqr = run(lqr, p, uses_full_state=True)

    for name, w in [("PID", w_pid), ("LQR", w_lqr)]:
        rise, os_ = metrics(t, w)
        print(f"{name}: rise time {rise:.3f} s, overshoot {os_:.1f}%")

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    ax1.plot(t, w_pid, label="PID")
    ax1.plot(t, w_lqr, label="LQR")
    ax1.axhline(SETPOINT, ls="--", c="gray", label="setpoint")
    ax1.set_ylabel("Speed (rad/s)")
    ax1.set_title("PID vs. LQR Speed Control")
    ax1.legend()
    ax1.grid(True)
    ax2.plot(t, u_pid, label="PID")
    ax2.plot(t, u_lqr, label="LQR")
    ax2.set_ylabel("Voltage (V)")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    ax2.grid(True)
    fig.tight_layout()
    fig.savefig("results/pid_vs_lqr.png", dpi=150)
    print("Saved results/pid_vs_lqr.png")


if __name__ == "__main__":
    main()