"""Generate every figure used in docs/LESSONS_LEARNED.md.

Run from the project root:  python docs/make_figures.py
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.motor_model import MotorParams, dynamics, state_space  # noqa: E402
from src.pid import PID  # noqa: E402
from src.lqr import LQR, augment_with_integrator, lqr_gain  # noqa: E402
from sim_pid import rk4_step  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
os.makedirs(OUT, exist_ok=True)
DT = 0.001
p = MotorParams()
A, B, C = state_space(p)
A_aug, B_aug = augment_with_integrator(A, B, C)


def save(fig, name):
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, name), dpi=130)
    plt.close(fig)
    print("saved", name)


def euler_step(x, u, dt):
    return x + dt * dynamics(x, u, p)


# 1. Motor circuit ---------------------------------------------------------
def fig_circuit():
    try:
        import schemdraw
        import schemdraw.elements as elm
    except ImportError:
        print("skipping circuit (pip install schemdraw)")
        return
    with schemdraw.Drawing(show=False) as d:
        d.config(fontsize=13)
        d += elm.SourceV().up().label("V\n(input)", loc="left")
        d += elm.Resistor().right().label("R = 1 Ω").label("i  →", loc="bottom")
        d += elm.Inductor2().right().label("L = 0.5 H")
        d += elm.Motor().down().label("back-EMF\n$K_e \\omega$", loc="bottom")
        d += elm.Line().left().tox(0)
        d.save(os.path.join(OUT, "fig1_motor_circuit.png"), dpi=130)
    print("saved fig1_motor_circuit.png")


# 2. Open-loop response ----------------------------------------------------
def fig_open_loop():
    t = np.arange(0, 3, DT)
    x = np.zeros(2)
    w, i = [], []
    for _ in t:
        x = rk4_step(x, 10.0, p, DT)
        w.append(x[0])
        i.append(x[1])
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(8, 5.5))
    a1.plot(t, w)
    a1.axhline(1.0, ls="--", c="gray", label="1 rad/s")
    a1.set_ylabel("Speed ω (rad/s)")
    a1.set_title("Open loop: apply a constant 10 V and wait")
    a1.legend()
    a1.grid(True)
    a2.plot(t, i, c="tab:red")
    a2.set_ylabel("Current i (A)")
    a2.set_xlabel("Time (s)")
    a2.grid(True)
    save(fig, "fig2_open_loop.png")


# 3. Euler vs RK4 ----------------------------------------------------------
def fig_integrators():
    t_true = np.arange(0, 3, DT)
    x = np.zeros(2)
    w_true = []
    for _ in t_true:
        x = rk4_step(x, 10.0, p, DT)
        w_true.append(x[0])
    big = 0.19
    t_big = np.arange(0, 3 + big, big)
    xe, xr = np.zeros(2), np.zeros(2)
    we, wr = [0.0], [0.0]
    for _ in t_big[1:]:
        xe = euler_step(xe, 10.0, big)
        xr = rk4_step(xr, 10.0, p, big)
        we.append(xe[0])
        wr.append(xr[0])
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t_true, w_true, c="k", lw=2, label="true answer (tiny steps)")
    ax.plot(t_big, we, "o-", label=f"Euler, dt = {big} s")
    ax.plot(t_big, wr, "s-", label=f"RK4, dt = {big} s")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed ω (rad/s)")
    ax.set_title("Same big step size, two integrators")
    ax.legend()
    ax.grid(True)
    save(fig, "fig3_euler_vs_rk4.png")


# 4. PID term breakdown ----------------------------------------------------
def fig_pid_terms():
    pid = PID(100, 200, 10, DT, -p.V_max, p.V_max)
    t = np.arange(0, 1.5, DT)
    x = np.zeros(2)
    P, I, D, U = [], [], [], []
    for _ in t:
        prev = pid.prev_meas
        u = pid.update(1.0, x[0])
        e = 1.0 - x[0]
        d_meas = 0.0 if prev is None else (x[0] - prev) / DT
        P.append(pid.kp * e)
        I.append(pid.ki * pid.integral)
        D.append(-pid.kd * d_meas)
        U.append(u)
        x = rk4_step(x, u, p, DT)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, P, label="P term  (Kp·e)")
    ax.plot(t, I, label="I term  (Ki·∫e dt)")
    ax.plot(t, D, label="D term  (−Kd·dω/dt)")
    ax.plot(t, U, "k", lw=2, label="total voltage (after 24 V limit)")
    ax.axhline(24, ls=":", c="gray")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Volts")
    ax.set_title("What each PID term contributes")
    ax.legend()
    ax.grid(True)
    save(fig, "fig4_pid_terms.png")


# 5. Windup ----------------------------------------------------------------
def fig_windup():
    t = np.arange(0, 3, DT)

    def run(anti_windup):
        integ, prev = 0.0, None
        x = np.zeros(2)
        w = []
        for _ in t:
            e = 1.0 - x[0]
            d = 0.0 if prev is None else (x[0] - prev) / DT
            prev = x[0]
            u_un = 100 * e + 200 * (integ + e * DT) - 10 * d
            u = min(max(u_un, -24), 24)
            if (not anti_windup) or u == u_un:
                integ += e * DT
            x = rk4_step(x, u, p, DT)
            w.append(x[0])
        return np.array(w)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, run(False), label="no anti-windup")
    ax.plot(t, run(True), label="with anti-windup (our PID)")
    ax.axhline(1.0, ls="--", c="gray")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed ω (rad/s)")
    ax.set_title("Integrator windup: same gains, same 24 V limit")
    ax.legend()
    ax.grid(True)
    save(fig, "fig5_windup.png")


# 6. Scalar LQR ------------------------------------------------------------
def fig_scalar_lqr():
    a, b = -10.0, 1.0
    ratio = np.logspace(-1, 4.3, 200)
    pole = -np.sqrt(a ** 2 + b ** 2 * ratio)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.semilogx(ratio, pole)
    for r_, lab in [(300, "q/r = 300 → pole −20"), (9900, "q/r = 9900 → pole −100")]:
        ax.plot(r_, -np.sqrt(a ** 2 + r_), "o", c="tab:red")
        ax.annotate(lab, (r_, -np.sqrt(a ** 2 + r_)), textcoords="offset points",
                    xytext=(-170, -4))
    ax.axhline(a, ls=":", c="gray", label="open-loop pole a = −10")
    ax.set_xlabel("q / r   (how much we care about error vs. effort)")
    ax.set_ylabel("closed-loop pole")
    ax.set_title("1-state LQR: more q/r = faster (more negative) pole")
    ax.legend()
    ax.set_ylim(-130, 5)
    ax.grid(True)
    save(fig, "fig6_scalar_lqr.png")


# 7 & 8. LQR tuning sweep and pole map ------------------------------------
TUNINGS = [
    ([1, 0, 1], 1.0),
    ([100, 0, 5000], 0.01),
    ([10, 0, 5000], 0.01),
    ([100, 0, 10000], 0.01),
]


def run_lqr(K, T=3.0):
    ctrl = LQR(K, DT, -p.V_max, p.V_max)
    t = np.arange(0, T, DT)
    x = np.zeros(2)
    w = []
    for _ in t:
        u = ctrl.update(1.0, x)
        x = rk4_step(x, u, p, DT)
        w.append(x[0])
    return t, np.array(w)


def fig_sweep_and_poles():
    fig, ax = plt.subplots(figsize=(8, 4.5))
    fig2, ax2 = plt.subplots(figsize=(7, 6))
    ol = np.linalg.eigvals(A)
    ax2.plot(ol.real, ol.imag, "kx", ms=12, mew=2, label="open loop (motor alone)")
    markers = ["o", "s", "^", "D"]
    for (qd, r), m in zip(TUNINGS, markers):
        K = lqr_gain(A_aug, B_aug, np.diag(np.array(qd, float)), np.array([[r]]))
        lab = f"Q = diag{tuple(qd)}, R = {r}"
        t, w = run_lqr(K)
        ax.plot(t, w, label=lab)
        cl = np.linalg.eigvals(A_aug - B_aug @ K)
        ax2.plot(cl.real, cl.imag, m, ms=8, mfc="none", mew=2, label=lab)
    ax.axhline(1.0, ls="--", c="gray")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed ω (rad/s)")
    ax.set_title("Turning the LQR knobs")
    ax.legend(fontsize=8)
    ax.grid(True)
    save(fig, "fig7_lqr_tuning.png")
    ax2.axvline(0, c="k", lw=0.8)
    ax2.axhline(0, c="k", lw=0.8)
    ax2.axvspan(0, 3, color="tab:red", alpha=0.08)
    ax2.text(0.3, 8, "unstable\nside", color="tab:red")
    ax2.set_xlim(-14, 3)
    ax2.set_xlabel("Real part (how fast it decays)")
    ax2.set_ylabel("Imaginary part (how much it wiggles)")
    ax2.set_title("Where the poles end up")
    ax2.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=2)
    ax2.grid(True)
    save(fig2, "fig8_pole_map.png")


# 9. Chatter ---------------------------------------------------------------
def fig_chatter():
    K = lqr_gain(A_aug, B_aug, np.diag([100.0, 0, 10000]), np.array([[0.01]]))
    t = np.arange(0, 0.25, DT)

    x = np.zeros(2)
    ctrl = LQR(K, DT, -p.V_max, p.V_max)
    u_cond = []
    for _ in t:
        u = ctrl.update(1.0, x)
        u_cond.append(u)
        x = rk4_step(x, u, p, DT)

    x = np.zeros(2)
    xi = 0.0
    u_bc = []
    for _ in t:
        u_un = float(-K[0] @ np.array([x[0], x[1], xi]))
        u = min(max(u_un, -p.V_max), p.V_max)
        xi += ((1.0 - x[0]) + 1.0 * (u - u_un)) * DT
        u_bc.append(u)
        x = rk4_step(x, u, p, DT)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, u_cond, label="conditional integration (on/off switch)")
    ax.plot(t, u_bc, label="back-calculation (gradual)")
    ax.set_ylim(20, 24.6)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("Zoomed in: LQR voltage while at the 24 V limit")
    ax.legend()
    ax.grid(True)
    save(fig, "fig9_chatter.png")


if __name__ == "__main__":
    fig_circuit()
    fig_open_loop()
    fig_integrators()
    fig_pid_terms()
    fig_windup()
    fig_scalar_lqr()
    fig_sweep_and_poles()
    fig_chatter()
