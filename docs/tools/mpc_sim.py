"""Faithful port of lk_mpc_solve() + closed-loop plant sim.

Goal: check whether the current MPC leaves a steady lateral offset e_y under
model mismatch / servo deadband (the "xe lech dan" symptom) and whether adding
LQI integral action removes it, and how it must be tuned.
"""
import numpy as np

L = 0.26
TAU = 0.15
DT = 0.05
N = 24
MAX_STEER = np.deg2rad(30.0)


def lk_mpc_solve(ey, epsi, dact, u_prev, v, rho, g, b, *,
                 zone=0.05, qyf=30.0, qpf=1.0, qyn=8.0, qpn=3.0,
                 r=0.05, rd=8.0, q_i=0.0, i_max=0.05, s0=0.0):
    """Exact port of lane_keeping.hpp::lk_mpc_solve (+ q_i/s0 for the LQI test)."""
    n = N
    vdt = max(0.05, v) * DT
    a = min(max(DT / max(1e-3, TAU), 0.0), 1.0)
    s = min(max((abs(ey) - 0.5 * zone) / zone, 0.0), 1.0)
    qy = qyn + (qyf - qyn) * s
    qp = qpn + (qpf - qpn) * s

    A = np.array([[1.0, vdt, 0.0],
                  [0.0, 1.0, vdt * g / L],
                  [0.0, 0.0, 1.0 - a]])
    B = np.array([0.0, 0.0, a])
    rho = list(rho) + [rho[-1]] * (n - len(rho))
    free = np.zeros((n, 3))
    su = np.zeros((n, n, 3))
    x = np.array([ey, epsi, dact], float)
    prev_su = np.zeros((n, 3))
    for k in range(n):
        w = np.array([0.0, vdt * (b / L - rho[k]), 0.0])
        x = A @ x + w
        free[k] = x
        for j in range(n):
            t = A @ prev_su[j]
            if j == k:
                t = t + B
            su[k, j] = t
        prev_su = su[k]

    H = np.zeros((n, n))
    f = np.zeros(n)
    cum_free = 0.0
    cum_su = np.zeros(n)
    si = min(max(s0, -i_max), i_max)
    for k in range(n):
        yi = su[k, :, 0]
        pi = su[k, :, 1]
        f += qy * yi * free[k, 0] + qp * pi * free[k, 1]
        H += qy * np.outer(yi, yi) + qp * np.outer(pi, pi)
        cum_free += free[k, 0]
        cum_su += su[k, :, 0]
        if q_i > 0.0:
            gi = DT * cum_su
            cc = si + DT * cum_free
            f += q_i * gi * cc
            H += q_i * np.outer(gi, gi)
    for i in range(n):
        H[i, i] += r + 2.0 * rd
        if i + 1 < n:
            H[i, i + 1] -= rd
            H[i + 1, i] -= rd
    H[n - 1, n - 1] -= rd
    f[0] -= rd * u_prev
    try:
        u = np.linalg.solve(H, -f)
    except np.linalg.LinAlgError:
        u = np.linalg.lstsq(H, -f, rcond=None)[0]
    return float(np.clip(u[0], -MAX_STEER, MAX_STEER))


def run(q_i=0.0, i_max=0.05, b_real=0.0, b_est=0.0, g_real=0.70, g_est=0.70,
        deadband=0.0, v=0.40, rho=0.0, T=30.0, ey0=0.03, epsi0=0.0,
        integ_band=0.15):
    n_steps = int(T / DT)
    ey, epsi, dact, u_prev, s_int = ey0, epsi0, 0.0, 0.0, 0.0
    hist = []
    a = min(max(DT / max(1e-3, TAU), 0.0), 1.0)
    for _ in range(n_steps):
        u = lk_mpc_solve(ey, epsi, dact, u_prev, v, [rho] * N, g_est, b_est,
                         q_i=q_i, i_max=i_max, s0=s_int)
        # --- plant: servo (with deadband) then kinematics with TRUE params ---
        u_app = 0.0 if abs(u) < deadband else np.sign(u) * (abs(u) - deadband)
        dact = (1.0 - a) * dact + a * u_app
        epsi += max(0.05, v) * DT * ((g_real * dact + b_real) / L - rho)
        ey += max(0.05, v) * DT * epsi
        u_prev = u
        if q_i > 0.0 and abs(ey) < integ_band:
            s_int = float(np.clip(s_int + ey * DT, -i_max, i_max))
        hist.append((ey, epsi, dact, u, s_int))
    hist = np.array(hist)
    tail = hist[int(n_steps * 0.6):, 0]
    return hist, float(np.mean(tail)), float(np.max(np.abs(tail)))


if __name__ == "__main__":
    b3 = np.deg2rad(3.0)
    print("=== S1: perfect model, no bias ===")
    h, m, mx = run(q_i=0.0)
    print(f"  q_i=0    e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")
    for qi in (5, 10, 20, 40, 80):
        h, m, mx = run(q_i=qi)
        print(f"  q_i={qi:<3d} e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")

    print("=== S2: bias b_real=3deg, oracle NOT learned (b_est=0) ===")
    for qi in (0, 5, 10, 20, 40, 80):
        h, m, mx = run(q_i=qi, b_real=b3, b_est=0.0)
        print(f"  q_i={qi:<3d} e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")

    print("=== S3: bias 3deg, oracle learned correctly (b_est=b_real) ===")
    for qi in (0, 10, 20):
        h, m, mx = run(q_i=qi, b_real=b3, b_est=b3)
        print(f"  q_i={qi:<3d} e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")

    print("=== S4: bias 3deg + servo deadband 0.5deg, b_est=0 ===")
    db = np.deg2rad(0.5)
    for qi in (0, 10, 20, 40):
        h, m, mx = run(q_i=qi, b_real=b3, b_est=0.0, deadband=db)
        print(f"  q_i={qi:<3d} e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")

    print("=== S5: bias 3deg, g_real=0.42 g_est=0.70 (steer ratio wrong) ===")
    for qi in (0, 10, 20, 40):
        h, m, mx = run(q_i=qi, b_real=b3, b_est=0.0, g_real=0.42, g_est=0.70)
        print(f"  q_i={qi:<3d} e_y_tail mean={m*100:+.3f} cm  numpy_max={mx*100:.3f}")
