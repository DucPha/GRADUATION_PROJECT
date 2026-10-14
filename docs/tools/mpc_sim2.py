"""Round 2: tune the LQI integral q_i for robustness (mismatch, noise, curves)."""
import numpy as np
from mpc_sim import lk_mpc_solve, L, TAU, DT, N

g3 = np.deg2rad(3.0)
MAX_STEER = np.deg2rad(30.0)


def run(q_i, *, b_real=0.0, b_est=0.0, g_real=0.70, g_est=0.70, deadband=0.0,
        v=0.40, T=40.0, ey0=0.03, i_max=0.05, integ_band=0.15, noise=0.0,
        rho_fn=None, seed=0):
    n_steps = int(T / DT)
    a = min(max(DT / max(1e-3, TAU), 0.0), 1.0)
    rng = np.random.default_rng(seed)
    ey, epsi, dact, u_prev, s_int = ey0, 0.0, 0.0, 0.0, 0.0
    hist = []
    for _ in range(n_steps):
        rho = 0.0 if rho_fn is None else rho_fn(len(hist) * DT)
        ey_meas = ey + (rng.normal(0, noise) if noise else 0.0)
        epsi_meas = epsi + (rng.normal(0, noise * 8) if noise else 0.0)
        u = lk_mpc_solve(ey_meas, epsi_meas, dact, u_prev, v, [rho] * N,
                         g_est, b_est, q_i=q_i, i_max=i_max, s0=s_int)
        u_app = 0.0 if abs(u) < deadband else np.sign(u) * (abs(u) - deadband)
        dact = (1.0 - a) * dact + a * u_app
        epsi += max(0.05, v) * DT * ((g_real * dact + b_real) / L - rho)
        ey += max(0.05, v) * DT * epsi
        u_prev = u
        if q_i > 0.0 and abs(ey_meas) < integ_band:
            s_int = float(np.clip(s_int + ey_meas * DT, -i_max, i_max))
        hist.append((ey, epsi, dact, u, s_int))
    hist = np.array(hist)
    tail = hist[int(n_steps * 0.5):, 0]
    return float(np.mean(tail)) * 100, float(np.max(np.abs(tail))) * 100, hist[:, 0] * 100


def corner(t):
    return 0.0 if t < 12.0 else 0.6  # 1/m step (R~1.7 m)


print("=== S5b g mismatch 0.42 vs 0.70, b 3deg, sweep q_i ===")
for qi in (0, 40, 60, 80, 120, 160, 240):
    m, mx, _ = run(qi, b_real=g3, g_real=0.42, g_est=0.70)
    print(f"  q_i={qi:<4d} mean={m:+.3f} cm  max|e_y| tail={mx:.3f} cm")

print("=== S8 noisy detection (sigma_e=2mm), b 3deg, perfectly learned ===")
for qi in (0, 20, 40, 80):
    m, mx, _ = run(qi, b_real=g3, b_est=g3, noise=0.002)
    print(f"  q_i={qi:<4d} mean={m:+.3f} cm  max|e_y| tail={mx:.3f} cm")

print("=== S9 corner step rho:0->0.6 at t=12s, b_est=0, mismatch ===")
for qi in (0, 20, 40, 80):
    m, mx, h = run(qi, b_real=g3, g_est=0.70, rho_fn=corner, T=40)
    pre = h[int(10 / DT):int(12 / DT)]
    print(f"  q_i={qi:<4d} mean_tail={m:+.3f} cm  max|e_y| tail={mx:.3f} "
          f" straight(10-12s) mean={np.mean(pre):+.3f}")

print("=== S10 i_max sweep (g mismatch), q_i=80 ===")
for im in (0.02, 0.05, 0.10, 0.20):
    m, mx, _ = run(80, b_real=g3, g_real=0.42, g_est=0.70, i_max=im)
    print(f"  i_max={im:<5.2f} mean={m:+.3f} cm  max|e_y| tail={mx:.3f} cm")

print("=== S11 recovery from big offset 15cm, b 3deg, b_est=0 ===")
for qi in (0, 40, 80):
    m, mx, h = run(qi, b_real=g3, ey0=0.15, T=30)
    t_conv = next((i * DT for i in range(len(h)) if abs(h[i]) < 1.0), None)
    print(f"  q_i={qi:<4d} mean_tail={m:+.3f} cm  time_to_<1cm={t_conv}")
