from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi
from models_casadi.builder.build_model import build_model
import numpy as np
import os
from scipy.integrate import solve_ivp

from models_casadi.control_acados.acados_model import export_model
from models_casadi.control_acados.acados_ocp import build_ocp

####################################################################
# Setup
####################################################################

params = AmonParamsCasadi(AmonParams())
x_dim = 23
u_dim = 5
N = 10
dt = 0.01
T_sim = 15.0  # skupni čas simulacije [s]
n_steps = int(T_sim / dt)

f = build_model(params, model_type="instant")
model = export_model(f, x_dim, u_dim)
solver = build_ocp(model, x_dim, u_dim, N=N, dt=dt)

# Numpy model za simulacijo (ista f, ampak numpy verzija)
params_np = AmonParams()
f_np = build_model(params_np, model_type="instant")  # numpy verzija

####################################################################
# Začetno stanje
####################################################################

x0 = np.zeros(x_dim, dtype=float)
x0[6] = 1.0  # kvaternion w=1

yref = np.zeros(x_dim + u_dim, dtype=float)
yref[6] = 1.0  # referenca: lebdenje v izhodišču

yref_N = np.zeros(x_dim, dtype=float)
yref_N[6] = 1.0

####################################################################
# Simulacijska zanka
####################################################################

x = x0.copy()
log_x = [x.copy()]
log_u = []
log_t = [0.0]

for step in range(n_steps):
    t = step * dt

    # --- 1. Nastavi začetno stanje in reference ---
    for i in range(N + 1):
        solver.set(i, "x", x)
    for i in range(N):
        solver.set(i, "u", np.zeros(u_dim))
        solver.set(i, "yref", yref)
    solver.set(N, "yref", yref_N)

    # --- 2. Reši OCP (ena SQP_RTI iteracija) ---
    status = solver.solve()
    if status not in (0, 2):  # 2 = max iter (še vedno uporaben u)
        print(f"  Opozorilo: solver status {status} pri t={t:.3f}s")

    u = solver.get(0, "u")

    # --- 3. Simuliraj fiziko z RK45 ---
    def ode(t_, x_):
        return np.array(f_np(x_, u)).flatten()  # prilagodi klic glede na tvoj numpy model

    sol = solve_ivp(ode, [0, dt], x, method="RK45", max_step=dt/4)
    x = sol.y[:, -1]

    # --- 4. Shrani ---
    log_x.append(x.copy())
    log_u.append(u.copy())
    log_t.append(t + dt)

    if step % 100 == 0:
        print(f"t={t:.2f}s | pos=({x[0]:.2f},{x[1]:.2f},{x[2]:.2f}) | u={np.round(u,2)}")

log_x = np.array(log_x)
log_u = np.array(log_u)
log_t = np.array(log_t)

####################################################################
# Graf
####################################################################

import matplotlib.pyplot as plt

fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axs[0].plot(log_t, log_x[:, 0], label="x")
axs[0].plot(log_t, log_x[:, 1], label="y")
axs[0].plot(log_t, log_x[:, 2], label="z")
axs[0].set_ylabel("Pozicija [m]")
axs[0].legend(); axs[0].grid()

axs[1].plot(log_t, log_x[:, 3], label="vx")
axs[1].plot(log_t, log_x[:, 4], label="vy")
axs[1].plot(log_t, log_x[:, 5], label="vz")
axs[1].set_ylabel("Hitrost [m/s]")
axs[1].legend(); axs[1].grid()

axs[2].plot(log_t[:-1], log_u[:, 0], label="EDF")
axs[2].plot(log_t[:-1], log_u[:, 1:], label=[f"servo{i}" for i in range(1,5)])
axs[2].set_ylabel("Ukazi")
axs[2].set_xlabel("Čas [s]")
axs[2].legend(); axs[2].grid()

plt.tight_layout()
output_dir = os.path.join(os.path.dirname(__file__), "output")
os.makedirs(output_dir, exist_ok=True)  # creates folder if it doesn't exist
plt.savefig(os.path.join(output_dir, "acados_sim.png"), dpi=150, bbox_inches="tight")
plt.close()