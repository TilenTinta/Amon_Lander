import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

AMON_ROOT = Path(__file__).resolve().parents[2]
if str(AMON_ROOT) not in sys.path:
    sys.path.insert(0, str(AMON_ROOT))
ACADOS_TEMPLATE_ROOT = AMON_ROOT / "acados" / "interfaces" / "acados_template"
if str(ACADOS_TEMPLATE_ROOT) not in sys.path:
    sys.path.insert(0, str(ACADOS_TEMPLATE_ROOT))

from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi
from models_casadi.builder.build_model import build_model
from models_casadi.control_acados.acados_model import export_model
from models_casadi.control_acados.acados_ocp import (
    build_ocp,
    hover_input_ref,
    hover_yrefs,
)


####################################################################
# Setup
####################################################################

params = AmonParamsCasadi(AmonParams())
x_dim = 23
u_dim = 5
N = 10
dt = 0.02
T_sim = 10.0
n_steps = int(T_sim / dt)

f = build_model(params, model_type="instant")
model = export_model(f, x_dim, u_dim)
solver = build_ocp(model, x_dim, u_dim, N=N, dt=dt, build=True)

####################################################################
# Zacetno stanje
####################################################################

x = np.zeros(x_dim, dtype=float)
x[6] = 1.0

u_prev = hover_input_ref(u_dim)

for stage in range(N + 1):
    solver.set(stage, "x", x)
for stage in range(N):
    solver.set(stage, "u", u_prev)

####################################################################
# Simulacijska zanka
####################################################################

log_x = [x.copy()]
log_u = []
log_t = [0.0]

for step in range(n_steps):
    t = step * dt

    solver.set(0, "lbx", x)
    solver.set(0, "ubx", x)

    yref, yref_N = hover_yrefs(x_dim, u_dim, z_ref=1.0, u_prev=u_prev)
    for stage in range(N):
        solver.set(stage, "yref", yref)
    solver.set(N, "yref", yref_N)

    status = solver.solve()
    if status != 0:
        print(f"Opozorilo: solver status {status} pri t={t:.3f}s")

    u = solver.get(0, "u")

    def ode(_, x_):
        return np.array(f(x_, u)).reshape(-1)

    sol = solve_ivp(ode, [0, dt], x, method="RK45", max_step=dt / 4)
    x = sol.y[:, -1]
    u_prev = u

    # Warm-start za naslednji NMPC korak.
    for stage in range(N):
        solver.set(stage, "x", solver.get(stage + 1, "x"))
    solver.set(N, "x", solver.get(N, "x"))
    for stage in range(N - 1):
        solver.set(stage, "u", solver.get(stage + 1, "u"))
    solver.set(N - 1, "u", u_prev)

    log_x.append(x.copy())
    log_u.append(u.copy())
    log_t.append(t + dt)

    if step % 50 == 0 or step == n_steps - 1:
        print(
            f"t={t:.2f}s | z={x[2]:.3f} m | vz={x[5]:.3f} m/s | "
            f"edf={u[0]:.2f}%"
        )

log_x = np.array(log_x)
log_u = np.array(log_u)
log_t = np.array(log_t)

####################################################################
# Graf
####################################################################

fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

axs[0].plot(log_t, log_x[:, 0], label="x")
axs[0].plot(log_t, log_x[:, 1], label="y")
axs[0].plot(log_t, log_x[:, 2], label="z")
axs[0].set_ylabel("Pozicija [m]")
axs[0].legend()
axs[0].grid()

axs[1].plot(log_t, log_x[:, 3], label="vx")
axs[1].plot(log_t, log_x[:, 4], label="vy")
axs[1].plot(log_t, log_x[:, 5], label="vz")
axs[1].set_ylabel("Hitrost [m/s]")
axs[1].legend()
axs[1].grid()

axs[2].plot(log_t[:-1], log_u[:, 0], label="EDF")
axs[2].plot(log_t[:-1], log_u[:, 1:], label=[f"servo{i}" for i in range(1, 5)])
axs[2].set_ylabel("Ukazi")
axs[2].set_xlabel("Cas [s]")
axs[2].legend()
axs[2].grid()

plt.tight_layout()
output_dir = os.path.join(os.path.dirname(__file__), "output")
os.makedirs(output_dir, exist_ok=True)
plt.savefig(os.path.join(output_dir, "acados_sim.png"), dpi=150, bbox_inches="tight")
plt.close()
