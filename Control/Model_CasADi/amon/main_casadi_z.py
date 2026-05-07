import time

import numpy as np

from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi, NX, NU
from models_casadi.builder.build_model import build_model
from models_casadi.builder.build_discrete_model import build_discrete_model
from models_casadi.control.nmpc import build_nmpc


### -------------------------- NMPC ---------------------------- ###

# ------------------------------------------------------------
# PARAMETRI - klic iz numpy verzije in predelava v CasADi DM
# ------------------------------------------------------------
params_np = AmonParams()
params = AmonParamsCasadi(params_np)

# ------------------------------------------------------------
# CONTINUOUS MODEL - x_dot = f(x,u)
# ------------------------------------------------------------
f = build_model(params, model_type="instant")  # "instant", "1st_order", "2nd_order"

# ------------------------------------------------------------
# DISCRETE MODEL - x_{k+1} = F(x_k, u_k)
# ------------------------------------------------------------
dt = 0.02
F = build_discrete_model(f, NX, NU, dt)

# ------------------------------------------------------------
# Algoritem vodenja: NMPC
# ------------------------------------------------------------
N = 10
opti, X, U, X0, X_ref, U_prev = build_nmpc(F, NX, NU, N, dt)

# --------------------------------------------------------
# INITIAL STATE
# --------------------------------------------------------
x = np.zeros(NX)
x[6] = 1.0

# --------------------------------------------------------
# TARGET (hover)
# --------------------------------------------------------
xref = np.zeros(NX)
xref[2] = 1.0  # z = 1 m
xref[6] = 1.0  # enotski quaternion, brez rotacije

# --------------------------------------------------------
# ZAČETNI PRIBLIŽKI
# --------------------------------------------------------
u_hover = 87.0 # Približen % edf-ja za let
u_prev = np.array([u_hover, 0.0, 0.0, 0.0, 0.0])

x_init = np.zeros((NX, N + 1))
u_init = np.tile(u_prev.reshape(NU, 1), (1, N))
x_rollout = x.copy()
x_init[:, 0] = x_rollout
for k in range(N):
    x_rollout = np.array(F(x_rollout, u_prev).full()).reshape(NX)
    x_init[:, k + 1] = x_rollout

opti.set_initial(X, x_init)
opti.set_initial(U, u_init)

# --------------------------------------------------------
# CLOSED-LOOP SIMULATION
# --------------------------------------------------------
sim_steps = 2000
log = []

for i in range(sim_steps):
    opti.set_value(X0, x)
    opti.set_value(X_ref, xref)
    opti.set_value(U_prev, u_prev)

    try:
        t0 = time.time()
        sol = opti.solve_limited() # solver modela
        solve_time = time.time() - t0
    except Exception as e:
        print(f"Solver failed at i={i}: {e}")
        break

    # parsanje podatkov iz solverja
    x_sol = sol.value(X)
    u_sol = sol.value(U)
    u = u_sol[:, 0]     # porabiš samo prvi control input iz trajektorije

    # Simulacija sistema z dobljenimi ukazi
    x_next = F(x, u)
    x = np.array(x_next.full()).reshape(NX)
    u_prev = u

    # Warm-start: imaš že skoraj pravo rešitev od prej -> samo popraviš to za naslednji časovni korak
    x_guess = np.hstack([x_sol[:, 1:], x_sol[:, -1:]])
    x_guess[:, 0] = x
    u_guess = np.hstack([u_sol[:, 1:], u_sol[:, -1:]])
    opti.set_initial(X, x_guess)
    opti.set_initial(U, u_guess)

    log.append((i, x[2], x[5], u[0], solve_time))

    if i % 10 == 0 or i == sim_steps - 1:
        print(f"i={i:03d}  z={x[2]: .3f} m  vz={x[5]: .3f} m/s  edf={u[0]: .2f}%  solve={solve_time:.3f}s")

if log:
    last = log[-1]
    print(f"Final: i={last[0]}  z={last[1]:.3f} m  vz={last[2]:.3f} m/s  edf={last[3]:.2f}%")
