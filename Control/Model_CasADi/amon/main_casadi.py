from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi, NX, NU
from models_casadi.builder.build_model import build_model
from models_casadi.builder.build_discrete_model import build_discrete_model
from models_casadi.control.nmpc import build_nmpc
import time 

### -------------------------- NMPC ---------------------------- ###

# ------------------------------------------------------------
# PARAMETRI - klic iz numpy verzije in predelava v CasADi DM
# ------------------------------------------------------------
params_np = AmonParams()
params = AmonParamsCasadi(params_np)

# ------------------------------------------------------------
# CONTINUOUS MODEL - x_dot = f(x,u) (diferencialna enačba sistema)
# ------------------------------------------------------------
f = build_model(params, model_type="instant") # "instant", "1st_order", "2nd_order"

# ------------------------------------------------------------
# DISCRETE MODEL - x_{k+1} = F(x_k, u_k) (za NMPC)
# ------------------------------------------------------------
dt = 0.02   # 0.01

F = build_discrete_model(f, NX, NU, dt)

# # ------------------------------------------------------------
# # TEST
# # ------------------------------------------------------------
# x0 = [0]*NX
# x0[6] = 1.0   # qw = 1 -> brez rotacije
# u0 = [0]*NU

# x_next = F(x0, u0)

# print("x_next =", x_next)




# ------------------------------------------------------------
# Algoritem vodenja: NMPC
# ------------------------------------------------------------
# horizon
N = 10

# RECEDING HORIZON CONTROL
# 1. poglej trenutno stanje
# 2. optimiziraj prihodnost (npr. 20 korakov naprej, N=20)
# 3. uporabiš SAMO prvi ukaz iz sekvence
# 4. ponoviš vse od začetka

opti, X, U, X0, X_ref = build_nmpc(F, NX, NU, N, dt)

# --------------------------------------------------------
# INITIAL STATE
# --------------------------------------------------------
# Dron potlen brez rotacije (na ravnini)
x0 = [0]*NX
x0[6] = 1.0  

# --------------------------------------------------------
# TARGET (hover)
# --------------------------------------------------------
xref = [0]*NX
xref[2] = 1.0 # pozicija (z = 1m)
xref[6] = 1.0 # quaternion (ohrani pokončno pozicijo)

# --------------------------------------------------------
# SET PARAMETERS
# --------------------------------------------------------
opti.set_value(X0, x0)
opti.set_value(X_ref, xref)

# --------------------------------------------------------
# SOLVE
# --------------------------------------------------------

opti.set_initial(X, 0)
opti.set_initial(U, 0)
opti.set_initial(X[6, :], 1.0)  # kvaternion = 1
opti.set_initial(U[0, :], 95)   # približen thrust za lebdenje

opti.solver("ipopt", {
    "ipopt.hessian_approximation": "limited-memory",
    "ipopt.max_iter": 200,
    "ipopt.tol": 1e-3,
    "ipopt.acceptable_tol": 1e-2,
    "ipopt.print_level": 0,
    "print_time": False
})

x = x0 # uporaba v iteraciji
for i in range(1000):

    opti.set_value(X0, x)
    opti.set_value(X_ref, xref)

    try:
        t0 = time.time()
        sol = opti.solve() # optimizacija
        t1 = time.time()
        print(f"Iteration {i}: Solver time = {t1 - t0:.3f} seconds")
    except Exception as e:
        print(f"Solver failed at i={i}: {e}")
        break

    u = sol.value(U[:,0]) # vzamem samo prvi ukaz iz optimizirane sekvence

    x = F(x, u)   # simulacija

    print("### Height:", x[2])
    print("### Thrust command:", u[0])