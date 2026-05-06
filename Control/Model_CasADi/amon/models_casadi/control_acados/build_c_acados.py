from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi
from models_casadi.builder.build_model import build_model
import numpy as np
import os

from models_casadi.control_acados.acados_model import export_model
from models_casadi.control_acados.acados_ocp import build_ocp

print("ACADOS_SOURCE_DIR =", os.environ.get("ACADOS_SOURCE_DIR"))
#export ACADOS_SOURCE_DIR=/home/tinta/Amon_lander/Model_CasADi/amon/acados


####################################################################
# Gradnja acados modela in solverja v C (za NMPC)
####################################################################

# Parametri
params = AmonParamsCasadi(AmonParams())
x_dim = 23
u_dim = 5
N = 10
dt = 0.01

# CasADi model
f = build_model(params, model_type="instant") # "instant", "1st_order", "2nd_order"

# acados model - akados rabi continuous time model in desktritiziral ga bo sam
model = export_model(f, x_dim, u_dim) # funkcija, dimenzija stanja, dimenzija ukazov

# acados solver
output_dir = os.path.join(os.path.dirname(__file__), "c_generated_code")
os.makedirs(output_dir, exist_ok=True)

solver = build_ocp(model, x_dim, u_dim, N=N, dt=dt, code_export_dir=output_dir)

####################################################################
# Začetno stanje
####################################################################
x0 = np.array([0]*x_dim, dtype=float) # list nima fiksnega memory layouta, nima dtype in ni contiguous v pomnilniku
x0[6] = 1.0

for i in range(N + 1):
    solver.set(i, "x", x0)  

for i in range(N):
    solver.set(i, "u", np.zeros(u_dim, dtype=float))

####################################################################
# Referenčne vrednosti (yref) - prilagodi glede na build_ocp cost
####################################################################

# yref za vmesne korake (0..N-1): tipično [x_ref, u_ref]
y_dim = x_dim + u_dim  # prilagodi če imaš drugačen cost
yref = np.zeros(y_dim, dtype=float)
yref[6] = 1.0  # kvaternion w=1 tudi v referenci

for i in range(N):
    solver.set(i, "yref", yref)

# yref za terminalni korak (N): samo stanje, brez u
yref_N = np.zeros(x_dim, dtype=float)
yref_N[6] = 1.0

solver.set(N, "yref", yref_N)

####################################################################
# Ena iteracija / solve
####################################################################
status = solver.solve()

if status == 0:
    print("Solver uspešen.")
else:
    print(f"Solver status: {status} (0=OK, 1=MAX_ITER, 2=MIN_STEP, 3=QP_FAIL, 4=INFEASIBLE)")

u0 = solver.get(0, "u")
x1 = solver.get(1, "x")

print("u0 =", u0)
print("x1 =", x1)