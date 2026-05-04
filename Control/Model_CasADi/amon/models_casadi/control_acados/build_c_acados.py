from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi
from models_casadi.builder.build_model import build_model
import numpy as np

from models_casadi.control_acados.acados_model import export_model
from models_casadi.control_acados.acados_ocp import build_ocp


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
f = build_model(params)

# acados model - akados rabi continuous time model in desktritiziral ga bo sam
model = export_model(f, x_dim, u_dim) # funkcija, dimenzija stanja, dimenzija ukazov

# acados solver
solver = build_ocp(model, x_dim, u_dim, N=N, dt=dt)

# initial state
#x0 = [0]*23
x0 = np.array([0]*x_dim, dtype=float) # list nima fiksnega memory layouta, nima dtype in ni contiguous v pomnilniku
x0[6] = 1.0

solver.set(0, "x", x0)

# solve
solver.solve()

u0 = solver.get(0, "u")
print("u0 =", u0)