from models_numpy.parameters import AmonParams
from models_casadi.parameters import AmonParamsCasadi
from models_casadi.builder.build_model import build_model
import numpy as np

from models_casadi.control_acados.acados_model import export_model
from models_casadi.control_acados.acados_ocp import build_ocp

# params
params = AmonParamsCasadi(AmonParams())

# CasADi model
f = build_model(params)

# acados model
model = export_model(f, 23, 5)

# acados solver
solver = build_ocp(model, 23, 5, N=10, dt=0.01)

# initial state
#x0 = [0]*23
x0 = np.array([0]*23, dtype=float)
x0[6] = 1.0

solver.set(0, "x", x0)

# solve
solver.solve()

u0 = solver.get(0, "u")
print("u0 =", u0)