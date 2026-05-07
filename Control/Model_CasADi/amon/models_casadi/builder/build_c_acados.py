import os
import sys
from pathlib import Path

import numpy as np

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


print("ACADOS_SOURCE_DIR =", os.environ.get("ACADOS_SOURCE_DIR"))

####################################################################
# Gradnja acados modela in solverja v C (za NMPC)
####################################################################
params = AmonParamsCasadi(AmonParams())
x_dim = 23
u_dim = 5
N = 10
dt = 0.02

f = build_model(params, model_type="instant")
model = export_model(f, x_dim, u_dim)

output_dir = os.path.join(os.path.dirname(__file__), "c_generated_code")
os.makedirs(output_dir, exist_ok=True)

solver = build_ocp(model, x_dim, u_dim, N=N, dt=dt, code_export_dir=output_dir)

####################################################################
# Zacetno stanje in hover reference
####################################################################

x0 = np.zeros(x_dim, dtype=float)
x0[6] = 1.0

# To je isti runtime korak, ki ga bos na MCU naredil z meritvijo trenutnega x.
solver.set(0, "lbx", x0)
solver.set(0, "ubx", x0)

for stage in range(N + 1):
    solver.set(stage, "x", x0)

u_hover = hover_input_ref(u_dim)
for stage in range(N):
    solver.set(stage, "u", u_hover)

yref, yref_N = hover_yrefs(x_dim, u_dim, z_ref=1.0, u_prev=u_hover)
for stage in range(N):
    solver.set(stage, "yref", yref)
solver.set(N, "yref", yref_N)

####################################################################
# Ena iteracija / solve
####################################################################

status = solver.solve()

if status == 0:
    print("Solver uspesen.")
else:
    print(f"Solver status: {status} (0=OK)")

u0 = solver.get(0, "u")
x1 = solver.get(1, "x")

print("u0 =", u0)
print("x1 =", x1)
