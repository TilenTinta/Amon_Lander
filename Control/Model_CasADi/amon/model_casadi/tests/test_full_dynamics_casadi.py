from casadi import MX, Function
from model_numpy.parameters import AmonParams
from model_casadi.parameters import AmonParamsCasadi
from model_casadi.components.full_dynamics import full_dynamics_2nd_order

params = AmonParamsCasadi(AmonParams())

nx = 23
nu = 5

x = MX.sym("x", nx)
u = MX.sym("u", nu)

x_dot = full_dynamics_2nd_order(x, u, params)

f = Function("f", [x, u], [x_dot]) # Funkcija za NMPC

print(f([0]*nx, [0]*nu))
