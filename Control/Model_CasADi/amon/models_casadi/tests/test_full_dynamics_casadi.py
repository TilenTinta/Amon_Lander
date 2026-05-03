from casadi import MX, Function
from models_numpy.parameters import AmonParams
from models_casadi.dynamics.full_dynamics import full_dynamics

params = AmonParams()

nx = 23
nu = 5

x = MX.sym("x", nx)
u = MX.sym("u", nu)

x_dot = full_dynamics(x, u, params)

f = Function("f", [x, u], [x_dot]) # Funkcija za NMPC

print(f([0]*nx, [0]*nu))