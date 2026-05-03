from casadi import MX, Function
from models_numpy.parameters import AmonParams
from models_casadi.components.rigid_body import velocity_dot, omega_dot

params = AmonParams()

v = MX.sym("v", 3)
q = MX.sym("q", 4)
F = MX.sym("F", 3)
omega = MX.sym("omega", 3)
tau = MX.sym("tau", 3)

v_dot = velocity_dot(v, q, F, params)
w_dot = omega_dot(omega, tau, params)

f = Function("rigid", [v, q, F, omega, tau], [v_dot, w_dot])

print(f([0,0,0], [1,0,0,0], [0,0,10], [0,0,0], [0,0,0]))