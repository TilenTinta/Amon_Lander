from casadi import MX, Function
from models_numpy.parameters import AmonParams
from models_casadi.components.tvc import total_force_body, total_torque_body

params = AmonParams()

T = MX.sym("T")
delta = MX.sym("delta", 4)

F = total_force_body(T, delta, params)
tau = total_torque_body(T, delta, params)

f = Function("tvc", [T, delta], [F, tau])

print(f(10, [0, 0, 0, 0]))