from casadi import MX, Function
from models_numpy.parameters import AmonParams
from models_casadi.components.actuators import edf_block_2nd_order

params = AmonParams()

T = MX.sym("T")
T_dot = MX.sym("T_dot")
cmd = MX.sym("cmd")

dT, dT_dot, T_cmd = edf_block_2nd_order(T, T_dot, cmd, params)

f = Function("edf2", [T, T_dot, cmd], [dT, dT_dot, T_cmd])

print(f(0, 0, 50))