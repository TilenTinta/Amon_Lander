from casadi import MX, Function
from model_numpy.parameters import AmonParams
from model_casadi.parameters import AmonParamsCasadi
from model_casadi.components.actuators import servo_block_2nd_order

params = AmonParamsCasadi(AmonParams())

delta = MX.sym("delta")
delta_dot = MX.sym("delta_dot")
cmd = MX.sym("cmd")

dd, dd_dot, cmd_out = servo_block_2nd_order(delta, delta_dot, cmd, params)

f = Function("servo", [delta, delta_dot, cmd], [dd, dd_dot, cmd_out])

print(f(0, 0, 10))
