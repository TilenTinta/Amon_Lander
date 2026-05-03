from casadi import MX, Function
from models_numpy.parameters import AmonParams
from models_casadi.components.actuators import servo_block_2nd_order

params = AmonParams()

delta = MX.sym("delta")
delta_dot = MX.sym("delta_dot")
cmd = MX.sym("cmd")

dd, dd_dot, cmd_out = servo_block_2nd_order(delta, delta_dot, cmd, params)

f = Function("servo", [delta, delta_dot, cmd], [dd, dd_dot, cmd_out])

print(f(0, 0, 10))