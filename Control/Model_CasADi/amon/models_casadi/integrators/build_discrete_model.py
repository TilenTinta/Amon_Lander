from casadi import *
from models_casadi.integrators.rk4 import rk4_step


def build_discrete_model(f, nx, nu, dt):
    """
    Ustvari diskretni model:
    x_{k+1} = F(x_k, u_k)
    """

    x = MX.sym("x", nx)
    u = MX.sym("u", nu)

    x_next = rk4_step(f, x, u, dt)

    return Function("F", [x, u], [x_next])