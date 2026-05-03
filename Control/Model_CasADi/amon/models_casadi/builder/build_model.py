from casadi import *
from models_casadi.dynamics.full_dynamics import full_dynamics
from models_casadi.parameters import NX, NU


def build_model(params):
    """
    CasADi ODE model
    """

    x = MX.sym("x", NX)
    u = MX.sym("u", NU)

    x_dot = full_dynamics(x, u, params)

    f = Function("f", [x, u], [x_dot])

    return f