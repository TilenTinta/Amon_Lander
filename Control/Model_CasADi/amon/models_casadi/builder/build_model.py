from casadi import *
from models_casadi.dynamics.full_dynamics import full_dynamics_2nd_order, full_dynamics_instant, full_dynamics_1st_order
from models_casadi.parameters import NX, NU


def build_model(params, model_type="instant"):
    """
    CasADi ODE model
    """

    x = MX.sym("x", NX)
    u = MX.sym("u", NU)

    if (model_type == "instant"):
        x_dot = full_dynamics_instant(x, u, params)
    if (model_type == "1st_order"):
        x_dot = full_dynamics_1st_order(x, u, params)
    if (model_type == "2nd_order"):
        x_dot = full_dynamics_2nd_order(x, u, params)

    f = Function("f", [x, u], [x_dot])

    return f