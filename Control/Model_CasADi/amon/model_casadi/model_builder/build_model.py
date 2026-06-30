from casadi import *
from amon.model_casadi.components.full_dynamics import full_dynamics_2nd_order, full_dynamics_instant, full_dynamics_1st_order
from model_casadi.parameters import NU, state_dimension


def build_model(params, model_type="instant"):
    """
    Ustvari zvezni model (CasADi ODE model):
    x_dot = f(x, u)
    """

    nx = state_dimension(model_type)

    x = MX.sym("x", nx)
    u = MX.sym("u", NU)

    if (model_type == "instant"):
        x_dot = full_dynamics_instant(x, u, params)
    if (model_type == "1st_order"):
        x_dot = full_dynamics_1st_order(x, u, params)
    if (model_type == "2nd_order"):
        x_dot = full_dynamics_2nd_order(x, u, params)

    f = Function("f", [x, u], [x_dot])

    return f
