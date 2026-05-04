from acados_template import AcadosModel
from casadi import MX

def export_model(f, nx, nu):

    model = AcadosModel()

    x = MX.sym("x", nx)
    u = MX.sym("u", nu)

    xdot = f(x, u)

    model.x = x
    model.u = u
    model.f_expl_expr = xdot    # f_expl_expr = explicit dynamics expression
    model.name = "amon_model"

    return model