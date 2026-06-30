import os

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import MX

from model_casadi.control.nmpc_config import (
    U_HOVER,
    input_lower_array,
    input_upper_array,
    q_raw_array,
    r_raw_array,
    scale_u_array,
    scale_x_array,
)


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


def hover_state_ref(nx, z_ref=1.0):
    xref = np.zeros(nx, dtype=float)
    xref[2] = z_ref
    xref[6] = 1.0
    return xref


def hover_input_ref(nu):
    uref = np.zeros(nu, dtype=float)
    uref[0] = U_HOVER
    return uref


def hover_yrefs(nx, nu, z_ref=1.0, u_prev=None):
    xref = hover_state_ref(nx, z_ref)
    uref = hover_input_ref(nu)

    return np.concatenate([xref, uref]), xref


def build_ocp(model, nx, nu, N, dt, code_export_dir=None, generate=True, build=False):
    ocp = AcadosOcp()
    ocp.model = model

    # --------------------------------------------------------
    # HORIZON
    # --------------------------------------------------------
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt

    # --------------------------------------------------------
    # COST
    # --------------------------------------------------------
    scale_x = scale_x_array(nx)
    scale_u = scale_u_array()
    Q_raw = q_raw_array(nx)
    R_raw = r_raw_array()

    # Same tuning idea as the stable CasADi NMPC: penalize normalized errors.
    Q = np.diag(Q_raw / (scale_x * scale_x))
    R = np.diag(R_raw / (scale_u * scale_u))

    ny = nx + nu
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ocp.cost.Vx = np.vstack([
        np.eye(nx),
        np.zeros((nu, nx))
    ])
    ocp.cost.Vu = np.vstack([
        np.zeros((nx, nu)),
        np.eye(nu)
    ])
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])
    ocp.cost.W_e = 3.0 * Q

    ocp.cost.yref, ocp.cost.yref_e = hover_yrefs(nx, nu)
    assert ocp.cost.yref.shape == (ny,)

    # --------------------------------------------------------
    # CONSTRAINTS
    # --------------------------------------------------------
    ocp.constraints.lbu = input_lower_array()
    ocp.constraints.ubu = input_upper_array()
    ocp.constraints.idxbu = np.arange(nu)

    x0 = np.zeros(nx, dtype=float)
    x0[6] = 1.0
    ocp.constraints.x0 = x0

    # --------------------------------------------------------
    # STM32 / embedded settings
    # --------------------------------------------------------
    ocp.solver_options.with_solution_sens_wrt_params = False
    ocp.solver_options.with_value_sens_wrt_params = False
    ocp.solver_options.sim_method_num_stages = 1 #2
    ocp.solver_options.sim_method_num_steps = 1 #2
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = 1 # N
    ocp.solver_options.nlp_solver_max_iter = 3 #5,1
    ocp.solver_options.ext_fun_compile_flags = "-O3 -ffast-math -funroll-loops"
    ocp.solver_options.print_level = 0
    ocp.solver_options.qp_solver_iter_max = 3 #20 >> 10
    ocp.solver_options.acados_with_static_memory = 1     # disables malloc
    ocp.solver_options.hpipm_ref = 1  # Use single precision HPIPM
    ocp.solver_options.blasfeo_ref = 1  # Use single precision BLASFEO
    ocp.solver_options.tf = 1.0  # Keep timing realistic
    ocp.solver_options.qp_solver_warm_start = 1  # Enable warm-starting
    ocp.solver_options.sens_method = "forw"  # Use forward sensitivity only 
    #ocp.solver_options.sens_forw_hess = 0    # Disable Hessian in sensitivity (instead of sens_method = "forw")

    # ocp.solver_options.with_solution_sens_wrt_params = False
    # ocp.solver_options.sim_method_num_stages = 2 #4
    # ocp.solver_options.sim_method_num_steps = 1 #2
    # ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # ocp.solver_options.integrator_type = "ERK"
    # ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # ocp.solver_options.qp_solver_cond_N = N
    # ocp.solver_options.nlp_solver_max_iter = 1 #5
    # ocp.solver_options.ext_fun_compile_flags = "-O2"
    # ocp.solver_options.print_level = 0
    # ocp.solver_options.qp_solver_iter_max = 20
    # ocp.solver_options.acados_with_static_memory = 1     # disables malloc

    if code_export_dir is None:
        code_export_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "acados_generated", "c_generated_code")
        )
    ocp.code_export_directory = code_export_dir

    return AcadosOcpSolver(
        ocp,
        json_file=os.path.join(code_export_dir, "acados_ocp.json"),
        generate=generate,
        build=build
    )
