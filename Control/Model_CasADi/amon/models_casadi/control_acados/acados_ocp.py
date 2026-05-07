import os

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver


U_HOVER = 87.0


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

    # LINEAR_LS stage cost vidi samo trenutni x,u. Prejsnji ukaz zato lahko
    # uporabimo samo kot runtime premik reference za prvi ukaz, ne kot poln
    # U[k] - U[k-1] cost brez razsirjenega modela.
    if u_prev is not None:
        uref = 0.6 * uref + 0.4 * np.asarray(u_prev, dtype=float)

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
    scale_x = np.array([
        10, 10, 5,        # position
        5, 5, 5,          # velocity
        1, 1, 1, 1,       # quaternion
        10, 10, 10,       # omega
        100,              # thrust state
        50,               # thrust_dot
        45, 45, 45, 45,   # servo states
        100, 100, 100, 100
    ], dtype=float)

    scale_u = np.array([20, 20, 20, 20, 20], dtype=float)

    Q_raw = np.array([
        2, 2, 260,          # p = [x, y, z]
        4, 4, 120,          # v = [vx, vy, vz]
        5, 40, 40, 10,      # q = [qw, qx, qy, qz]
        8, 8, 2,            # omega = [wx, wy, wz]
        0.0,                # T
        0.0,                # T_dot
        0.2, 0.2, 0.2, 0.2, # delta
        0.1, 0.1, 0.1, 0.1  # delta_dot
    ], dtype=float)

    R_raw = np.array([
        0.12, 
        0.25, 
        0.25, 
        0.25, 
        0.25
    ], dtype=float)

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
    ocp.constraints.lbu = np.array([0, -45, -45, -45, -45], dtype=float)
    ocp.constraints.ubu = np.array([100, 45, 45, 45, 45], dtype=float)
    ocp.constraints.idxbu = np.arange(nu)

    x0 = np.zeros(nx, dtype=float)
    x0[6] = 1.0
    ocp.constraints.x0 = x0

    # --------------------------------------------------------
    # STM32 / embedded settings
    # --------------------------------------------------------
    ocp.solver_options.with_solution_sens_wrt_params = False
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 2
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.nlp_solver_max_iter = 5
    ocp.solver_options.ext_fun_compile_flags = "-O2"
    ocp.solver_options.print_level = 0
    ocp.solver_options.qp_solver_iter_max = 20

    if code_export_dir is None:
        code_export_dir = os.path.join(os.path.dirname(__file__), "c_generated_code")
    ocp.code_export_directory = code_export_dir

    return AcadosOcpSolver(
        ocp,
        json_file=os.path.join(code_export_dir, "acados_ocp.json"),
        generate=generate,
        build=build
    )
