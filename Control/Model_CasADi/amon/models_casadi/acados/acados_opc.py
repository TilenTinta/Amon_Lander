from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np

def build_ocp(model, nx, nu, N, dt):

    ocp = AcadosOcp()

    ocp.model = model

    # horizon
    ocp.dims.N = N

    # časovni horizont
    ocp.solver_options.tf = N * dt

    # --------------------------------------------------------
    # COST
    # --------------------------------------------------------
    Q = np.diag([
        10,10,50,
        1,1,5,
        10,10,10,10,
        1,1,1,
        0.1,
        0.01,
        1,1,1,1,
        0.1,0.1,0.1,0.1
    ])

    R = np.diag([0.1]*nu)

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])

    ocp.cost.W_e = Q

    # mapping
    ocp.cost.Vx = np.hstack([np.eye(nx), np.zeros((nx, nu))])
    ocp.cost.Vu = np.hstack([np.zeros((nu, nx)), np.eye(nu)])
    ocp.cost.Vx_e = np.eye(nx)

    # --------------------------------------------------------
    # CONSTRAINTS
    # --------------------------------------------------------
    ocp.constraints.lbu = np.array([0, -45, -45, -45, -45])
    ocp.constraints.ubu = np.array([100, 45, 45, 45, 45])
    ocp.constraints.idxbu = np.arange(nu)

    # --------------------------------------------------------
    # SOLVER OPTIONS
    # --------------------------------------------------------
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    return AcadosOcpSolver(ocp, json_file="acados_ocp.json")