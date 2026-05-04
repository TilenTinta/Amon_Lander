from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np

# OCP (Optimal Control Problem)
def build_ocp(model, nx, nu, N, dt):

    ocp = AcadosOcp() # definicija Optimal Control Problem-a

    ocp.model = model

    # horizon
    ocp.solver_options.N_horizon = N

    # časovni horizont
    ocp.solver_options.tf = N * dt

    # --------------------------------------------------------
    # COST
    # --------------------------------------------------------
    # Kaznovanje napake stanj
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

    # Kaznovanje napake ukazov
    R = np.diag([        
        0.1,   # EDF → thrust
        0.1,   # servo 1
        0.1,   # servo 2
        0.1,   # servo 3
        0.1    # servo 4
    ])

    # Referenca  
    ny = nx + nu
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # cost type - least squares cost / kvadrat napake (cost = (y - yref)^T W (y - yref))
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # mapping (y = Vx * x + Vu * u), cost deluje na state + control
    ocp.cost.Vx = np.vstack([
        np.eye(nx),
        np.zeros((nu, nx))
    ])

    ocp.cost.Vu = np.vstack([
        np.zeros((nx, nu)),
        np.eye(nu)
    ])

    # zadnji korak / konec horizonta kaznuješ samo stanje, ne ukaze
    ocp.cost.Vx_e = np.eye(nx) 

    # Kazen napake stanj in ukazov (matrično)
    ocp.cost.W = np.block([
        [Q, np.zeros((nx, nu))],
        [np.zeros((nu, nx)), R]
    ])

    # zadnji korak / konec horizonta kaznuješ samo stanje (aktuatorji so važni le na poti)
    ocp.cost.W_e = Q

    # --------------------------------------------------------
    # CONSTRAINTS - omejitve aktuatorjev
    # --------------------------------------------------------
    ocp.constraints.lbu = np.array([0, -45, -45, -45, -45])
    ocp.constraints.ubu = np.array([100, 45, 45, 45, 45])
    ocp.constraints.idxbu = np.arange(nu)

    # --------------------------------------------------------
    # SOLVER OPTIONS
    # --------------------------------------------------------
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"   # uporabljen solver (HPIPM)
    ocp.solver_options.integrator_type = "ERK"                  # Uporabljen integrator (explicit Runge-Kutta)
    ocp.solver_options.nlp_solver_type = "SQP_RTI"              # Uporabljen solver (SQP_RTI - Real-Time Iteration)

    return AcadosOcpSolver(ocp, json_file="acados_ocp.json")