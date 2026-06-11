from casadi import *

from model_casadi.control.nmpc_config import (
    INPUT_LOWER,
    INPUT_UPPER,
    Q_RAW,
    R_DELTA_RAW,
    R_RAW,
    SCALE_U,
    SCALE_X,
    U_HOVER,
)


def build_nmpc(F, nx, nu, N, dt):
    # --------------------------------------------------------
    # OPTIMIZER
    # --------------------------------------------------------
    opti = Opti()

    # --------------------------------------------------------
    # VARIABLES
    # --------------------------------------------------------
    X = opti.variable(nx, N + 1)  # states - cela prihodnja trajektorija
    U = opti.variable(nu, N)      # controls - cel control signal

    # --------------------------------------------------------
    # PARAMETERS (reference + initial state)
    # --------------------------------------------------------
    X_ref = opti.parameter(nx)   # ciljna referenca
    X0 = opti.parameter(nx)      # trenutno stanje
    U_prev = opti.parameter(nu)  # zadnji uporabljeni ukaz

    # --------------------------------------------------------
    # COST FUNCTION
    # --------------------------------------------------------
    scale_x = SCALE_X

    # Ukazi so v procentih/stopinjah. Skala 20 pomeni, da je 20 enot ze
    # velik odmik, zato optimizer ne skace po celotnem aktuatorskem obmocju.
    scale_u = SCALE_U

    Q = diag(Q_RAW)

    R = diag(R_RAW)

    R_delta = diag(R_DELTA_RAW)

    # Masa = 2.488kg -> m*g = 24.4 N, v EDF tabeli priblizno 86-88%

    u_hover = U_HOVER
    u_ref = vertcat(u_hover, 0, 0, 0, 0)

    # --------------------------------------------------------
    # COST FUNKCIJE
    # --------------------------------------------------------
    cost = 0
    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        x_error = (xk - X_ref) / scale_x
        cost += mtimes(x_error.T, Q @ x_error)

        u_error = (uk - u_ref) / scale_u
        cost += mtimes(u_error.T, R @ u_error)

        if k == 0:
            du = (uk - U_prev) / scale_u
        else:
            du = (uk - U[:, k - 1]) / scale_u
        cost += mtimes(du.T, R_delta @ du)

    xN_error = (X[:, N] - X_ref) / scale_x # normalizacija različnih enot
    cost += 3 * mtimes(xN_error.T, Q @ xN_error)

    opti.minimize(cost)

    # --------------------------------------------------------
    # OMEJITVE DINAMIKE MODELA - to more zmerej veljat (fizika sistema)
    # --------------------------------------------------------
    for k in range(N):
        x_next = F(X[:, k], U[:, k])
        opti.subject_to(X[:, k + 1] == x_next)

    # --------------------------------------------------------
    # ZAČETNA STANJA
    # --------------------------------------------------------
    opti.subject_to(X[:, 0] == X0)

    # --------------------------------------------------------
    # OMEJITVE AKTUATORJEV
    # --------------------------------------------------------
    opti.subject_to(opti.bounded(INPUT_LOWER[0], U[0, :], INPUT_UPPER[0]))      # EDF [%]
    opti.subject_to(opti.bounded(INPUT_LOWER[1], U[1:5, :], INPUT_UPPER[1]))   # servos [deg]

    # --------------------------------------------------------
    # SOLVER
    # --------------------------------------------------------
    opti.solver("ipopt", {
        "ipopt.hessian_approximation": "limited-memory",
        "ipopt.max_iter": 80,
        "ipopt.tol": 1e-3,
        "ipopt.acceptable_tol": 5e-2,
        "ipopt.acceptable_constr_viol_tol": 5e-2,
        "ipopt.acceptable_dual_inf_tol": 1e-1,
        "ipopt.acceptable_compl_inf_tol": 1e-3,
        "ipopt.acceptable_iter": 2,
        "ipopt.print_level": 0,
        "print_time": False,
    })

    return opti, X, U, X0, X_ref, U_prev
