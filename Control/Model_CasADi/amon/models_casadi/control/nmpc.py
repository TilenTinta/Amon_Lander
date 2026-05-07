from casadi import *


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
    scale_x = [
        10, 10, 5,        # pozicija
        5, 5, 5,          # hitrost
        1, 1, 1, 1,       # quaternion
        10, 10, 10,       # omega
        100,              # thrust
        50,               # thrust_dot
        45, 45, 45, 45,   # servo
        100, 100, 100, 100
    ]

    # Ukazi so v procentih/stopinjah. Skala 20 pomeni, da je 20 enot ze
    # velik odmik, zato optimizer ne skace po celotnem aktuatorskem obmocju.
    scale_u = [20, 20, 20, 20, 20]

    Q = diag([
        2, 2, 260,          # p = [x, y, z]
        4, 4, 120,          # v = [vx, vy, vz]
        5, 40, 40, 10,      # q = [qw, qx, qy, qz]
        8, 8, 2,            # omega = [wx, wy, wz]
        0.0,                # T
        0.0,                # T_dot
        0.2, 0.2, 0.2, 0.2, # delta
        0.1, 0.1, 0.1, 0.1  # delta_dot
    ])

    R = diag([
        0.12, 
        0.25, 
        0.25, 
        0.25, 
        0.25
    ])

    R_delta = diag([
        0.5, 
        0.8, 
        0.8, 
        0.8, 
        0.8
    ])

    # Masa = 2.488kg -> m*g = 24.4 N, v EDF tabeli priblizno 86-88%

    u_hover = 87.0
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
    opti.subject_to(opti.bounded(0, U[0, :], 100))      # EDF [%]
    opti.subject_to(opti.bounded(-45, U[1:5, :], 45))   # servos [deg]

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
