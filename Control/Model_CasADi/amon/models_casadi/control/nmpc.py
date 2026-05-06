from casadi import *

def build_nmpc(F, nx, nu, N, dt):

    # --------------------------------------------------------
    # OPTIMIZER
    # --------------------------------------------------------
    opti = Opti()

    # --------------------------------------------------------
    # VARIABLES
    # --------------------------------------------------------
    X = opti.variable(nx, N+1)   # states - cela prihodnja trajektorija
    U = opti.variable(nu, N)     # controls - cel control signal

    # --------------------------------------------------------
    # PARAMETERS (reference + initial state)
    # --------------------------------------------------------
    X_ref = opti.parameter(nx)  # ciljna referenca  
    X0 = opti.parameter(nx)     # trenutno stanje

    # --------------------------------------------------------
    # COST FUNCTION
    # --------------------------------------------------------
    # Q - koliko kaznuješ napako tega stanja
    Q = diag([
        10, 10, 100,         # p = [x, y, z] → pozicija
        1, 1, 100,           # v = [vx, vy, vz] → hitrost
        10, 10, 10, 10,      # q = quaternion → orientacija (stabilnost!)
        1, 1, 1,             # omega = [wx, wy, wz] → kotna hitrost
        0.01,                   # T → thrust 
        0.01,                # T_dot → sprememba thrust-a ("gladkost" EDF)
        1, 1, 1, 1,          # delta = koti servojev 
        0.1, 0.1, 0.1, 0.1   # delta_dot = hitrost servojev (gladko gibanje)
    ])

    # R - koliko kaznuješ uporabo aktuatorjev
    R = diag([
        0.1,   # EDF → thrust
        0.1,   # servo 1
        0.1,   # servo 2
        0.1,   # servo 3
        0.1    # servo 4
    ])  

    cost = 0
    u_hover = 87  # približen thrust za lebdenje = 80%
    W_hover = 30

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        # ----------------------------------------
        # STATE COST - kaznuješ odmik od cilja
        # ----------------------------------------
        #cost += mtimes((xk - X_ref).T, Q @ (xk - X_ref))

        # ----------------------------------------
        # CONTROL COST - kaznuješ agresivne ukaze
        # ----------------------------------------
        # cost += mtimes(uk.T, R @ uk)
        u_ref = vertcat(u_hover, 0, 0, 0, 0)
        cost += mtimes((uk - u_ref).T, R @ (uk - u_ref))

        # ----------------------------------------
        # HOVER THRUST COST - vem da pod n% ne leti
        # ----------------------------------------
        #cost += sumsqr(uk[0] - u_hover)
        #cost += W_hover * (uk[0] - u_hover)**2

    # ----------------------------------------
    # prisili sistem da zaključi na cilju
    # ----------------------------------------
    xN = X[:, N]
    Q_terminal = 2 * Q
    cost += mtimes((xN - X_ref).T, Q_terminal @ (xN - X_ref))


    opti.minimize(cost)

    # --------------------------------------------------------
    # DYNAMICS CONSTRAINTS
    # --------------------------------------------------------
    for k in range(N):
        x_next = F(X[:, k], U[:, k])
        opti.subject_to(X[:, k+1] == x_next)

    # --------------------------------------------------------
    # INITIAL CONDITION
    # --------------------------------------------------------
    opti.subject_to(X[:, 0] == X0)

    # --------------------------------------------------------
    # OMEJITVE AKTUATORJEV
    # --------------------------------------------------------
    opti.subject_to(opti.bounded(0, U[0, :], 100))     # EDF
    opti.subject_to(opti.bounded(-45, U[1:5, :], 45))  # servos

    # --------------------------------------------------------
    # SOLVER
    # --------------------------------------------------------
    opti.solver("ipopt") # nonlinear optimizer

    return opti, X, U, X0, X_ref