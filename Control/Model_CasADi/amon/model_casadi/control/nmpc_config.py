import numpy as np


U_HOVER = 90.0

# Masa = 2.488kg -> m*g = 24.4 N, v EDF tabeli priblizno 86-88%

# --------------------------------------------------------
# NMPC TIMING
# --------------------------------------------------------
NMPC_DT = 0.01      # 100 Hz
NMPC_N = 20         # horizont = 0.20 s
NMPC_MODEL_TYPE = "instant"  # "instant", "1st_order", "2nd_order"
SIM_TIME = 10.0

# --------------------------------------------------------
# COST FUNCTION
# --------------------------------------------------------
SCALE_X = [
    10, 10, 5,        # position
    5, 5, 5,          # velocity
    1, 1, 1, 1,       # quaternion
    10, 10, 10,       # omega
    100,              # thrust state
    50,               # thrust_dot
    45, 45, 45, 45,   # servo states
    100, 100, 100, 100
]

# Ukazi so v procentih/stopinjah. Skala 20 pomeni, da je 20 enot ze
# velik odmik, zato optimizer ne skace po celotnem aktuatorskem obmocju.
SCALE_U = [20, 20, 20, 20, 20]

Q_RAW = [
    2, 2, 260,          # p = [x, y, z]
    4, 4, 120,          # v = [vx, vy, vz]
    5, 40, 40, 10,      # q = [qw, qx, qy, qz]
    8, 8, 2,            # omega = [wx, wy, wz]
    0.0,                # T
    0.0,                # T_dot
    0.2, 0.2, 0.2, 0.2, # delta
    0.1, 0.1, 0.1, 0.1  # delta_dot
]

R_RAW = [
    0.12, 
    0.25, 
    0.25, 
    0.25, 
    0.25
]

R_DELTA_RAW = [
    0.5, 
    0.8, 
    0.8, 
    0.8, 
    0.8
]

INPUT_LOWER = [0, -45, -45, -45, -45]
INPUT_UPPER = [90, 45, 45, 45, 45]


def scale_x_array():
    return np.array(SCALE_X, dtype=float)


def scale_u_array():
    return np.array(SCALE_U, dtype=float)


def q_raw_array():
    return np.array(Q_RAW, dtype=float)


def r_raw_array():
    return np.array(R_RAW, dtype=float)


def input_lower_array():
    return np.array(INPUT_LOWER, dtype=float)


def input_upper_array():
    return np.array(INPUT_UPPER, dtype=float)
