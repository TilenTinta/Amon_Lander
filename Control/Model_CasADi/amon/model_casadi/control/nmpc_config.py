import numpy as np


U_HOVER = 90.0

# Masa = 2.488kg -> m*g = 24.4 N, v EDF tabeli priblizno 86-88%

# --------------------------------------------------------
# NMPC TIMING
# --------------------------------------------------------
NMPC_DT = 0.02      # 100Hz = 0.01, 50Hz = 0.02
NMPC_N = 6         # horizont = NMPC_DT * NMPC_N
NMPC_MODEL_TYPE = "instant"  # "instant", "1st_order", "2nd_order"
SIM_TIME = 10.0

# --------------------------------------------------------
# INITIAL POSITION
# --------------------------------------------------------
INITIAL_ROLL_DEG = 10.0
INITIAL_PITCH_DEG = 0.0
INITIAL_YAW_DEG = 0.0

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
    100, 100, 100, 100 # servo rates
]

# Ukazi so v procentih/stopinjah. Skala 20 pomeni, da je 20 enot ze
# velik odmik, zato optimizer ne skace po celotnem aktuatorskem obmocju.
SCALE_U = [20, 20, 20, 20, 20]

# Q_RAW = [
#     2, 2, 260,          # p = [x, y, z]
#     4, 4, 120,          # v = [vx, vy, vz]
#     5, 40, 40, 10,      # q = [qw, qx, qy, qz]
#     8, 8, 2,            # omega = [wx, wy, wz]
#     0.0,                # T
#     0.0,                # T_dot
#     0.2, 0.2, 0.2, 0.2, # delta
#     0.1, 0.1, 0.1, 0.1  # delta_dot
# ]

Q_RAW = [
    0.2, 0.2, 80,       # x, y, z position
    1.0, 1.0, 80,       # vx, vy, vz
    5, 80, 80, 5,       # qw, qx, qy, qz
    30, 30, 5,          # wx, wy, wz
    0.0,                # T
    0.0,                # T_dot
    0.1, 0.1, 0.1, 0.1, # servo states
    0.2, 0.2, 0.2, 0.2  # servo dot
]

# R_RAW = [
#     0.12, 
#     0.25, 
#     0.25, 
#     0.25, 
#     0.25
# ]

R_RAW = [
    0.3,   # thrust
    1.5, 1.5, 1.5, 1.5  # vanes
]

# R_DELTA_RAW = [
#     0.5, 
#     0.8, 
#     0.8, 
#     0.8, 
#     0.8
# ]

R_DELTA_RAW = [
    2.0,
    4.0, 4.0, 4.0, 4.0
]

INPUT_LOWER = [70, -45, -45, -45, -45] # EDF 0 -> 70
INPUT_UPPER = [90, 45, 45, 45, 45]

# Sprememba števila stanj glede na izbran model (poenostavitev za MCU)
def state_selection(nx):
    if nx == 13:
        return list(range(13))
    if nx == 18:
        return list(range(13)) + [13] + list(range(15, 19))
    if nx == 23:
        return list(range(23))

    raise ValueError(f"Nepodprta dimenzija stanja: {nx}")


def select_state_weights(values, nx=None):
    values = np.array(values, dtype=float)

    if nx is None:
        return values

    return values[state_selection(nx)]


def scale_x_array(nx=None):
    return select_state_weights(SCALE_X, nx)


def scale_u_array():
    return np.array(SCALE_U, dtype=float)


def q_raw_array(nx=None):
    return select_state_weights(Q_RAW, nx)


def r_raw_array():
    return np.array(R_RAW, dtype=float)


def input_lower_array():
    return np.array(INPUT_LOWER, dtype=float)


def input_upper_array():
    return np.array(INPUT_UPPER, dtype=float)
