from casadi import *

# ============================================================
# Import komponente
# ============================================================
from models_casadi.components.actuators import (
    edf_block_2nd_order,
    servo_block_4_2nd_order,
)

from models_casadi.components.tvc import (
    total_force_body,
    total_torque_body,
)

from models_casadi.components.rigid_body import (
    position_dot,
    velocity_dot,
    omega_dot,
)

from models_casadi.components.rotations import quat_derivative

from models_casadi.parameters import *


# ============================================================
# Unpack state
# ============================================================
def unpack_state(x):
    p = x[X_P]
    v = x[X_V]
    q = x[X_Q]
    omega = x[X_W]

    T = x[X_T]
    T_dot = x[X_T_DOT]

    delta = x[X_DELTA]
    delta_dot = x[X_DELTA_DOT]

    return p, v, q, omega, T, T_dot, delta, delta_dot


# ============================================================
# Unpack input
# ============================================================
def unpack_input(u):
    edf_cmd = u[0]
    servo_cmd = u[1:5]

    return edf_cmd, servo_cmd


# ============================================================
# FULL MODEL
# ============================================================
def full_dynamics(x, u, params):
    """
    Celoten Amon drone CasADi model
    """

    p, v, q, omega, T, T_dot, delta, delta_dot = unpack_state(x)
    edf_cmd, servo_cmd = unpack_input(u)

    # --------------------------------------------------------
    # EDF (2nd order)
    # --------------------------------------------------------
    dT, dT_dot, T_cmd = edf_block_2nd_order(
        T, T_dot, edf_cmd, params
    )

    # --------------------------------------------------------
    # Servo (4x 2nd order)
    # --------------------------------------------------------
    ddelta, ddelta_dot, delta_cmd = servo_block_4_2nd_order(
        delta, delta_dot, servo_cmd, params
    )

    # --------------------------------------------------------
    # TVC
    # --------------------------------------------------------
    F_b = total_force_body(T, delta, params)
    tau_b = total_torque_body(T, delta, params)

    # --------------------------------------------------------
    # Rigid body
    # --------------------------------------------------------
    p_dot = position_dot(v)
    v_dot = velocity_dot(v, q, F_b, params)
    q_dot = quat_derivative(q, omega)
    w_dot = omega_dot(omega, tau_b, params)

    # --------------------------------------------------------
    # sestavi x_dot
    # --------------------------------------------------------
    x_dot = vertcat(
        p_dot,
        v_dot,
        q_dot,
        w_dot,
        dT,
        dT_dot,
        ddelta,
        ddelta_dot
    )

    return x_dot