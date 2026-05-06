from casadi import *

# ============================================================
# Import komponente
# ============================================================
from models_casadi.components.actuators import (
    edf_block_instant,
    edf_block_1st_order,
    edf_block_2nd_order,
    servo_block_4_instant,
    servo_block_4_1st_order,
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
# FULL MODEL - takojšnji odziv
# ============================================================
def full_dynamics_instant(x, u, params):
    """
    Celoten Amon drone CasADi model (instant aktuatorji)
    """

    p, v, q, omega, T, T_dot, delta, delta_dot = unpack_state(x)
    edf_cmd, servo_cmd = unpack_input(u)

    # --------------------------------------------------------
    # EDF (instant)
    # --------------------------------------------------------
    T_cmd = edf_block_instant(edf_cmd, params)

    # --------------------------------------------------------
    # Servo (instant)
    # --------------------------------------------------------
    delta_cmd = servo_block_4_instant(servo_cmd, params)

    # --------------------------------------------------------
    # TVC (uporabimo TAKOJ command)
    # --------------------------------------------------------
    F_b = total_force_body(T_cmd, delta_cmd, params)
    tau_b = total_torque_body(T_cmd, delta_cmd, params)

    # --------------------------------------------------------
    # Rigid body
    # --------------------------------------------------------
    p_dot = position_dot(v)
    v_dot = velocity_dot(v, q, F_b, params)
    q_dot = quat_derivative(q, omega)
    w_dot = omega_dot(omega, tau_b, params)

    # --------------------------------------------------------
    # state-e prisilimo na command
    # --------------------------------------------------------
    dT = T_cmd - T
    dT_dot = 0

    ddelta = delta_cmd - delta
    ddelta_dot = MX.zeros(4)

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



# ============================================================
# FULL MODEL - prvi red
# ============================================================
def full_dynamics_1st_order(x, u, params):
    """
    Celoten Amon drone CasADi model (1. red aktuatorji)
    """

    p, v, q, omega, T, T_dot, delta, delta_dot = unpack_state(x)
    edf_cmd, servo_cmd = unpack_input(u)

    # --------------------------------------------------------
    # EDF (1st order)
    # --------------------------------------------------------
    dT, T_cmd = edf_block_1st_order(
        T, edf_cmd, params
    )

    # --------------------------------------------------------
    # Servo (1st order)
    # --------------------------------------------------------
    ddelta, delta_cmd = servo_block_4_1st_order(
        delta, servo_cmd, params
    )

    # --------------------------------------------------------
    # TVC (uporabljamo trenutni T in delta)
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
    # Ignoriramo 2nd order dinamiko
    # --------------------------------------------------------
    dT_dot = 0
    ddelta_dot = MX.zeros(4)

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



# ============================================================
# FULL MODEL - drugi red
# ============================================================
def full_dynamics_2nd_order(x, u, params):
    """
    Celoten Amon drone CasADi model (2. red aktuatorji)
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
