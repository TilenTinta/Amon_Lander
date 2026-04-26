"""
Amon Lander - Celoten model dinamike 

Opombe:
- poseben EDF navor trenutno ni dodatno stanje
- TVC uporablja sin(delta), kot v Simulinku
"""

import numpy as np

from models.parameters import (X_P, X_V, X_Q, X_W, X_T, X_DELTA)
from .rotations import quat_derivative
from .actuators import edf_block, edf_block_2nd_order, servo_block_4
from .tvc import total_force_body, total_torque_body
from .rigid_body import position_dot, velocity_dot, omega_dot


# ============================================================
# Razbitje vektorja stanj
# ============================================================
def unpack_state(x):
    """
    Razbije vektor stanj na posamezne komponente.
    """

    p = x[X_P]
    v = x[X_V]
    q = x[X_Q]
    omega = x[X_W]
    T = x[X_T]
    delta = x[X_DELTA]

    return p, v, q, omega, T, delta


# ============================================================
# Razbitje vhodnega vektorja
# ============================================================
def unpack_input(u):
    """
    Razbije vhodni vektor na:
    - ukaz EDF
    - ukaze 4 servotov
    """

    edf_cmd = u[0]
    servo_cmd = u[1:5]

    return edf_cmd, servo_cmd


# ============================================================
# Celoten model dinamike/drona
# ============================================================
def full_dynamics(x, u, params):
    """
    Celoten nelinearni model.

    Vhod:
    - x : vektor stanj
    - u : vhodni vektor
    - params : parametri modela

    Izhod:
    - x_dot : odvod stanja
    """

    # --------------------------------------------------------
    # Razbitje stanja in vhodov
    # --------------------------------------------------------
    p, v, q, omega, T, delta = unpack_state(x)
    edf_cmd, servo_cmd = unpack_input(u)

    # --------------------------------------------------------
    # EDF blok
    # --------------------------------------------------------
    T_dot, T_cmd = edf_block(T=T, edf_cmd=edf_cmd, params=params)
    #T_ddot, T_dot, T_cmd = edf_block_2nd_order(T=T, T_dot=T_dot, edf_cmd=edf_cmd, params=params)

    # --------------------------------------------------------
    # Servo bloki
    # --------------------------------------------------------
    delta_dot, delta_cmd_rad = servo_block_4(delta=delta, servo_cmd=servo_cmd,params=params)

    # --------------------------------------------------------
    # TVC model
    # Uporabimo vzgon T in kote delta
    # --------------------------------------------------------
    F_b = total_force_body(T=T, delta=delta, params=params)

    tau_b = total_torque_body(T=T, delta=delta, params=params)

    # --------------------------------------------------------
    # Dinamika togega telesa
    # --------------------------------------------------------
    p_dot = position_dot(v)
    v_dot = velocity_dot(v, q, F_b, params)
    q_dot = quat_derivative(q, omega)
    w_dot = omega_dot(omega, tau_b, params)

    # --------------------------------------------------------
    # Sestava odvoda stanj
    # --------------------------------------------------------
    x_dot = np.zeros_like(x)

    x_dot[X_P] = p_dot
    x_dot[X_V] = v_dot
    x_dot[X_Q] = q_dot
    x_dot[X_W] = w_dot
    x_dot[X_T] = T_dot
    x_dot[X_DELTA] = delta_dot

    return x_dot


# ============================================================
# Pomožna funkcija za pregled blokov
# ============================================================
def evaluate_model_blocks(x, u, params):
    """
    Pomožna funkcija za debug.

    Vrne pomembne vmesne signale:
    - T_cmd
    - delta_cmd_rad
    - F_b
    - tau_b
    """

    _, _, _, _, T, delta = unpack_state(x)
    edf_cmd, servo_cmd = unpack_input(u)

    _, T_cmd = edf_block(T=T, edf_cmd=edf_cmd, params=params)

    _, delta_cmd_rad = servo_block_4(delta=delta, servo_cmd=servo_cmd, params=params)

    F_b = total_force_body(T=T, delta=delta, params=params)

    tau_b = total_torque_body(T=T, delta=delta, params=params)

    return {"T_cmd": T_cmd, "delta_cmd_rad": delta_cmd_rad,"F_b": F_b,"tau_b": tau_b}