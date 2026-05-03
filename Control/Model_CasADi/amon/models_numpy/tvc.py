"""
Amon Lander - TVC model

Opombe:
- vse sile in momenti so v BODY sistemu
- uporablja nelinearni model sin(delta)
"""

import numpy as np


# ============================================================
# EDF sila
# ============================================================
def thrust_force(T):
    """
    EDF sila v body sistemu.
    Le pretvorba v matrični zapis

    F_T = [0, 0, T]
    """

    return np.array([0.0, 0.0, T])


# ============================================================
# Moment odmika EDF
# ============================================================
def thrust_torque_offset(T, params):
    """
    tau_T = r_T x F_T
    """

    F_T = thrust_force(T)
    r_T = params.physical.r_T

    return np.cross(r_T, F_T)


# ============================================================
# Sila enega fina
# ============================================================
def single_fin_force(T, delta_i, n_i, k_i):
    """
    F_i = T * k_i * sin(delta_i) * n_i
    """

    return T * k_i * np.sin(delta_i) * n_i


# ============================================================
# Sila vseh finov
# ============================================================
def fins_force(T, delta, params):
    """
    Vsota sil vseh finov
    """

    F_total = np.zeros(3)

    for i in range(4):
        n_i = params.tvc.n_fins[:, i]
        k_i = params.tvc.k_fins[i]

        F_i = single_fin_force(T, delta[i], n_i, k_i)

        F_total += F_i

    return F_total


# ============================================================
# Moment vseh finov
# ============================================================
def fins_torque(T, delta, params):
    """
    Vsota momentov vseh finov
    """

    tau_total = np.zeros(3)

    for i in range(4):
        r_i = params.tvc.r_fins[:, i]
        n_i = params.tvc.n_fins[:, i]
        k_i = params.tvc.k_fins[i]

        F_i = single_fin_force(T, delta[i], n_i, k_i)

        tau_i = np.cross(r_i, F_i)

        tau_total += tau_i

    return tau_total


# ============================================================
# Skupna sila (BODY)
# ============================================================
def total_force_body(T, delta, params):
    """
    F_b = F_T + F_fins + f_0_T
    """

    F_T = thrust_force(T)
    F_fins = fins_force(T, delta, params)

    f_0_T = params.physical.f_0_T

    return F_T + F_fins + f_0_T


# ============================================================
# Skupni moment (BODY)
# ============================================================
def total_torque_body(T, delta, params):
    """
    tau_b = tau_T + tau_fins + tau_0_T
    """

    tau_T = thrust_torque_offset(T, params)
    tau_fins = fins_torque(T, delta, params)

    tau_0_T = params.physical.tau_0_T

    return tau_T + tau_fins + tau_0_T