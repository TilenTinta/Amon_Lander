"""
Amon Lander - Dinamika togega telesa

Opombe:
- K_d = 0
- f_dist = 0
- masa je v parametrih podana v gramih
"""

import numpy as np
from .rotations import quat_to_rot


# ============================================================
# Gravitacijski vektor
# ============================================================
def gravity_vector(params):
    """
    Gravitacijski vektor v inercialnem koordinatnem sistemu.
    """

    return np.array([0.0, 0.0, params.physical.g])


# ============================================================
# Dinamika pozicije
# ============================================================
def position_dot(v):
    """
    Odvod pozicije.

    p_dot = v
    """

    return v.copy()


# ============================================================
# Dinamika hitrosti
# ============================================================
def velocity_dot(v, q, F_b, params):
    """
    Odvod hitrosti.

    Po Simulink modelu:
    v_dot = g + (1/m) * R(q) * F_b

    kjer:
    - v    : hitrost v inertial sistemu
    - q    : kvaternion
    - F_b  : sila v body sistemu
    - R(q) : preslikava Body -> Inertial

    Opomba:
    - masa je v gramih, zato jo pretvorimo v kg
    """

    R = quat_to_rot(q)
    g = gravity_vector(params)

    mass_kg = params.physical.mass / 1000.0

    return g + (R @ F_b) / mass_kg


# ============================================================
# Giroskopski člen
# ============================================================
def coriolis_term(omega, params):
    """
    Giroskopski člen:
    omega x (J * omega)
    """

    J = params.physical.J

    return np.cross(omega, J @ omega)


# ============================================================
# Dinamika kotne hitrosti
# ============================================================
def omega_dot(omega, tau_b, params):
    """
    Odvod kotne hitrosti.

    Eulerjeva enačba:
    omega_dot = J^-1 * (tau_b - omega x (J * omega))
    """

    J = params.physical.J

    return np.linalg.solve(J, tau_b - coriolis_term(omega, params))