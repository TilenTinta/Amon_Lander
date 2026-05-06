from casadi import *
from models_casadi.components.rotations import quat_to_rot


# ============================================================
# Gravitacija
# ============================================================
def gravity_vector(params):
    """
    g v inertial sistemu
    """
    return vertcat(0, 0, -params.physical.g)


# ============================================================
# Pozicija
# ============================================================
def position_dot(v):
    """
    p_dot = v
    """
    return v


# ============================================================
# Hitrost
# ============================================================
def velocity_dot(v, q, F_b, params):
    """
    v_dot = g + R(q) * F_b / m
    """

    R = quat_to_rot(q)
    g = gravity_vector(params)

    # masa v kg (jaz podajam v gramih)
    mass_kg = params.physical.mass / 1000.0

    return g + mtimes(R, F_b) / mass_kg


# ============================================================
# Coriolis člen
# ============================================================
def coriolis_term(omega, params):
    """
    omega x (J * omega)
    """

    J = params.physical.J

    return cross(omega, mtimes(J, omega))


# ============================================================
# Kotna dinamika
# ============================================================
def omega_dot(omega, tau_b, params):
    """
    omega_dot = J^-1 (tau - omega x Jω)
    """

    J = params.physical.J

    return solve(J, tau_b - coriolis_term(omega, params))