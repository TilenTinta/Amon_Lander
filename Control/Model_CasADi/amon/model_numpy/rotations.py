"""
Amon Lander - Rotacije in kvaternioni

Opombe:
- q = [qw, qx, qy, qz]
- kvaternion preslika BODY -> INERTIAL
"""

import numpy as np


# ============================================================
# Kvaternion -> rotacijska matrika
# ============================================================
def quat_to_rot(q):
    """
    Pretvorba kvaterniona v rotacijsko matriko.

    q = [qw, qx, qy, qz]
    """

    qw, qx, qy, qz = q

    R = np.array([
        [
            1 - 2*(qy*qy + qz*qz),
            2*(qx*qy - qw*qz),
            2*(qx*qz + qw*qy)
        ],
        [
            2*(qx*qy + qw*qz),
            1 - 2*(qx*qx + qz*qz),
            2*(qy*qz - qw*qx)
        ],
        [
            2*(qx*qz - qw*qy),
            2*(qy*qz + qw*qx),
            1 - 2*(qx*qx + qy*qy)
        ]
    ])

    return R


# ============================================================
# Omega matrika
# ============================================================
def omega_matrix(omega):
    """
    Omega matrika za odvod kvaterniona.
    """

    wx, wy, wz = omega

    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx,  0.0,  wz, -wy],
        [wy, -wz,  0.0,  wx],
        [wz,  wy, -wx,  0.0]
    ])


# ============================================================
# Odvod kvaterniona
# ============================================================
def quat_derivative(q, omega):
    """
    Izračun q_dot.
    @ - matrično množenje
    """

    return 0.5 * omega_matrix(omega) @ q


# ============================================================
# Normalizacija kvaterniona
# ============================================================
def quat_normalize(q):
    """
    Normalizacija kvaterniona.
    """

    norm = np.linalg.norm(q)

    if norm == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


# ============================================================
# Normalizacija kvaterniona v stanju
# ============================================================
def normalize_state_quaternion(x, idx_q):
    """
    Normalizira kvaternion znotraj stanja.
    """

    x_new = x.copy()

    q = x[idx_q]
    x_new[idx_q] = quat_normalize(q)

    return x_new


# ============================================================
# Kvaternion -> Euler koti
# ============================================================
def quat_to_euler(q):
    """
    Pretvorba kvaterniona v Euler kote (roll, pitch, yaw).

    Vrne kote v radianih.
    """

    qw, qx, qy, qz = q

    # Roll (X)
    sinr_cosp = 2 * (qw*qx + qy*qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (Y)
    sinp = 2 * (qw*qy - qz*qx)

    # clamp zaradi numerike
    sinp = np.clip(sinp, -1.0, 1.0)

    pitch = np.arcsin(sinp)

    # Yaw (Z)
    siny_cosp = 2 * (qw*qz + qx*qy)
    cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


# ============================================================
# Euler koti iz celotnega stanja
# ============================================================
def state_to_euler(x, idx_q):
    """
    Iz celotnega stanja vrne Euler kote.
    """

    q = x[idx_q]

    return quat_to_euler(q)