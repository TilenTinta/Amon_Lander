from models_casadi.integrators.rk4 import normalize_quaternion
from casadi import *

# ============================================================
# Kvaternion -> rotacijska matrika
# ============================================================
def quat_to_rot(q):
    """
    q = [qw, qx, qy, qz]
    """

    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]

    R = vertcat(
        horzcat(1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)),
        horzcat(2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)),
        horzcat(2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy))
    )

    return R


# ============================================================
# Omega matrika
# ============================================================
def omega_matrix(omega):
    wx = omega[0]
    wy = omega[1]
    wz = omega[2]

    return vertcat(
        horzcat(0,   -wx, -wy, -wz),
        horzcat(wx,   0,   wz, -wy),
        horzcat(wy,  -wz,  0,   wx),
        horzcat(wz,   wy, -wx,  0)
    )


# ============================================================
# Odvod kvaterniona
# ============================================================
def quat_derivative(q, omega):
    #q = normalize_quaternion(q)
    return 0.5 * mtimes(omega_matrix(omega), q)