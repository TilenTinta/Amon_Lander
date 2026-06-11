from casadi import *


def normalize_quaternion(q):
    """
    Normalizacija kvaterniona brez NaN vrednosti
    """
    norm_q = norm_2(q)

    return if_else(
        norm_q > 1e-8,
        q / norm_q,
        vertcat(1, 0, 0, 0)   # fallback
    )


def rk4_step(f, x, u, dt):

    k1 = f(x, u)
    k2 = f(x + dt/2 * k1, u)
    k3 = f(x + dt/2 * k2, u)
    k4 = f(x + dt * k3, u)

    x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    # --------------------------------------------------------
    # Normalizacija kvaterniona
    # --------------------------------------------------------
    q = x_next[6:10]
    q = normalize_quaternion(q)
    x_next[6:10] = q

    return x_next


