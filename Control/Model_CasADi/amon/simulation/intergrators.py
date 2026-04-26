"""
Amon Lander - Integratorji

Opombe:
- po vsakem koraku normalizira kvaternion
"""

import numpy as np

from models.parameters import X_Q
from models.rotations import quat_normalize


# ============================================================
# Normalizacija kvaterniona v vektorju stanja
# ============================================================
def normalize_state_quaternion(x):
    """
    Normalizira kvaternion znotraj celotnega vektorja stanja.
    """

    x_new = x.copy()
    x_new[X_Q] = quat_normalize(x_new[X_Q])

    return x_new


# ============================================================
# Eulerjev integracijski korak
# ============================================================
def euler_step(dynamics_function, x, u, dt, params):
    """
    En Eulerjev integracijski korak.

    Vhod:
    - dynamics_function : funkcija modela, npr. full_dynamics
    - x                : trenutno stanje
    - u                : trenutni vhod
    - dt               : časovni korak
    - params           : parametri modela

    Izhod:
    - x_next : naslednje stanje
    """

    x_dot = dynamics_function(x, u, params)

    x_next = x + dt * x_dot

    x_next = normalize_state_quaternion(x_next)

    return x_next


# ============================================================
# RK4 integracijski korak
# ============================================================
def rk4_step(dynamics_function, x, u, dt, params):
    """
    En Runge-Kutta 4 integracijski korak.

    Vhod:
    - dynamics_function : funkcija modela, npr. full_dynamics
    - x                : trenutno stanje
    - u                : trenutni vhod
    - dt               : časovni korak
    - params           : parametri modela

    Izhod:
    - x_next : naslednje stanje
    """

    k1 = dynamics_function(x, u, params)
    k2 = dynamics_function(x + 0.5 * dt * k1, u, params)
    k3 = dynamics_function(x + 0.5 * dt * k2, u, params)
    k4 = dynamics_function(x + dt * k3, u, params)

    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    x_next = normalize_state_quaternion(x_next)

    return x_next


# ============================================================
# Več zaporednih integracijskih korakov
# ============================================================
def integrate_steps(dynamics_function, x0, U, dt, params, method="rk4"):
    """
    Simulira sistem čez več korakov.

    Vhod:
    - dynamics_function : funkcija modela
    - x0               : začetno stanje
    - U                : matrika vhodov [N x nu]
    - dt               : časovni korak
    - params           : parametri modela
    - method           : "euler" ali "rk4"

    Izhod:
    - X : matrika stanj [N+1 x nx]
    """

    n_steps = U.shape[0]
    nx = len(x0)

    X = np.zeros((n_steps + 1, nx))
    X[0] = x0.copy()

    x = x0.copy()

    for k in range(n_steps):
        u = U[k]

        if method == "euler":
            x = euler_step(dynamics_function, x, u, dt, params)
        elif method == "rk4":
            x = rk4_step(dynamics_function, x, u, dt, params)
        else:
            raise ValueError("Nepodprta metoda integracije. Uporabi 'euler' ali 'rk4'.")

        X[k + 1] = x

    return X