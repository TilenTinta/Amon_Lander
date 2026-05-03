"""
Amon Lander - Simulator
"""

import numpy as np
from collections import deque

from models_numpy.full_dynamic import full_dynamics
from models_numpy.actuators import delay_buffer, edf_block, edf_block_2nd_order, servo_block_2nd_order
from simulation.intergrators import rk4_step


# ============================================================
# Glavna simulacija
# ============================================================
def simulate(x0, U, dt, params):
    """
    Izvede simulacijo.

    Vhod:
    - x0 : začetno stanje
    - U  : matrika vhodov [N x nu]
    - dt : časovni korak
    - params : parametri

    Izhod:
    - X : stanja [N+1 x nx]
    """

    n_steps = U.shape[0]
    nx = len(x0)

    X = np.zeros((n_steps + 1, nx))
    X[0] = x0.copy()

    x = x0.copy()

    for k in range(n_steps):
        u = U[k]

        x = rk4_step(full_dynamics, x, u, dt, params)

        X[k + 1] = x

    return X


# ============================================================
# Hover test
# ============================================================
def hover_input(params, thrust_hover):
    """
    Ustvari vhod za hover.

    thrust_hover:
    - vrednost EDF [%], ki približno drži dron v zraku
    """

    u = np.zeros(5)

    u[0] = thrust_hover  # EDF
    u[1:5] = 0.0         # servoti

    return u


def run_hover_test(x0, params, dt, sim_time, thrust_hover):
    """
    Zažene hover test.
    """

    n_steps = int(sim_time / dt)

    U = np.zeros((n_steps, 5))

    for k in range(n_steps):
        U[k] = hover_input(params, thrust_hover)

    X = simulate(x0, U, dt, params)

    return X, U


# ============================================================
# Step test (en servo)
# ============================================================
def step_input(base_thrust, step_value, step_start, k):
    """
    Ustvari vhod za step test.

    step_value: stopinje (servo)
    """

    u = np.zeros(5)

    u[0] = base_thrust

    if k >= step_start:
        u[1] = step_value  # servo 1

    return u


def run_step_test(x0, params, dt, sim_time, thrust_hover, step_deg):
    """
    Step test na enem servotu.
    """

    n_steps = int(sim_time / dt)
    step_start = n_steps // 4

    U = np.zeros((n_steps, 5))

    for k in range(n_steps):
        U[k] = step_input(base_thrust=thrust_hover, step_value=step_deg, step_start=step_start, k=k)

    X = simulate(x0, U, dt, params)

    return X, U


# ============================================================
# Dodatno: časovni vektor
# ============================================================
def time_vector(n_steps, dt):
    """
    Ustvari časovni vektor.
    """

    return np.arange(0, (n_steps + 1) * dt, dt)


# ============================================================
# Dodatno: simulacija EDF
# ============================================================
def simulate_edf(time, edf_cmd, params):
    dt = time[1] - time[0]

    T = 0.0
    T_dot = 0.0
    T_sim = []

    delay_buffer = delay_buffer(params.actuator.edf_delay, dt, init_value=0.0)

    for k in range(len(time)):
        cmd = edf_cmd[k]

        #T_dot, T_cmd = edf_block(T, cmd, params)
        dT, dT_dot, T_cmd, edf_delayed = edf_block_2nd_order(T, T_dot, cmd, delay_buffer, params)

        # Eulerjeva integracija
        T = T + dt * dT
        T_dot = T_dot + dt * dT_dot

        #print(T, T_dot, dT, dT_dot)

        T_sim.append(T)

    return np.array(T_sim)


# ============================================================
# Dodatno: simulacija servo
# ============================================================
def simulate_servo(time, servo_cmd, params):
    dt = time[1] - time[0]

    delta = 0.0
    delta_dot = 0.0
    delta_sim = []

    delay = delay_buffer(params.actuator.servo_delay, dt, init_value=0.0)

    for k in range(len(time)):
        cmd = servo_cmd[k]

        ddelta, ddelta_dot, delta_cmd, servo_cmd_delayed = servo_block_2nd_order(delta, delta_dot, cmd, delay, params)

        # Eulerjeva integracija
        delta = delta + dt * ddelta
        delta_dot = delta_dot + dt * ddelta_dot

        delta_sim.append(delta)

    return np.rad2deg(np.array(delta_sim))
