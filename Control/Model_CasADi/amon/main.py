"""
Amon Lander - Glavni file
"""

import numpy as np
import matplotlib.pyplot as plt

from amon.models.parameters import (
    X_P,
    X_V,
    X_Q,
    X_W,
    X_T,
    X_DELTA
)


# ============================================================
# Time vector
# ============================================================
def make_time_vector(X, dt):
    return np.arange(X.shape[0]) * dt


# ============================================================
# Plot position
# ============================================================
def plot_position(X, dt):
    t = make_time_vector(X, dt)
    p = X[:, X_P]

    plt.figure()
    plt.plot(t, p[:, 0], label="px")
    plt.plot(t, p[:, 1], label="py")
    plt.plot(t, p[:, 2], label="pz")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("Position")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot velocity
# ============================================================
def plot_velocity(X, dt):
    t = make_time_vector(X, dt)
    v = X[:, X_V]

    plt.figure()
    plt.plot(t, v[:, 0], label="vx")
    plt.plot(t, v[:, 1], label="vy")
    plt.plot(t, v[:, 2], label="vz")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Velocity")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot quaternion
# ============================================================
def plot_quaternion(X, dt):
    t = make_time_vector(X, dt)
    q = X[:, X_Q]

    plt.figure()
    plt.plot(t, q[:, 0], label="qw")
    plt.plot(t, q[:, 1], label="qx")
    plt.plot(t, q[:, 2], label="qy")
    plt.plot(t, q[:, 3], label="qz")
    plt.xlabel("Time [s]")
    plt.ylabel("Quaternion")
    plt.title("Quaternion")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot angular velocity
# ============================================================
def plot_angular_velocity(X, dt):
    t = make_time_vector(X, dt)
    w = X[:, X_W]

    plt.figure()
    plt.plot(t, w[:, 0], label="wx")
    plt.plot(t, w[:, 1], label="wy")
    plt.plot(t, w[:, 2], label="wz")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular velocity [rad/s]")
    plt.title("Angular Velocity")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot thrust
# ============================================================
def plot_thrust(X, dt):
    t = make_time_vector(X, dt)
    T = X[:, X_T]

    plt.figure()
    plt.plot(t, T, label="T")
    plt.xlabel("Time [s]")
    plt.ylabel("Thrust [N]")
    plt.title("EDF Thrust")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot fin deflections
# ============================================================
def plot_fin_deflections(X, dt):
    t = make_time_vector(X, dt)
    delta = X[:, X_DELTA]

    plt.figure()
    plt.plot(t, delta[:, 0], label="delta1")
    plt.plot(t, delta[:, 1], label="delta2")
    plt.plot(t, delta[:, 2], label="delta3")
    plt.plot(t, delta[:, 3], label="delta4")
    plt.xlabel("Time [s]")
    plt.ylabel("Fin angle [rad]")
    plt.title("Fin Deflections")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


# ============================================================
# Plot all main states
# ============================================================
def plot_all_states(X, dt):
    plot_position(X, dt)
    plot_velocity(X, dt)
    plot_quaternion(X, dt)
    plot_angular_velocity(X, dt)
    plot_thrust(X, dt)
    plot_fin_deflections(X, dt)
    plt.show()


# ============================================================
# Plot inputs
# ============================================================
def plot_inputs(U, dt):
    t = np.arange(U.shape[0]) * dt

    plt.figure()
    plt.plot(t, U[:, 0], label="s_cmd")
    plt.xlabel("Time [s]")
    plt.ylabel("Throttle command")
    plt.title("Throttle Command")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.figure()
    plt.plot(t, U[:, 1], label="delta_cmd_1")
    plt.plot(t, U[:, 2], label="delta_cmd_2")
    plt.plot(t, U[:, 3], label="delta_cmd_3")
    plt.plot(t, U[:, 4], label="delta_cmd_4")
    plt.xlabel("Time [s]")
    plt.ylabel("Fin command [rad]")
    plt.title("Fin Commands")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()