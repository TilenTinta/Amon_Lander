"""
Amon Lander - Plotting

Uporablja matplotlib za vizualizacijo rezultatov.
"""

import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path


OUTPUT_DIR = Path(__file__).resolve().parent / "output"


# ============================================================
# Plot EDF odziva
# ============================================================
def plot_edf_response(time, T_measured, T_sim, edf_cmd):
    """
    Primerjava realnega in simuliranega odziva EDF.
    """

    plt.figure()

    # Thrust
    plt.subplot(2, 1, 1)
    plt.plot(time, T_measured, label="T real")
    plt.plot(time, T_sim, "--", label="T simulacija")
    plt.ylabel("Thrust [N]")
    plt.legend()
    plt.grid()

    # Command
    plt.subplot(2, 1, 2)
    plt.plot(time, edf_cmd, label="EDF command [%]")
    plt.ylabel("EDF [%]")
    plt.xlabel("Time [s]")
    plt.grid()

    plt.tight_layout()
    #plt.show()
    output_dir = OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)  # creates folder if it doesn't exist
    plt.savefig(os.path.join(output_dir, "edf_identification.png"), dpi=150, bbox_inches="tight")
    plt.close()



# ============================================================
# Plot servo odziva
# ============================================================
def plot_servo_response(time, T_measured, T_sim, servo_cmd):
    """
    Primerjava realnega in simuliranega odziva servota.
    """

    plt.figure()

    # Thrust
    plt.subplot(2, 1, 1)
    plt.plot(time, T_measured, label="Kot real")
    plt.plot(time, T_sim, "--", label="Kot simulacija")
    plt.ylabel("Kot [deg]")
    plt.legend()
    plt.grid()

    # Command
    plt.subplot(2, 1, 2)
    plt.plot(time, servo_cmd, label="Servo command [deg]")
    plt.ylabel("Servo [deg]")
    plt.xlabel("Time [s]")
    plt.grid()

    plt.tight_layout()
    #plt.show()
    output_dir = OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)  # creates folder if it doesn't exist
    plt.savefig(os.path.join(output_dir, "servo_identification.png"), dpi=150, bbox_inches="tight")
    plt.close()


# ============================================================
# Plot stanj
# ============================================================
def plot_states(time, X):
    """
    Osnovni plot stanj (pozicija, hitrost).
    """

    plt.figure()

    # Pozicija
    plt.subplot(2, 1, 1)
    plt.plot(time, X[:, 0], label="x")
    plt.plot(time, X[:, 1], label="y")
    plt.plot(time, X[:, 2], label="z")
    plt.ylabel("Position [m]")
    plt.legend()
    plt.grid()

    # Hitrost
    plt.subplot(2, 1, 2)
    plt.plot(time, X[:, 3], label="vx")
    plt.plot(time, X[:, 4], label="vy")
    plt.plot(time, X[:, 5], label="vz")
    plt.ylabel("Velocity [m/s]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()
