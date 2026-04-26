from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from models.parameters import AmonParams
from simulation.plotting import plot_servo_response
from simulation.simulator import simulate_servo

params = AmonParams()

# load data - csv file
_AMON_DIR = Path(__file__).resolve().parents[1]
_DATA_PATH = _AMON_DIR / "data" / "servo_encoder_log_20260423.csv"

data = np.genfromtxt(
    str(_DATA_PATH),
    delimiter=",",
    names=True,
    dtype=None,
    encoding="utf-8",
)

SERVO_CHANNEL = "y_plus"  # None - za vse kanale / x_plus, x_minus, y_plus, y_minus
if SERVO_CHANNEL is not None:
    channel = data["servo_channel"].astype(str)
    data = data[channel == SERVO_CHANNEL]

time = data["pc_time_s"].astype(float)
servo_cmd_from_log = data["commanded_angle_deg"].astype(float)
delta_measured_deg = data["encoder_angle_deg"].astype(float)

# --- Ročne servo komande/rampa --------------------------------------
USE_MANUAL_SERVO_CMD = False

# --- IDENTIFICATION ----------------------------------------------------------
RUN_IDENTIFICATION = False




def stepwise_cmd_from_knots(time_s: np.ndarray, knot_time_s: np.ndarray, knot_cmd: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(knot_time_s, time_s, side="right") - 1
    idx = np.clip(idx, 0, len(knot_cmd) - 1)
    return knot_cmd[idx]


def residuals(theta, time_s, servo_cmd_deg, delta_measured_deg, params_):
    params_.actuator.servo_delay = theta[0]
    params_.actuator.wn_up_servo = theta[1]
    params_.actuator.zeta_up_servo = theta[2]
    params_.actuator.wn_down_servo = theta[3]
    params_.actuator.zeta_down_servo = theta[4]

    delta_sim_deg = simulate_servo(time_s, servo_cmd_deg, params_)

    mask = time_s > 0.2
    return (delta_sim_deg - delta_measured_deg)[mask]


def run_identification(time_s, servo_cmd_deg, delta_measured_deg, params_):
    theta0 = [
        params_.actuator.servo_delay,
        params_.actuator.wn_up_servo,
        params_.actuator.zeta_up_servo,
        params_.actuator.wn_down_servo,
        params_.actuator.zeta_down_servo,
    ]

    lower_bounds = [
        0.0,   # delay
        1.0,   # wn_up
        0.05,  # zeta_up
        1.0,   # wn_down
        0.05,  # zeta_down
    ]

    upper_bounds = [
        0.2,     # delay
        2000.0,  # wn_up
        5.0,     # zeta_up
        2000.0,  # wn_down
        5.0,     # zeta_down
    ]

    # Levenberg-Marquardt algoritem 
    # - Gradient descent + Newton-Raphson, Jacobian matrika: za oceno trenutnih parametrov, 
    # - lambda določa kateri parameter se koliko uporablja (večji lambda = bolj gradient descent, manjši lambda = bolj Newton-Raphson)
    return least_squares(
        residuals,
        theta0,
        bounds=(lower_bounds, upper_bounds),
        args=(time_s, servo_cmd_deg, delta_measured_deg, params_),
        verbose=2,
    )




# -------------------------------- MAIN --------------------------------
# Run code: python -m simulation.identification_servo
t = time - time[0]

if USE_MANUAL_SERVO_CMD:
    knot_time_s = np.array([0.0, 2.0, 5.0, 8.0, 12.0], dtype=float)
    knot_cmd_deg = np.array([0.0, 0.0, 10.0, -10.0, 0.0], dtype=float)
    servo_cmd_deg = stepwise_cmd_from_knots(t, knot_time_s, knot_cmd_deg)
else:
    servo_cmd_deg = servo_cmd_from_log

if RUN_IDENTIFICATION:
    result = run_identification(t, servo_cmd_deg, delta_measured_deg, params)
    print("\nOptimal parameters:")
    print(result.x)
    residuals(result.x, t, servo_cmd_deg, delta_measured_deg, params)

delta_sim_deg = simulate_servo(t, servo_cmd_deg, params)

plot_servo_response(t, delta_measured_deg, delta_sim_deg, servo_cmd_deg)

