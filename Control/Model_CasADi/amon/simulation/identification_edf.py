from pathlib import Path
from scipy.optimize import least_squares

import numpy as np
from simulation.plotting import plot_edf_response
from simulation.simulator import simulate_edf
from models.parameters import AmonParams

params = AmonParams()

# load data - csv file
_AMON_DIR = Path(__file__).resolve().parents[1]
_DATA_PATH = _AMON_DIR / "data" / "edf_motor_characterization_20260420_step_newtons.csv"

data = np.genfromtxt(
    str(_DATA_PATH),
    delimiter=",",
    names=True,
    dtype=None,  
    encoding="utf-8",
)

time = data["pc_time_s"].astype(float)
T_measured = data["force1_N"].astype(float)

# --- Ročne EDF komande/rampa --------------------------------------
USE_MANUAL_EDF_CMD = False

# --- IDENTIFIKACIJA ------------------------------------------
RUN_IDENTIFICATION = True




def stepwise_cmd_from_knots(time_s: np.ndarray, knot_time_s: np.ndarray, knot_cmd_pct: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(knot_time_s, time_s, side="right") - 1
    idx = np.clip(idx, 0, len(knot_cmd_pct) - 1)
    return knot_cmd_pct[idx]

# Optimizacija parametrov EDF modela
def residuals(theta, time, edf_cmd, T_measured, params):

    # razpakiraj parametre
    params.actuator.edf_delay = theta[0]
    params.actuator.wn_up_edf = theta[1]
    params.actuator.zeta_up_edf = theta[2]
    params.actuator.wn_down_edf = theta[3]
    params.actuator.zeta_down_edf = theta[4]

    # simulacija
    T_sim = simulate_edf(time, edf_cmd, params)

    # ignoriraj prvih 0.2s (zaradi delay)
    mask = time > 0.2
    return (T_sim - T_measured)[mask]

def run_identification(time, edf_cmd, T_measured, params):

    theta0 = [
        params.actuator.edf_delay,
        params.actuator.wn_up_edf,
        params.actuator.zeta_up_edf,
        params.actuator.wn_down_edf,
        params.actuator.zeta_down_edf,
    ]

    lower_bounds = [
        0.0,    # delay
        1.0,    # wn_up
        0.1,    # zeta_up
        1.0,    # wn_down
        0.1     # zeta_down
    ]

    upper_bounds = [
        0.01,
        2000.0,
        5.0,
        2000.0,
        5.0
    ]

    result = least_squares(
        residuals,
        theta0,
        bounds=(lower_bounds, upper_bounds),
        args=(time, edf_cmd, T_measured, params),
        verbose=2,
        max_nfev=1000,
        loss="huber",
    )

    return result


# -------------------------------- MAIN --------------------------------
t = time - time[0]

if USE_MANUAL_EDF_CMD:

    # Ramp up - ramp down: 0 do 100% s korakom 5% vsakih 3s, nato nazaj do 0% s korakom 5% vsakih 3s
    test_time_s = np.array([
        0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0,
        30.0, 33.0, 36.0, 39.0, 42.0, 45.0, 48.0, 51.0, 54.0, 57.0,
        60.0, 63.0, 66.0, 69.0, 72.0, 75.0, 78.0, 81.0, 84.0, 87.0,
        90.0, 93.0, 96.0, 99.0, 102.0, 105.0, 108.0, 111.0, 114.0, 117.0,
        120.0
    ], dtype=float)

    test_cmd_pct = np.array([
        0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0,
        50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0,
        100.0, 95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 55.0,
        50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0, 15.0, 10.0, 5.0,
        0.0
    ], dtype=float)

    # # Step odziv: 0 - 20%, 20 - 60%
    # test_time_s = np.array([
    #     2.0, 5.0, 8.0, 10.0, 12.0
    # ], dtype=float)

    # test_cmd_pct = np.array([
    #     0, 20, 40, 60, 0
    # ], dtype=float)

    edf_cmd = stepwise_cmd_from_knots(t, test_time_s, test_cmd_pct)
else:
    edf_cmd = data["edf_power_percent"].astype(float)


# Optimizacija parametrov
if RUN_IDENTIFICATION:
    result = run_identification(t, edf_cmd, T_measured, params)

    print("\nOptimal parameters:")
    print(result.x)

    # posodobi params z optimalnimi vrednostmi
    residuals(result.x, t, edf_cmd, T_measured, params)



# simulate
T_sim = simulate_edf(t, edf_cmd, params)

# plot
plot_edf_response(t, T_measured, T_sim, edf_cmd)
