"""
Amon Lander - Parametri modela

Glavna referenca:
- amon_model.m
- Simulink implementacija (prememba programa)
"""

import numpy as np
from dataclasses import dataclass, field


# ============================================================
# Razširjen vektor stanj
# x = [p, v, q, omega, T, delta]
# ============================================================
X_P = slice(0, 3)         # pozicija
X_V = slice(3, 6)         # hitrost
X_Q = slice(6, 10)        # kvaternion
X_W = slice(10, 13)       # kotna hitrost
X_T = 13                  # vzgon EDF
X_DELTA = slice(14, 18)   # koti finov

X_STANJ = 18


# ============================================================
# Vhodni vektor
# u = [edf_cmd, servo1_cmd, servo2_cmd, servo3_cmd, servo4_cmd]
# ============================================================
U_EDF = 0
U_SERVO = slice(1, 5)

U_STANJ = 5


# ============================================================
# Osnovne konstante
# ============================================================
G = 9.81    # gravitacija


# ============================================================
# Fizični parametri
# ============================================================
@dataclass
class PhysicalParams:
    # Gravitacija
    g: float = 9.81

    # Zunanja sila / motnja - je ni / zanemarimo
    f_dist: float = 0.0

    # Zračni upor - ga ni / zanemarimo
    K_d: float = 0.0

    # Masa [g] (pazi na baterije katere uporabljaš)
    mass_dron: float = 1915.0
    mass_battery: float = 573.0
    mass: float = mass_dron + mass_battery

    # Matrika vztrajnosti
    # Vpliva na pospešek - koliko močno fin “prime zrak”
    J: np.ndarray = field(default_factory=lambda: np.array([
        [0.02,  0.0,   0.0],
        [0.0,   0.018, 0.0],
        [0.0,   0.0,   0.03]
    ], dtype=float))

    # OptiTrack marker odmik
    r_mb: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 0.2
    ], dtype=float))

    # Offset EDF-ja iz CoM
    r_T: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 0.0
    ], dtype=float))

    # Default sila EDF-ja na drona
    f_0_T: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 0.0
    ], dtype=float))

    # Default navor EDF-ja na drona
    tau_0_T: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 0.0
    ], dtype=float))

    # Yaw koeficient
    c_yaw: float = 0.0

    # Skupna zakasnitev med ukazom in efektom
    t_d: float = 0.0


# ============================================================
# Parametri aktuatorjev
# ============================================================
@dataclass
class ActuatorParams:
    # Zakasnitev EDF-ja
    tau_edf_up: float = 0.003
    tau_edf_down: float = 0.003

    # NMPC dt = 10-20ms: delay je 0.05–0.1 koraka -> zanemarljivo
    edf_delay: float = 0.001

    wn_up_edf    = 661.8   # narava frekvenca: rad/s - kako hitro EDF želi doseči cilj
    zeta_up_edf  = 1.263   # koeficient dušenja: 1.0 = kritično dušenje, >1 = predušenje, <1 = nedušenje
    wn_down_edf  = 695.6   
    zeta_down_edf = 0.870

    # Zakasnitev servota
    tau_servo: float = 0.0001

    # NMPC dt = 10-20ms: delay je 0.05–0.1 koraka -> zanemarljivo
    servo_delay: float = 0.0005

    wn_up_servo: float = 1335.0   # narava frekvenca: rad/s - kako hitro servo želi doseči cilj
    zeta_up_servo: float = 0.95   # koeficient dušenja: 1.0 = kritično dušenje, >1 = predušenje, <1 = nedušenje
    wn_down_servo: float = 1122.5
    zeta_down_servo: float = 0.916

    # Omejitve EDF [%]
    edf_min: float = 0.0
    edf_max: float = 100.0

    # Omejitve servota [deg]
    servo_min: float = -45.0
    servo_max: float = 45.0

    # Offseti servotov [rad]
    servo_1_offset: float = 0.0
    servo_2_offset: float = 0.0
    servo_3_offset: float = 0.0
    servo_4_offset: float = 0.0

    @property
    def delta_0(self) -> np.ndarray:
        return np.array([
            self.servo_1_offset,
            self.servo_2_offset,
            self.servo_3_offset,
            self.servo_4_offset
        ], dtype=float)


# ============================================================
# Parametri EDF lookup tabele
# ============================================================
@dataclass
class ThrustParams:
    # Lookup tabela za EDF [%]
    edf_table: np.ndarray = field(default_factory=lambda: np.array([
        0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
    ], dtype=float))

    # Lookup tabela za vzgon [N] - uporaba(edf_motor_characterization_20260419_2_newtons.csv, edf_motor_characterization_20260420_step_newtons.csv)
    thrust_table: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 1.043400058293994, 3.307305432039027, 3.3032399923486793, 5.941793085683571, 5.941138774261509, 8.097211792203785, 10.40435511895069, 10.439941638979901, 12.464797142140686, 14.583097941968916, 14.478891227741295, 16.971861748524187, 16.796433637071743, 20.248446399869124, 23.439424199120122, 23.091672689642436, 26.09097093181935, 25.84693293930825, 25.76864154101192
        #0.09306760657162888, 0.08615986556555269, 1.043400058293994, 3.307305432039027, 3.3032399923486793, 5.941793085683571, 5.941138774261509, 8.097211792203785, 10.40435511895069, 10.439941638979901, 12.464797142140686, 14.583097941968916, 14.478891227741295, 16.971861748524187, 16.796433637071743, 20.248446399869124, 23.439424199120122, 23.091672689642436, 26.09097093181935, 25.84693293930825, 25.76864154101192
    ], dtype=float))

    # Lookup tabela za navor EDF [Nm]
    torque_table: np.ndarray = field(default_factory=lambda: np.array([
        0.0, 0.0, 0.0, 0.0, 0.0
    ], dtype=float))


# ============================================================
# Parametri finov in TVC
# ============================================================
@dataclass
class TVCParams:
    # Položaj finov glede na CoM
    # Zaporedje: x+, x-, y+, y-
    r_fins: np.ndarray = field(default_factory=lambda: np.array([
        [ 0.05,  -0.05,   0.0,    0.0  ],
        [ 0.0,    0.0,    0.05,  -0.05 ],
        [-0.115, -0.115, -0.115, -0.115]
    ], dtype=float))

    # Smerni vektorji sil pri pozitivnem odklonu servota
    # Zaporedje: x+, x-, y+, y-
    n_fins: np.ndarray = field(default_factory=lambda: np.array([
        [0.0,  0.0,  1.0, -1.0],
        [1.0, -1.0,  0.0,  0.0],
        [0.0,  0.0,  0.0,  0.0]
    ], dtype=float))

    # Koeficienti učinkovitosti finov
    # Vpliva na amplitudo - kako težko je dron zavrteti
    k_fins: np.ndarray = field(default_factory=lambda: np.array([
        1.0, 1.0, 1.0, 1.0
    ], dtype=float))

    # Matrika sil
    B_F: np.ndarray = field(init=False)

    # Matrika momentov
    B_tau: np.ndarray = field(init=False)

    def __post_init__(self):
        self.B_F = np.zeros((3, 4), dtype=float)
        self.B_tau = np.zeros((3, 4), dtype=float)

        for i in range(4):
            self.B_F[:, i] = self.k_fins[i] * self.n_fins[:, i]
            self.B_tau[:, i] = np.cross(self.r_fins[:, i], self.B_F[:, i])


# ============================================================
# Združeni parametri modela
# ============================================================
@dataclass
class AmonParams:
    physical: PhysicalParams = field(default_factory=PhysicalParams)
    actuator: ActuatorParams = field(default_factory=ActuatorParams)
    thrust: ThrustParams = field(default_factory=ThrustParams)
    tvc: TVCParams = field(default_factory=TVCParams)


# ============================================================
# Začetno stanje
# ============================================================
def default_state() -> np.ndarray:
    x = np.zeros(X_STANJ, dtype=float)

    # Enotski kvaternion
    x[X_Q] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    return x


# ============================================================
# Default vhod
# ============================================================
def default_input() -> np.ndarray:
    u = np.zeros(U_STANJ, dtype=float)
    return u
