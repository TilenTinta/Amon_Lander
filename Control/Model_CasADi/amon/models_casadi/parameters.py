from casadi import *


# ============================================================
# STATE INDEXES (CASADI)
# ============================================================

X_P = slice(0, 3)
X_V = slice(3, 6)
X_Q = slice(6, 10)
X_W = slice(10, 13)

X_T = 13
X_T_DOT = 14

X_DELTA = slice(15, 19)
X_DELTA_DOT = slice(19, 23)

NX = 23
NU = 5


# ============================================================
# PARAMETERS CLASS
# ============================================================

class AmonParamsCasadi:
    def __init__(self, params_numpy):

        # ---------------------------
        # PHYSICAL
        # ---------------------------
        self.physical = params_numpy.physical

        # CasADi DM conversion
        self.physical.J = DM(self.physical.J)
        self.physical.r_T = DM(self.physical.r_T)
        self.physical.f_0_T = DM(self.physical.f_0_T)
        self.physical.tau_0_T = DM(self.physical.tau_0_T)

        # ---------------------------
        # ACTUATORS
        # ---------------------------
        self.actuator = params_numpy.actuator

        #self.actuator.delta_0 = DM(self.actuator.delta_0)
        self.delta_0 = DM(params_numpy.actuator.delta_0)

        # ---------------------------
        # THRUST TABLE
        # ---------------------------
        self.thrust = params_numpy.thrust

        self.thrust.edf_table = DM(self.thrust.edf_table)
        self.thrust.thrust_table = DM(self.thrust.thrust_table)

        # ---------------------------
        # TVC
        # ---------------------------
        self.tvc = params_numpy.tvc

        self.tvc.r_fins = DM(self.tvc.r_fins)
        self.tvc.n_fins = DM(self.tvc.n_fins)
        self.tvc.k_fins = DM(self.tvc.k_fins)