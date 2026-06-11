from casadi import *


# ============================================================
# Thrust vektor
# ============================================================
def thrust_force(T):
    """
    Gradnja CasADi thrust vektorja
    """
    return vertcat(0, 0, T)



# ============================================================
# Sila zaradi offseta motorja na CoM
# ============================================================
def thrust_torque_offset(T, params):
    """
    CasADi torque zaradi offseta motorja na CoM
    """
    F_T = thrust_force(T)
    r_T = params.physical.r_T
    return cross(r_T, F_T)



# ============================================================
# Sila na posameznem finu
# ============================================================
def single_fin_force(T, delta_i, n_i, k_i):
    """
    Sila na posameznem krilcu:
    F_i = T * k_i * sin(delta_i) * n_i
    Info:
        - sin(delta_i): nelinearna odvisnost sile od kota krilca (basic aerodynamics)
    """
    return T * k_i * sin(delta_i) * n_i



def fins_force(T, delta, params):
    """
    Skupna sila zaradi krilc
    """
    F_total = vertcat(0, 0, 0)

    for i in range(4):
        n_i = params.tvc.n_fins[:, i]
        k_i = params.tvc.k_fins[i]

        F_i = single_fin_force(T, delta[i], n_i, k_i)

        F_total = F_total + F_i

    return F_total



# ============================================================
# Moment zaradi finov
# ============================================================
def fins_torque(T, delta, params):
    """
    Moment zaradi krilc: tau_i = r_i x F_i
    """
    tau_total = vertcat(0, 0, 0)

    for i in range(4):
        r_i = params.tvc.r_fins[:, i]
        n_i = params.tvc.n_fins[:, i]
        k_i = params.tvc.k_fins[i]

        F_i = single_fin_force(T, delta[i], n_i, k_i)

        tau_i = cross(r_i, F_i)

        tau_total = tau_total + tau_i

    return tau_total



# ============================================================
# Skupna sila v body koordinatnem sistemu
# ============================================================
def total_force_body(T, delta, params):
    """
    Skupna sila v body koordinatnem sistemu
    """
    F_T = thrust_force(T)
    F_fins = fins_force(T, delta, params)

    f_0 = params.physical.f_0_T

    return F_T + F_fins + f_0



# ============================================================
# Skupni moment v body koordinatnem sistemu
# ============================================================
def total_torque_body(T, delta, params):
    """
    Skupni moment v body koordinatnem sistemu
    """
    tau_T = thrust_torque_offset(T, params)
    tau_fins = fins_torque(T, delta, params)

    tau_0 = params.physical.tau_0_T

    return tau_T + tau_fins + tau_0