from casadi import *
from models_casadi.core.math import saturate, lookup_1d, deg_to_rad


def edf_block_instant(edf_cmd, params):
    """
    EDF brez dinamike (instant)
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    edf_cmd = saturate(edf_cmd, params.actuator.edf_min, params.actuator.edf_max)

    # --------------------------------------------------------
    # Lookup
    # --------------------------------------------------------
    T_cmd = lookup_1d(edf_cmd, params.thrust.edf_table, params.thrust.thrust_table)

    return T_cmd



def edf_block_1st_order(T, edf_cmd, params):
    """
    CasADi EDF model (1. red)
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    edf_cmd = saturate(edf_cmd, params.actuator.edf_min, params.actuator.edf_max)

    # --------------------------------------------------------
    # Lookup
    # --------------------------------------------------------
    T_cmd = lookup_1d(edf_cmd, params.thrust.edf_table, params.thrust.thrust_table)

    # --------------------------------------------------------
    # Dinamika 1. reda
    # --------------------------------------------------------
    tau_up = params.actuator.tau_edf_up
    tau_down = params.actuator.tau_edf_down
    tau = if_else(T_cmd > T, tau_up, tau_down)
    dT = (T_cmd - T) / tau

    return dT, T_cmd



def edf_block_2nd_order(T, T_dot, edf_cmd, params):
    """
    CasADi EDF model (2. red)
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    edf_cmd = saturate(edf_cmd, params.actuator.edf_min, params.actuator.edf_max)

    # --------------------------------------------------------
    # Lookup
    # --------------------------------------------------------
    T_cmd = lookup_1d(edf_cmd, params.thrust.edf_table, params.thrust.thrust_table)

    # --------------------------------------------------------
    # Parametri (up/down)
    # --------------------------------------------------------
    wn_up = params.actuator.wn_up_edf
    zeta_up = params.actuator.zeta_up_edf

    wn_down = params.actuator.wn_down_edf
    zeta_down = params.actuator.zeta_down_edf

    wn = if_else(T_cmd > T, wn_up, wn_down)
    zeta = if_else(T_cmd > T, zeta_up, zeta_down)

    # --------------------------------------------------------
    # Dinamika 2. reda
    # --------------------------------------------------------
    dT = T_dot

    dT_dot = wn**2 * (T_cmd - T) - 2 * zeta * wn * T_dot

    return dT, dT_dot, T_cmd



def servo_block_instant(servo_cmd_deg, params, servo_offset_rad=0.0):
    """
    Servo brez dinamike
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    servo_cmd_deg = saturate(servo_cmd_deg, params.actuator.servo_min, params.actuator.servo_max)

    # --------------------------------------------------------
    # Deg -> rad + offset
    # --------------------------------------------------------
    delta_cmd = deg_to_rad(servo_cmd_deg) + servo_offset_rad

    return delta_cmd



def servo_block_1st_order(delta, servo_cmd_deg, params, servo_offset_rad=0.0):
    """
    CasADi servo model (1. red)
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    servo_cmd_deg = saturate(servo_cmd_deg, params.actuator.servo_min, params.actuator.servo_max)

    # --------------------------------------------------------
    # Deg -> rad + offset
    # --------------------------------------------------------
    delta_cmd = deg_to_rad(servo_cmd_deg) + servo_offset_rad

    # --------------------------------------------------------
    # Dinamika 1. reda
    # --------------------------------------------------------
    tau = params.actuator.tau_servo
    ddelta = (delta_cmd - delta) / tau

    return ddelta, delta_cmd



def servo_block_2nd_order(delta, delta_dot, servo_cmd_deg, params, servo_offset_rad=0.0):
    """
    CasADi servo model (2. red, brez delay)
    """

    # --------------------------------------------------------
    # Saturacija
    # --------------------------------------------------------
    servo_cmd_deg = saturate(servo_cmd_deg, params.actuator.servo_min, params.actuator.servo_max)

    # --------------------------------------------------------
    # Deg -> rad + offset
    # --------------------------------------------------------
    delta_cmd = deg_to_rad(servo_cmd_deg) + servo_offset_rad

    # --------------------------------------------------------
    # Parametri (up/down)
    # --------------------------------------------------------
    wn_up = params.actuator.wn_up_servo
    zeta_up = params.actuator.zeta_up_servo

    wn_down = params.actuator.wn_down_servo
    zeta_down = params.actuator.zeta_down_servo

    wn = if_else(delta_cmd > delta, wn_up, wn_down)
    zeta = if_else(delta_cmd > delta, zeta_up, zeta_down)

    # --------------------------------------------------------
    # Dinamika 2. reda
    # --------------------------------------------------------
    ddelta = delta_dot

    ddelta_dot = wn**2 * (delta_cmd - delta) - 2 * zeta * wn * delta_dot

    return ddelta, ddelta_dot, delta_cmd



def servo_block_4_instant(servo_cmd_deg, params):
    """
    4 servo modeli brez dinamike
    """

    delta_cmd = []

    offsets = params.actuator.delta_0

    for i in range(4):
        cmd = servo_block_instant(
            servo_cmd_deg[i],
            params,
            servo_offset_rad=offsets[i]
        )

        delta_cmd.append(cmd)

    return vertcat(*delta_cmd)



def servo_block_4_1st_order(delta, servo_cmd_deg, params):
    """
    4 servo modeli (1. red)
    """

    ddelta = []
    delta_cmd = []

    offsets = params.delta_0

    for i in range(4):
        d, cmd = servo_block_1st_order(
            delta[i],
            servo_cmd_deg[i],
            params,
            servo_offset_rad=offsets[i]
        )

        ddelta.append(d)
        delta_cmd.append(cmd)

    return vertcat(*ddelta), vertcat(*delta_cmd)



def servo_block_4_2nd_order(delta, delta_dot, servo_cmd_deg, params):
    """
    4 servo modeli (2. red)
    """

    ddelta = []
    ddelta_dot = []
    delta_cmd = []

    #offsets = params.actuator.delta_0
    offsets = params.delta_0

    for i in range(4):
        d, d_dot, cmd = servo_block_2nd_order(
            delta[i],
            delta_dot[i],
            servo_cmd_deg[i],
            params,
            servo_offset_rad=offsets[i]
        )

        ddelta.append(d)
        ddelta_dot.append(d_dot)
        delta_cmd.append(cmd)

    # CasADi vector (np.zeros(4))
    return vertcat(*ddelta), vertcat(*ddelta_dot), vertcat(*delta_cmd)