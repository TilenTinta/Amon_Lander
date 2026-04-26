"""
Amon Lander - Model aktuatorjev

"""

import numpy as np
from collections import deque


# ============================================================
# Saturacija
# ============================================================
def saturate(value, value_min, value_max):
    return np.clip(value, value_min, value_max)


# ============================================================
# Stopinje -> radiani
# ============================================================
def deg_to_rad(angle_deg):
    return angle_deg * np.pi / 180.0


# ============================================================
# Lookup tabela - Linearna interpolacija
# ============================================================
def lookup_1d(x, x_table, y_table):
    """
    1D lookup (kot Simulink 1-D Lookup Table)
    """

    return np.interp(x, x_table, y_table)


# ============================================================
# EDF blok
# ============================================================
def edf_block(T, edf_cmd, params):
    """
    EDF blok

    Vhod:
    - T        : vzgon [N]
    - edf_cmd  : ukaz [%]

    Izhod:
    - T_dot
    - T_cmd
    """

    # Saturacija vhoda
    edf_cmd = saturate(edf_cmd, params.actuator.edf_min, params.actuator.edf_max)

    # Lookup tabela
    T_cmd = lookup_1d(edf_cmd, params.thrust.edf_table, params.thrust.thrust_table)

    # tau_edf_up in tau_edf_down sta lahko različna
    if T_cmd > T:
        tau = params.actuator.tau_edf_up
    else:
        tau = params.actuator.tau_edf_down

    # Dinamika 1. reda
    T_dot = (T_cmd - T) / tau

    # Dinamika 2. reda - mogoče???

    return T_dot, T_cmd



def edf_block_2nd_order(T, T_dot, edf_cmd, delay_buffer, params):
    """
    EDF blok - model 2. reda z zakasnitvijo

    Stanja:
    - T
    - T_dot

    Vhod:
    - edf_cmd [%]

    Izhodi:
    - dT
    - dT_dot
    - T_cmd
    - edf_cmd_delayed
    """

    # Saturacija vhoda
    edf_cmd = saturate(edf_cmd, params.actuator.edf_min, params.actuator.edf_max)

    # FIFO zakasnitev
    edf_cmd_delayed = delay_buffer.popleft()
    delay_buffer.append(edf_cmd)
    
    # Lookup tabela
    T_cmd = lookup_1d(edf_cmd_delayed, params.thrust.edf_table, params.thrust.thrust_table)

    # Parametri gor / dol
    if T_cmd > T:
        w_n = params.actuator.wn_up_edf
        zeta = params.actuator.zeta_up_edf
    else:
        w_n = params.actuator.wn_down_edf
        zeta = params.actuator.zeta_down_edf

    # 2. red
    dT = T_dot
    dT_dot = w_n**2 * (T_cmd - T) - 2.0 * zeta * w_n * T_dot

    #print(f"cmd={edf_cmd:.1f}, delayed={edf_cmd_delayed:.1f}, T_cmd={T_cmd:.2f}")

    return dT, dT_dot, T_cmd, edf_cmd_delayed


# ============================================================
# Servo blok (en servo)
# ============================================================
def servo_block(delta, servo_cmd_deg, servo_offset_rad, params):
    """
    Servo blok

    Vhod:
    - delta           : kot [rad]
    - servo_cmd_deg   : ukaz [deg]
    - servo_offset_rad: offset [rad]

    Izhod:
    - delta_dot
    - delta_cmd_rad
    """

    # Saturacija v stopinjah
    servo_cmd_deg = saturate(servo_cmd_deg, params.actuator.servo_min, params.actuator.servo_max)

    # Pretvorba v radiane
    delta_cmd_rad = deg_to_rad(servo_cmd_deg)

    # Dodaj offset
    delta_cmd_rad = delta_cmd_rad + servo_offset_rad

    # Dinamika 1. reda
    delta_dot = (delta_cmd_rad - delta) / params.actuator.tau_servo

    return delta_dot, delta_cmd_rad


def servo_block_2nd_order(delta, delta_dot, servo_cmd_deg, delay_buffer, params, servo_offset_rad=0.0):
    """
    Servo blok - model 2. reda z zakasnitvijo

    Stanja:
    - delta
    - delta_dot

    Vhod:
    - servo_cmd_deg [deg]

    Izhod:
    - ddelta
    - ddelta_dot
    - delta_cmd_rad
    - servo_cmd_deg_delayed
    """

    # Saturacija v stopinjah
    servo_cmd_deg = saturate(servo_cmd_deg, params.actuator.servo_min, params.actuator.servo_max)

    # FIFO zakasnitev
    servo_cmd_deg_delayed = delay_buffer.popleft()
    delay_buffer.append(servo_cmd_deg)

    # Pretvorba v radiane (delayed)
    delta_cmd_rad = deg_to_rad(servo_cmd_deg_delayed) + servo_offset_rad

    # Parametri gor / dol
    if delta_cmd_rad > delta:
        w_n = params.actuator.wn_up_servo
        zeta = params.actuator.zeta_up_servo
    else:
        w_n = params.actuator.wn_down_servo
        zeta = params.actuator.zeta_down_servo

    # 2. red
    ddelta = delta_dot
    ddelta_dot = w_n**2 * (delta_cmd_rad - delta) - 2.0 * zeta * w_n * delta_dot

    return ddelta, ddelta_dot, delta_cmd_rad, servo_cmd_deg_delayed


# ============================================================
# 4 servo bloki
# ============================================================
def servo_block_4(delta, servo_cmd, params):
    """
    4 servo bloki

    Vhod:
    - delta      : trenutni koti [rad]
    - servo_cmd  : ukazi [deg]

    Izhod:
    - delta_dot
    - delta_cmd_rad
    """

    delta_dot = np.zeros(4)
    delta_cmd_rad = np.zeros(4)

    offsets = params.actuator.delta_0

    for i in range(4):
        delta_dot[i], delta_cmd_rad[i] = servo_block(delta[i], servo_cmd[i], offsets[i], params)

    return delta_dot, delta_cmd_rad
