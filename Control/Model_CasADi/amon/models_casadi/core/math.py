from casadi import *

# ============================================================
# Saturacija (clip)
# ============================================================
def saturate(x, xmin, xmax):
    """
    CasADi saturacija
    - Info:
        uporablja fmin/fmax (Python min/max)
    """
    return fmin(fmax(x, xmin), xmax)


# ============================================================
# Stopinje -> radiani
# ============================================================
def deg_to_rad(angle_deg):
    return angle_deg * pi / 180.0


# ============================================================
# 1D lookup tabela (linear interpolation)
# ============================================================
def lookup_1d(x, x_table, y_table):
    """
    CasADi verzija np.interp

    x_table, y_table = list ali DM
    """

    # pretvori v CasADi DM (Casadi verzija Numpy array-a)
    x_table = DM(x_table)
    y_table = DM(y_table)

    y = 0

    # linear interpolation segment-wise
    for i in range(x_table.size1() - 1):
        x0 = x_table[i]
        x1 = x_table[i + 1]
        y0 = y_table[i]
        y1 = y_table[i + 1]

        # linear interpolation
        slope = (y1 - y0) / (x1 - x0)
        y_lin = y0 + slope * (x - x0)

        # casadi if_else
        cond = logic_and(x >= x0, x <= x1)

        y = if_else(cond, y_lin, y)

    # ektrapolacija (robovi)
    y = if_else(x < x_table[0], y_table[0], y)
    y = if_else(x > x_table[-1], y_table[-1], y)

    return y