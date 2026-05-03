from casadi import MX, SX, DM


def is_symbolic(x):
    return isinstance(x, (MX, SX))


def to_dm(x):
    return DM(x)