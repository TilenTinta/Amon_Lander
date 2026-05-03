from casadi import MX, Function
from models_casadi.core.math import saturate, deg_to_rad, lookup_1d

# ============================================================
# TEST 1: saturate
# ============================================================
x = MX.sym("x")

y = saturate(x, 0, 10)
f = Function("f", [x], [y])

print("saturate test:")
print(f(15))   # out: 10
print(f(-5))   # out: 0
print(f(5))    # out: 5


# ============================================================
# TEST 2: deg_to_rad
# ============================================================
y = deg_to_rad(x)
f = Function("deg2rad", [x], [y])

print("\ndeg_to_rad test:")
print(f(180))  # ~3.1415


# ============================================================
# TEST 3: lookup_1d
# ============================================================
x_table = [0, 50, 100]
y_table = [0, 10, 20]

y = lookup_1d(x, x_table, y_table)
f = Function("lookup", [x], [y])

print("\nlookup test:")
print(f(25))   # ~5
print(f(75))   # ~15
print(f(150))  # 20 (clamped)