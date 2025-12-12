import sympy as sp

# 先定義符號
x, y = sp.symbols('x y')

# 定義 t1 與 t2 的符號表達式
t1_expr = 2*sp.atan((40*y + sp.sqrt(-x**4 - 2*x**2*y**2 + 2258*x**2 - y**4 + 2258*y**2 - 108241)) / (x**2 + 40*x + y**2 - 329))
t2_expr = 2*sp.atan((40*y - sp.sqrt(-x**4 + 80*x**3 - 2*x**2*y**2 - 142*x**2 + 80*x*y**2 - 58320*x - y**4 + 1458*y**2 + 634959)) / (x**2 - 80*x + y**2 + 871))

# 用 lambdify 轉成可用於純 python 的函式
f_t1 = sp.lambdify((x, y), t1_expr, modules='math')
f_t2 = sp.lambdify((x, y), t2_expr, modules='math')

# 定義 sol3 函式
def sol3(x_val, y_val):
    t1 = f_t1(x_val, y_val)
    t2 = f_t2(x_val, y_val)
    return t1, t2

# 呼叫 sol3(20, 40)
t1_rad, t2_rad = sol3(20, 40)

# 印出角度（度數）
import math
print(f"t1 = {math.degrees(t1_rad):.2f}°")
print(f"t2 = {math.degrees(t2_rad):.2f}°")
