import sympy as sp
from sympy import sqrt, atan

x, y = sp.symbols('x y', real=True)

def sol3(x_val, y_val):
    try:
        num1 = 40*y + sqrt(-x**4 - 2*x**2*y**2 + 2258*x**2 - y**4 + 2258*y**2 - 108241)
        den1 = x**2 + 40*x + y**2 - 329
        num2 = 40*y - sqrt(-x**4 + 80*x**3 - 2*x**2*y**2 - 142*x**2 + 80*x*y**2 - 58320*x - y**4 + 1458*y**2 + 634959)
        den2 = x**2 - 80*x + y**2 + 871

        val1 = num1.subs({'x': x_val, 'y': y_val}).evalf()
        val2 = den1.subs({'x': x_val, 'y': y_val}).evalf()
        val3 = num2.subs({'x': x_val, 'y': y_val}).evalf()
        val4 = den2.subs({'x': x_val, 'y': y_val}).evalf()

        # 檢查是否為實數且非零分母
        if not (val1.is_real and val2.is_real and val4.is_real and val2 != 0 and val4 != 0):
            raise ValueError("Non-real or division by zero")

        t1 = 2 * sp.atan(val1 / val2)
        t2 = 2 * sp.atan(val3 / val4)
        return float(t1), float(t2)
    except Exception as e:
        print("Error in sol3:", e)
        return None, None

# 測試：
t1_rad, t2_rad = sol3(20, 40)
if t1_rad is not None:
    print(f"t1 = {sp.N(sp.deg(t1_rad)):.2f}°, t2 = {sp.N(sp.deg(t2_rad)):.2f}°")
