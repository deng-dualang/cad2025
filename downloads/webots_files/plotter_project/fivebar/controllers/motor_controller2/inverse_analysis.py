# pip install sympy
import sympy as sp

# ===== 定義符號變數 =====
Cx, Cy = sp.symbols('Cx Cy', real=True)              # C 點的輸入座標
Bx, By, Dx, Dy = sp.symbols('Bx By Dx Dy', real=True)  # 待解出的 B 與 D 點
theta1, theta2 = sp.symbols('theta1 theta2', real=True)  # 欲求解的角度變數（弧度）

# ===== 幾何參數 =====
Ax, Ay = 6.2500, 15.0000
Ex, Ey = -6.2500, 15.0000

L1 = 16.8200  # AB
L2 = 26.9250  # BC
L3 = 26.9250  # CD
L4 = 16.8200  # DE

# 固定點與座標
A = sp.Matrix([Ax, Ay])
E = sp.Matrix([Ex, Ey])
C = sp.Matrix([Cx, Cy])
B = sp.Matrix([Bx, By])
D = sp.Matrix([Dx, Dy])

# ===== C 為圓心，與 B 的距離 = L2（link2）=====
eq_CB = (C - B).dot(C - B) - L2**2

# ===== C 為圓心，與 D 的距離 = L3（link3）=====
eq_CD = (C - D).dot(C - D) - L3**2

# ===== A 為圓心，B 為外點，距離 = L1（link1）=====
eq_AB = (B - A).dot(B - A) - L1**2

# ===== E 為圓心，D 為外點，距離 = L4（link4）=====
eq_DE = (D - E).dot(D - E) - L4**2

# ===== inverse 函式 =====
def inverse_kinematics(cx_val, cy_val):
    subs = {Cx: cx_val, Cy: cy_val}
    
    # 將數值代入 C 座標，解出 B 與 D 的所有交點（實數解）
    solutions = sp.solve([
        eq_CB.subs(subs),
        eq_CD.subs(subs),
        eq_AB,
        eq_DE
    ], (Bx, By, Dx, Dy), dict=True)
    
    results = []
    for sol in solutions:
        if all(sol[var].is_real for var in [Bx, By, Dx, Dy]):
            Bval = sp.Matrix([sol[Bx], sol[By]])
            Dval = sp.Matrix([sol[Dx], sol[Dy]])
            # 從 A → B 向量，算出 theta1（AB 的旋轉角）
            v1 = Bval - A
            theta1_calc = sp.atan2(v1[1], v1[0])
            # 從 E → D 向量，算出 theta2（ED 的旋轉角）
            v2 = Dval - E
            theta2_calc = sp.atan2(v2[1], v2[0])
            # 轉換為角度並保留小數點後 4 位
            deg1 = round(sp.deg(theta1_calc.evalf()), 4)
            deg2 = round(sp.deg(theta2_calc.evalf()), 4)
            pair = (deg1, deg2)
            if pair not in results:
                results.append(pair)
    
    return results

# ===== 驗證區 =====
if __name__ == "__main__":
    print("🧪 測試 inverse_kinematics(C = (0.0, 0.0))")
    result = inverse_kinematics(0.0, 0.0)
    for i, (t1, t2) in enumerate(result, 1):
        print(f"解 #{i}: θ1 = {t1:.4f}°, θ2 = {t2:.4f}°")