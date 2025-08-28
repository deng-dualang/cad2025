import sympy as sp

def inverse(cx_val, cy_val, l1_val, l2_val, l3_val, l4_val):
    # 定義符號變數
    Cx, Cy, L1, L2, L3, L4 = sp.symbols('Cx Cy L1 L2 L3 L4', real=True)
 
    # 固定點 A 與 E
    Ax, Ay = 0.0625, 0.15
    Ex, Ey = -0.0625, 0.15
 
    # ========== θ₁（順時針，從 +X 起算） ==========
    vec_A = sp.Matrix([Cx - Ax, Cy - Ay])
    r1 = vec_A.norm()
    angle_A = sp.atan2(vec_A[1], vec_A[0])
    cos_alpha1 = (L1**2 + r1**2 - L2**2) / (2 * L1 * r1)
    cos_alpha1 = sp.Max(-1, sp.Min(1, cos_alpha1))  # 保護 acos 領域
    alpha1 = sp.acos(cos_alpha1)
    theta1_a = -sp.deg(angle_A - alpha1)
    theta1_b = -sp.deg(angle_A + alpha1)
 
    # ========== θ₂（逆時針，從 –X 起算） ==========
    vec_E = sp.Matrix([Cx - Ex, Cy - Ey])
    r2 = vec_E.norm()
    angle_E = sp.atan2(vec_E[1], vec_E[0])
    cos_alpha2 = (L4**2 + r2**2 - L3**2) / (2 * L4 * r2)
    cos_alpha2 = sp.Max(-1, sp.Min(1, cos_alpha2))  # 保護 acos 領域
    alpha2 = sp.acos(cos_alpha2)
    # 補回 forward 中的 +180°
    theta2_a = sp.deg(angle_E - alpha2) - 180
    theta2_b = sp.deg(angle_E + alpha2) - 180
 
    # 數值代入
    subs = {
        Cx: cx_val,
        Cy: cy_val,
        L1: l1_val,
        L2: l2_val,
        L3: l3_val,
        L4: l4_val
    }
 
    try:
        θ1a = float(theta1_a.evalf(subs=subs))
        θ1b = float(theta1_b.evalf(subs=subs))
        θ2a = float(theta2_a.evalf(subs=subs))
        θ2b = float(theta2_b.evalf(subs=subs))
    except Exception as e:
        return [f"發生錯誤：{e}"]
         
    def normalize(a):
        return round(a % 360, 4)
 
 
    # 組合所有可能構型角度
    results = [
        (normalize(θ1a), normalize(θ2a)),
        (normalize(θ1a), normalize(θ2b)),
        (normalize(θ1b), normalize(θ2a)),
        (normalize(θ1b), normalize(θ2b)),
    ]
 
    #return results
    return normalize(θ1b), normalize(θ2a)

scale = 1 / 400.0
L1 = L4 = 67.27 * scale
L2 = L3 = 110 * scale

print(inverse(-0.073, -0.117, L1, L2, L3, L4))