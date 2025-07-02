import sympy as sp

# 正向運動學：給定角度與長度，求 C 點（兩個構型）
def forward(theta1_deg, theta2_deg, l1, l2, l3, l4):
    # 角度轉弧度，考慮旋轉方向
    theta1 = -theta1_deg * sp.pi / 180             # A 點，順時針
    theta2 = (180 + theta2_deg) * sp.pi / 180      # E 點，左水平方向起點，逆時針旋轉

    # 固定點座標
    Ax, Ay = 0, 0
    Ex, Ey = -50, 0

    # B, D 點位置
    Bx = Ax + l1 * sp.cos(theta1)
    By = Ay + l1 * sp.sin(theta1)
    Dx = Ex + l4 * sp.cos(theta2)
    Dy = Ey + l4 * sp.sin(theta2)

    # 向量 BD
    vec_x = Dx - Bx
    vec_y = Dy - By
    d = sp.sqrt(vec_x**2 + vec_y**2)

    # 中點 M
    Mx = (Bx + Dx) / 2
    My = (By + Dy) / 2

    # 高（從 M 垂直方向偏移）
    try:
        h = sp.sqrt(l2**2 - (d / 2)**2)
    except:
        raise ValueError("構型不可行：連桿無法形成三角形")

    # 單位向量 u = BD / ||BD||
    ux = vec_x / d
    uy = vec_y / d

    # 垂直向量（旋轉 90°）
    vx = -uy
    vy = ux

    # C 點 1（上構型）
    C1x = Mx + h * vx
    C1y = My + h * vy

    # C 點 2（下構型）
    C2x = Mx - h * vx
    C2y = My - h * vy

    print("B 點座標:", float(Bx.evalf()), float(By.evalf()))
    print("D 點座標:", float(Dx.evalf()), float(Dy.evalf()))
    print("C 點 1（上構型）:", float(C1x.evalf()), float(C1y.evalf()))
    print("C 點 2（下構型）:", float(C2x.evalf()), float(C2y.evalf()))

    return (float(C1x.evalf()), float(C1y.evalf())), (float(C2x.evalf()), float(C2y.evalf()))
    
print(forward(30, 45, 67.27, 110, 110, 67.27))