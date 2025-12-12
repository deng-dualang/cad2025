# pip install sympy
import sympy as sp

# 定義符號
x, y = sp.symbols('x y', real=True)
L1 = L4 = 20  # link1 連桿長度
L2 = L3 = 27
B = 20  # 馬達間距

# 定義角度符號
'''
定義左馬達與右馬達的角度為符號變數：t1 與 t2（角度為徑度制，從正 X 軸起算，逆時針為正）
'''
t1, t2 = sp.symbols('t1 t2', real=True)

# 左馬達在 (0,0)
# 左側第一段連桿末端點的位置：A = (L*cos(t1), L*sin(t1))
# 第二段連桿起端點(亦即 A 點)與末端點 (x, y) 的距離應為 L：|A - (x, y)| = L

eq1 = sp.Eq((L1*sp.cos(t1) - x)**2 + (L1*sp.sin(t1) - y)**2, L2**2)

# 右馬達在 (B, 0)
# 右側第一段連桿末端點的位置：B = (B - L*cos(t2), L*sin(t2))
# 第二段連桿末端點(亦即 B 點)與末端點 (x, y) 的距離也應為 L：|B - (x, y)| = L

eq2 = sp.Eq((B - L1*sp.cos(t2) - x)**2 + (L1*sp.sin(t2) - y)**2, L2**2)

# 解這兩個方程組（注意，可能會有四組解）
solutions = sp.solve([eq1, eq2], (t1, t2), dict=True)

# 求出的 solutions 帶入 (20, 40) 選擇 t1 and t2 都小於 90 度的解
# 且將 solutions 轉為 def motor(x, y) 然後傳回 t1 and t2 in degree