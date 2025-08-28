# pip install numpy
import numpy as np  # 匯入數值運算模組，用於陣列與三角運算

# ====== 五連桿系統的幾何參數 ======
A = np.array([6.2500, 15.0000])     # 固定點 A 的座標
E = np.array([-6.2500, 15.0000])    # 固定點 E 的座標
L1 = 16.8200  # AB 連桿長度
L2 = 26.9250  # BC 連桿長度
L3 = 26.9250  # CD 連桿長度
L4 = 16.8200  # DE 連桿長度

# ====== forward 函式：根據角度計算 C 點可能位置 ======
def forward(theta1, theta2):
    """
    輸入：
        theta1：AB 的旋轉角度（以起始位置為 0，順時針為正）
        theta2：ED 的旋轉角度（以起始位置為 0，逆時針為正）
    回傳：
        C 點的兩個可能解座標 [(x1, y1), (x2, y2)]，每組為 tuple 格式，小數點四位
    """
    
    # 計算 B 點座標，從 A 點延伸 L1 長度，旋轉角度為 -theta1（因順時針為正）
    B = A + L1 * np.array([np.cos(-theta1), np.sin(-theta1)])
    
    # 計算 D 點座標，從 E 點延伸 L4 長度，旋轉角度為 theta2（逆時針為正）
    D = E + L4 * np.array([np.cos(theta2), np.sin(theta2)])

    # ====== 嵌套函式：計算兩圓交點 ======
    def circle_intersections(P1, r1, P2, r2):
        """
        給定兩圓（圓心 P1, P2 與半徑 r1, r2），回傳它們的交點（0、1 或 2 個）
        """
        d = np.linalg.norm(P2 - P1)  # 圓心距
        if d > r1 + r2 or d < abs(r1 - r2):
            return []  # 圓相離或包住對方，無交點

        a = (r1**2 - r2**2 + d**2) / (2 * d)  # 投影到圓心連線上的距離
        h = np.sqrt(max(r1**2 - a**2, 0))     # 高度：交點距離中點的垂直距離

        P3 = P1 + a * (P2 - P1) / d           # 基準交點（位於兩圓心連線上）
        offset = h * np.array([              # 垂直偏移向量，產生兩個交點
            -(P2[1] - P1[1]),
            P2[0] - P1[0]
        ]) / d

        # 回傳兩個交點座標，四位小數，並轉為 tuple 格式
        p1 = tuple(np.round(P3 + offset/100, 4).tolist())
        p2 = tuple(np.round(P3 - offset/100, 4).tolist())
        return [p1, p2]

    return circle_intersections(B, L2, D, L3)  # 呼叫圓交點解法，取得 C 點可能位置
    
print(forward(15, 45))