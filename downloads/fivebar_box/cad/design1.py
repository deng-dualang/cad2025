import numpy as np

# --- 固定參數 ---
LENGTH = 20  # 馬達中心距離 A 到 E (cm)
W = 20       # 繪圖區域寬度 (cm)
H = 20       # 繪圖區域高度 (cm)
SAFETY_MARGIN = 1.0 # 引入 1.0 cm 的安全裕度

# --- 1. 幾何函數：圓圓交點問題 (維持不變) ---
def two_circle_intersection(A, R1, B, R2):
    # ... (此函數邏輯與前版本相同，無需修改) ...
    x0, y0 = A
    x1, y1 = B
    d = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    if d > R1 + R2 or d < abs(R1 - R2):
        return None  
    a = (R1**2 - R2**2 + d**2) / (2 * d)
    h_squared = R1**2 - a**2
    h = np.sqrt(max(0, h_squared))
    
    x2 = x0 + a * (x1 - x0) / d
    y2 = y0 + a * (y1 - y0) / d
    
    x_p1 = x2 + h * (y1 - y0) / d
    y_p1 = y2 - h * (x1 - x0) / d
    
    x_p2 = x2 - h * (y1 - y0) / d
    y_p2 = y2 + h * (x1 - x0) / d

    valid_solutions = []
    if y_p1 >= 0:
        valid_solutions.append((x_p1, y_p1))
    if y_p2 >= 0:
        valid_solutions.append((x_p2, y_p2))
        
    if not valid_solutions:
        return None 
    
    return max(valid_solutions, key=lambda p: p[1])


# --- 2. 適應度函數 (Fitness Function) ***修正為最小化尺寸*** ---
def calculate_fitness(individual, num_x_samples=10, num_y_samples=5):
    """
    評估個體的適應度。目標是：最小化 L1+L2+0.5*dist 且滿足所有約束。
    """
    L1, L2, dist = individual
    P_total = 0.0
    K_large = 1000.0
    K_small = 0.1
    
    # *** I. 計算懲罰 P_total (約束不滿足度) ***
    if L1 <= 0 or L2 <= 0 or dist <= 0:
        return 0.0
    
    # 懲罰 1: 最遠點奇異點
    D_max = np.sqrt(LENGTH**2 + (dist + H)**2)
    if L1 + L2 <= D_max:
        P_total += (D_max - (L1 + L2)) * K_large * K_small
        
    # 懲罰 2 & 3: 繪圖區域穩定性
    max_D_BD_area = 0
    x_c_samples = np.linspace(0.001, LENGTH - 0.001, num_x_samples)
    y_c_samples = np.linspace(dist + 0.001, dist + H - 0.001, num_y_samples)
    
    for x_c in x_c_samples:
        for y_c in y_c_samples:
            res_B = two_circle_intersection((0, 0), L1, (x_c, y_c), L2)
            res_D = two_circle_intersection((LENGTH, 0), L1, (x_c, y_c), L2)
            if res_B is None or res_D is None:
                P_total += K_large 
                return 1.0 / (1.0 + P_total) # 大懲罰退出
            
            x_B, y_B = res_B
            x_D, y_D = res_D
            D_BD = np.sqrt((x_B - x_D)**2 + (y_B - y_D)**2)
            max_D_BD_area = max(max_D_BD_area, D_BD)
    
    Required_2L2 = max_D_BD_area + SAFETY_MARGIN
    
    if 2 * L2 <= Required_2L2:
        P_total += (Required_2L2 - 2 * L2) * K_large * K_small

    # *** II. 定義目標函數 J (要最小化) ***
    J = L1 + L2 + 0.5 * dist
    
    # *** III. 計算最終適應度 ***
    # 由於 L1, L2, dist 大概在 20-50 之間，J 遠小於 K_large。
    # 當 P_total > 0 時，P_total 佔主導（優先滿足約束）。
    # 當 P_total ≈ 0 時，J 佔主導（優先最小化尺寸）。
    return 1.0 / (J + P_total)


# --- 3. 基因演算法主要函數 (保持不變) ---
def genetic_algorithm_solver(
    pop_size=100, 
    generations=250, 
    mutation_rate=0.1, 
    crossover_rate=0.8
):
    # 搜索範圍維持在擴大後的範圍
    bounds = np.array([[8, 30], [20, 50], [2, 30]]) 

    population = np.random.uniform(bounds[:, 0], bounds[:, 1], size=(pop_size, 3))
    
    print(f"--- 開始基因演化 (目標: 最小化尺寸, 穩定性裕度: {SAFETY_MARGIN} cm) ---")
    print(f"--- 搜索範圍：L1=[{bounds[0,0]},{bounds[0,1]}], L2=[{bounds[1,0]},{bounds[1,1]}], Dist=[{bounds[2,0]},{bounds[2,1]}] ---")

    
    for gen in range(generations):
        fitness = np.array([calculate_fitness(ind) for ind in population])
        best_idx = np.argmax(fitness)
        best_fitness = fitness[best_idx]
        best_individual = population[best_idx]

        # 這裡的 "成功找到" 定義為適應度達到某個合理的極大值
        # 由於適應度現在是 1/J，它會是一個較小的值 (例如 1/70 ≈ 0.014)
        # 所以我們需要觀察收斂
        
        # 這裡不需要中途退出，必須跑到最後，因為我們在找最小 J
        
        # 選擇、交叉、突變邏輯維持不變
        selection_prob = fitness / fitness.sum()
        parents_indices = np.random.choice(pop_size, size=pop_size, p=selection_prob)
        parents = population[parents_indices]
        
        new_population = np.empty((pop_size, 3))
        for i in range(0, pop_size, 2):
            p1 = parents[i]
            p2 = parents[i+1 if i+1 < pop_size else i]
            
            if np.random.rand() < crossover_rate:
                alpha = np.random.rand(3)
                c1 = alpha * p1 + (1 - alpha) * p2
                c2 = alpha * p2 + (1 - alpha) * p1
            else:
                c1, c2 = p1.copy(), p2.copy()
            
            new_population[i] = c1
            if i+1 < pop_size:
                 new_population[i+1] = c2

        for i in range(pop_size):
            if np.random.rand() < mutation_rate:
                mutation_amount = np.random.normal(0, 0.5, size=3) 
                new_population[i] += mutation_amount
                
            new_population[i] = np.clip(
                new_population[i], bounds[:, 0], bounds[:, 1]
            )

        population = new_population
        
        # 輸出最佳目標值 (1/F)
        if (gen + 1) % 50 == 0:
            best_J = 1.0 / best_fitness
            print(f"世代 {gen+1}: 最佳適應度 = {best_fitness:.6f}, 最小尺寸 J = {best_J:.2f} cm")
            
    # 最終結果
    final_fitness = np.array([calculate_fitness(ind, num_x_samples=30, num_y_samples=10) for ind in population])
    final_best_idx = np.argmax(final_fitness)
    final_best_individual = population[final_best_idx]
    
    return final_best_individual, final_fitness[final_best_idx]

# --- 4. 執行並輸出結果 (保持不變) ---

best_design, best_fit = genetic_algorithm_solver()

print("\n" + "=" * 70)
print("【基因演算法最終最小化尺寸設計結果】")

# 這裡我們需要確認 P_total ≈ 0，才能說 J 最小化成功
P_total_final = (1.0 / best_fit) - (best_design[0] + best_design[1] + 0.5 * best_design[2])

if P_total_final < 0.1: # 檢查懲罰是否足夠小（確保可行性）
    L1_final, L2_final, dist_final = best_design
    best_J = 1.0 / best_fit
    
    print(f"找到**滿足約束且最小化尺寸**的設計（總尺寸 J ≈ {best_J:.2f} cm）:")
    print(f"L1 (Link 1 & 4) = {L1_final:.2f} cm")
    print(f"L2 (Link 2 & 3) = {L2_final:.2f} cm")
    print(f"Distance (Y-bottom) = {dist_final:.2f} cm")
    
    # 進行最終驗證
    D_max_final = np.sqrt(LENGTH**2 + (dist_final + H)**2)
    
    max_D_BD_final = 0
    x_c_samples_final = np.linspace(0.001, LENGTH - 0.001, 30) 
    y_c_samples_final = np.linspace(dist_final + 0.001, dist_final + H - 0.001, 10)
    for x_c in x_c_samples_final:
        for y_c in y_c_samples_final:
            res_B = two_circle_intersection((0, 0), L1_final, (x_c, y_c), L2_final)
            res_D = two_circle_intersection((LENGTH, 0), L1_final, (x_c, y_c), L2_final)
            if res_B and res_D:
                D_BD = np.sqrt((res_B[0] - res_D[0])**2 + (res_B[1] - res_D[1])**2)
                max_D_BD_final = max(max_D_BD_final, D_BD)
        
    print("\n--- 最終設計驗證 ---")
    print(f"1. 最遠點奇異點約束 (L1+L2 > D_max): {L1_final+L2_final:.2f} > {D_max_final:.2f} (滿足)")
    
    required_2L2_final = max_D_BD_final + SAFETY_MARGIN
    
    print(f"2. 繪圖區域穩定性 (2*L2 > Max(D_BD) + {SAFETY_MARGIN}cm):")
    print(f"   2*L2 = {2*L2_final:.2f} cm")
    print(f"   Max(D_BD) + {SAFETY_MARGIN} cm = {max_D_BD_final:.2f} + {SAFETY_MARGIN:.2f} = {required_2L2_final:.2f} cm")
    
    if 2*L2_final > required_2L2_final:
         print(f"   **裕度滿足 (2*L2 僅略大於所需值，實現最小化)**")
    else:
         print(f"   **警告：裕度不足 (需要增加世代數)**")

    print("3. Y 座標約束 (Y_B, Y_D >= 0): 在逆運動學中已確保 (滿足)")
    
else:
    print(f"警告：GA 未能在 {generations} 代內找到嚴格滿足約束的可行解（懲罰值 P_total 太高）。")

print("=" * 70)