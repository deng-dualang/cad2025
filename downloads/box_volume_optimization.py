import math
import random
import time
import builtins

# 強制將內建函式還原，避免覆蓋導致錯誤
sum = builtins.sum
int = builtins.int
range = builtins.range


# === 全域常數 ===
MAXPOP = 5000
MAXDIM = 35
MAXIMAPROBLEM = 1
PENALITY = -1000 if MAXIMAPROBLEM else 1000

# 亂數參數
IM1, IM2 = 2147483563, 2147483399
AM = 1.0 / IM1
IMM1 = IM1 - 1
IA1, IA2 = 40014, 40692
IQ1, IQ2 = 53668, 52774
IR1, IR2 = 12211, 3791
NTAB = 32
NDIV = 1 + IMM1 // NTAB
EPS = 1.2e-7
RNMX = 1.0 - EPS

# === 資料結構 ===
c = [[0.0] * MAXDIM for _ in range(MAXPOP)]
d = [[0.0] * MAXDIM for _ in range(MAXPOP)]
best = [0.0] * MAXDIM
bestit = [0.0] * MAXDIM
tmp = [0.0] * MAXDIM
cost = [0.0] * MAXPOP
inibound_h = 50.0
inibound_l = 0.0

# === 隨機狀態 ===
rnd_state = {
    "idum2": 123456789,
    "iy": 0,
    "iv": [0] * NTAB
}

def assignd(D, a, b):
    for j in range(D):
        a[j] = b[j]

def rnd_uni(seed_wrapper):
    idum = seed_wrapper[0]
    s = rnd_state
    if idum <= 0:
        idum = max(1, -idum)
        s["idum2"] = idum
        for j in range(NTAB + 7, -1, -1):
            k = idum // IQ1
            idum = IA1 * (idum - k * IQ1) - k * IR1
            if idum < 0:
                idum += IM1
            if j < NTAB:
                s["iv"][j] = idum
        s["iy"] = s["iv"][0]
    k = idum // IQ1
    idum = IA1 * (idum - k * IQ1) - k * IR1
    if idum < 0:
        idum += IM1
    k = s["idum2"] // IQ2
    s["idum2"] = IA2 * (s["idum2"] - k * IQ2) - k * IR2
    if s["idum2"] < 0:
        s["idum2"] += IM2
    j = s["iy"] // NDIV
    s["iy"] = s["iv"][j] - s["idum2"]
    s["iv"][j] = idum
    if s["iy"] < 1:
        s["iy"] += IMM1
    temp = AM * s["iy"]
    seed_wrapper[0] = idum
    return RNMX if temp > RNMX else temp

def evaluate(D, tmp, nfeval_wrapper):
    surface = 80.0
    nfeval_wrapper[0] += 1
    z = (surface - tmp[0] * tmp[1]) / (2.0 * (tmp[0] + tmp[1]))
    volume = tmp[0] * tmp[1] * z
    if volume <= 0 or tmp[0] <= inibound_l or tmp[1] <= inibound_l:
        return PENALITY
    return volume

def main():
    strat = ["", "DE/best/1/exp", "DE/rand/1/exp", "DE/rand-to-best/1/exp",
             "DE/best/2/exp", "DE/rand/2/exp", "DE/best/1/bin", "DE/rand/1/bin",
             "DE/rand-to-best/1/bin", "DE/best/2/bin", "DE/rand/2/bin"]

    D = 2
    NP = 200
    seed = 3
    genmax = 2000
    refresh = 100
    strategy = 3
    F = 1.2
    CR = 0.9

    cmin = None
    nfeval = [0]
    seed_wrapper = [-seed]
    output_lines = []

    if not (0 < D <= MAXDIM and 0 < NP <= MAXPOP):
        print("維度或族群數超出範圍")
        return

    for i in range(NP):
        for j in range(D):
            c[i][j] = inibound_l + rnd_uni(seed_wrapper) * (inibound_h - inibound_l)
        cost[i] = evaluate(D, c[i], nfeval)

    cmin, imin = cost[0], 0
    for i in range(1, NP):
        if (MAXIMAPROBLEM and cost[i] > cmin) or (not MAXIMAPROBLEM and cost[i] < cmin):
            cmin, imin = cost[i], i

    assignd(D, best, c[imin])
    assignd(D, bestit, c[imin])
    pold, pnew = c, d
    start = time.time()

    for generation in range(1, genmax + 1):
        for i in range(NP):
            idxs = [x for x in range(NP) if x != i]
            r1, r2, r3, r4, r5 = random.sample(idxs, 5)
            assignd(D, tmp, pold[i])
            n = int(rnd_uni(seed_wrapper) * D)

            for L in range(D):
                if rnd_uni(seed_wrapper) < CR or L == D - 1:
                    if strategy == 1:
                        tmp[n] = bestit[n] + F * (pold[r2][n] - pold[r3][n])
                    elif strategy == 2:
                        tmp[n] = pold[r1][n] + F * (pold[r2][n] - pold[r3][n])
                    elif strategy == 3:
                        tmp[n] = tmp[n] + F * (bestit[n] - tmp[n]) + F * (pold[r1][n] - pold[r2][n])
                    elif strategy == 4:
                        tmp[n] = bestit[n] + F * (pold[r1][n] + pold[r2][n] - pold[r3][n] - pold[r4][n])
                    elif strategy == 5:
                        tmp[n] = pold[r5][n] + F * (pold[r1][n] + pold[r2][n] - pold[r3][n] - pold[r4][n])
                    elif strategy == 6:
                        tmp[n] = bestit[n] + F * (pold[r2][n] - pold[r3][n])
                    elif strategy == 7:
                        tmp[n] = pold[r1][n] + F * (pold[r2][n] - pold[r3][n])
                    elif strategy == 8:
                        tmp[n] = tmp[n] + F * (bestit[n] - tmp[n]) + F * (pold[r1][n] - pold[r2][n])
                    elif strategy == 9:
                        tmp[n] = bestit[n] + F * (pold[r1][n] + pold[r2][n] - pold[r3][n] - pold[r4][n])
                    else:
                        tmp[n] = pold[r5][n] + F * (pold[r1][n] + pold[r2][n] - pold[r3][n] - pold[r4][n])
                n = (n + 1) % D

            trial_cost = evaluate(D, tmp, nfeval)
            improved = (trial_cost >= cost[i]) if MAXIMAPROBLEM else (trial_cost <= cost[i])

            if improved:
                cost[i] = trial_cost
                assignd(D, pnew[i], tmp)
                if (MAXIMAPROBLEM and trial_cost > cmin) or (not MAXIMAPROBLEM and trial_cost < cmin):
                    cmin = trial_cost
                    imin = i
                    assignd(D, best, tmp)
            else:
                assignd(D, pnew[i], pold[i])

        assignd(D, bestit, best)
        pold, pnew = pnew, pold
        cmean = sum(cost) / NP
        cvar = sum((ci - cmean) ** 2 for ci in cost) / (NP - 1)

        if generation % refresh == 1:
            print(f"Generation {generation} | Best = {cmin:.8f} | Variance = {cvar:.6f}\n")
            for j in range(D):
                print(f"  best[{j}] = {best[j]:.6f}")
            output_lines.append(f"{nfeval[0]}   {cmin:.10g}\n")

    # Final output
    print("\nFinal Result:\n")
    print(f"Best-so-far obj. funct. value = {cmin:.10g}\n")
    for j in range(D):
        print(f"best[{j}] = {best[j]:.10g}\n")
    print(f"\nNFEs = {nfeval[0]}   Strategy: {strat[strategy]}\n")
    print(f"NP = {NP}    F = {F:.2f}    CR = {CR:.2f}   cost-variance = {cvar:.5g}\n")
    print(f"\nDone! Time elapsed: {time.time() - start:.4f} seconds\n")

# 呼叫主程式（可在 Pyodide 中執行）
main()
