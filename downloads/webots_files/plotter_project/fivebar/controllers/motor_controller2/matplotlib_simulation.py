import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

with open("motion_data.pkl", "rb") as f:
    data = pickle.load(f)

theta1_list = data["theta1_list"]
theta2_list = data["theta2_list"]
valid_Cs = np.array(data["valid_Cs"])
A = np.array(data["A"])
E = np.array(data["E"])
L1 = data["L1"]
L4 = data["L4"]

actual_C = []

for t1, t2 in zip(theta1_list, theta2_list):
    Bx = A[0] + L1 * np.cos(-t1)
    By = A[1] + L1 * np.sin(-t1)
    Dx = E[0] + L4 * np.cos(t2)
    Dy = E[1] + L4 * np.sin(t2)
    Cx = (Bx + Dx) / 2
    Cy = (By + Dy) / 2
    actual_C.append((Cx, Cy))

actual_C = np.array(actual_C)

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-0.25, 0.35)
ax.set_ylim(-0.6, 0.25)
ax.set_aspect('equal')
ax.set_title("C 點軌跡誤差視覺化")

# 路徑線
exp_path, = ax.plot([], [], 'r-', label='預期 C 路徑')
act_path, = ax.plot([], [], 'g-', label='實際 C 路徑')
arrow_lines = []

def update(i):
    ax.clear()
    ax.set_xlim(-0.25, 0.35)
    ax.set_ylim(-0.6, 0.25)
    ax.set_aspect('equal')
    ax.axhline(0, color='gray', linestyle='--')
    ax.set_title("C 點軌跡誤差視覺化")
    ax.plot(valid_Cs[:i+1, 0], valid_Cs[:i+1, 1], 'r-', label='預期 C 路徑')
    ax.plot(actual_C[:i+1, 0], actual_C[:i+1, 1], 'g-', label='實際 C 路徑')
    for j in range(i+1):
        dx = actual_C[j, 0] - valid_Cs[j, 0]
        dy = actual_C[j, 1] - valid_Cs[j, 1]
        ax.arrow(valid_Cs[j, 0], valid_Cs[j, 1], dx, dy,
                 head_width=0.003, head_length=0.005, color='blue', length_includes_head=True, alpha=0.6)
    ax.legend()

ani = FuncAnimation(fig, update, frames=len(valid_Cs), interval=100)
ani.save("deviation_debug.gif", writer=PillowWriter(fps=16))
print("✅ 完成：deviation_debug.gif")