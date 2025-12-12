import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import math

# 逆向運動學方程式
x, y = sp.symbols('x y', real=True)
t1_expr = 2 * sp.atan((40*y + sp.sqrt(-x**4 - 2*x**2*y**2 + 2258*x**2 - y**4 + 2258*y**2 - 108241)) / (x**2 + 40*x + y**2 - 329))
t2_expr = 2 * sp.atan((40*y + sp.sqrt(-x**4 + 80*x**3 - 2*x**2*y**2 - 142*x**2 + 80*x*y**2 - 58320*x - y**4 + 1458*y**2 + 634959)) / (x**2 - 80*x + y**2 + 871))
f_t1 = sp.lambdify((x, y), t1_expr, modules='math')
f_t2 = sp.lambdify((x, y), t2_expr, modules='math')

L1 = 20
L2 = 27
motor_distance = 20

def sol4(x_val, y_val):
    t1 = f_t1(x_val, y_val)
    t2 = f_t2(x_val, y_val)
    return t1, t2

def get_points_C(xC, yC):
    t1, t2 = sol4(xC, yC)
    A = np.array([0, 0])
    E = np.array([motor_distance, 0])
    B = A + L1 * np.array([math.cos(t1), math.sin(t1)])
    D = E + L1 * np.array([-math.cos(t2), math.sin(t2)])
    C = np.array([xC, yC])
    return A, B, C, D, E

def generate_path_points(num_points_per_edge=50):
    points = []
    points += [(0, y) for y in np.linspace(20, 40, num_points_per_edge)]
    points += [(x, 40) for x in np.linspace(0, 20, num_points_per_edge)]
    points += [(20, y) for y in np.linspace(40, 20, num_points_per_edge)]
    points += [(x, 20) for x in np.linspace(20, 0, num_points_per_edge)]
    return points

def animate_5bar(save_as_gif=True, gif_filename="5bar_animation.gif"):
    path_points = generate_path_points()
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-20, 45)
    ax.set_ylim(0, 50)
    ax.grid(True)
    ax.set_title("5-bar planar plotter animation")

    rect_x = [0, 0, 20, 20, 0]
    rect_y = [20, 40, 40, 20, 20]
    ax.plot(rect_x, rect_y, 'k--', label='Desired path')

    link1_line, = ax.plot([], [], 'ro-', lw=3, label='Link1 (AB)')
    link2_line, = ax.plot([], [], 'go-', lw=3, label='Link2 (BC)')
    link3_line, = ax.plot([], [], 'bo-', lw=3, label='Link3 (CD)')
    link4_line, = ax.plot([], [], 'mo-', lw=3, label='Link4 (DE)')

    ax.legend()

    def update(frame):
        xC, yC = path_points[frame]
        try:
            A, B, C, D, E = get_points_C(xC, yC)
        except Exception as e:
            print(f"Error at frame {frame}: {e}")
            return link1_line, link2_line, link3_line, link4_line

        link1_line.set_data([A[0], B[0]], [A[1], B[1]])
        link2_line.set_data([B[0], C[0]], [B[1], C[1]])
        link3_line.set_data([C[0], D[0]], [C[1], D[1]])
        link4_line.set_data([D[0], E[0]], [D[1], E[1]])
        return link1_line, link2_line, link3_line, link4_line

    anim = FuncAnimation(fig, update, frames=len(path_points), interval=50, blit=True, repeat=True)

    if save_as_gif:
        writer = PillowWriter(fps=20)
        anim.save(gif_filename, writer=writer)
        print(f"GIF 已儲存：{gif_filename}")

    plt.show()

if __name__ == "__main__":
    animate_5bar()
