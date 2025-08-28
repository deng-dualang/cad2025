import numpy as np  # NumPy is used for vector and matrix operations
import matplotlib.pyplot as plt  # Matplotlib handles plotting
# need ffmpeg.exe from sharex
from matplotlib.animation import FuncAnimation, FFMpegWriter  # Animation tools for updating frames and saving to MP4
# pip install pillow
from matplotlib.animation import PillowWriter # also Save animation as a GIF

# Specify the path to FFmpeg executable for saving video files
plt.rcParams['animation.ffmpeg_path'] = r'Y:\sharex\ffmpeg.exe'

# Define lengths of the five-bar linkage arms
L1 = L4 = 67.27  # Input links (equal length)
L2 = L3 = 110    # Output links (also equal)
A = np.array([0.0, 0.0])  # Fixed pivot point A on the left
E = np.array([-50.0, 0.0])  # Fixed pivot point E on the right

# Function to find intersection points of two circles centered at P0 and P1
def circle_circle_intersection(P0, r0, P1, r1):
    d = np.linalg.norm(P1 - P0)  # Distance between centers
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:  # No solution
        return []
    a = (r0**2 - r1**2 + d**2) / (2 * d)  # Distance from P0 to midpoint
    h = np.sqrt(abs(r0**2 - a**2))  # Height from line to intersection
    v = (P1 - P0) / d  # Unit vector between centers
    P2 = P0 + a * v  # Base point of perpendicular
    offset = h * np.array([-v[1], v[0]])  # Rotate 90° to get intersection offsets
    return [P2 + offset, P2 - offset]  # Return both intersection points

# Inverse kinematics: given point C, solve for θ1, θ2, B, D
def inverse_kinematics(C):
    BA = circle_circle_intersection(A, L1, C, L2)  # Find possible points for B
    DE = circle_circle_intersection(E, L4, C, L3)  # Find possible points for D
    if BA and DE:  # If both exist
        B = BA[0]  # Use first solution for B
        D = DE[1]  # Use first solution for D
        if B[1] < A[1] and D[1] < E[1]:  # Only keep configuration where B and D stay below A and E
            θ1 = np.arctan2(-B[1], B[0])  # θ1 angle from A to B
            θ2 = np.arctan2(D[1], D[0] - E[0])  # θ2 angle from E to D
            return θ1, θ2, B, D  # Return valid joint angles and positions
    return None  # No valid solution

# Generate 12 evenly spaced C positions in a circle below the base
N = 12  # Number of positions
radius = 30  # Radius of the circle
safe_y = -145  # Lower center to ensure all joints stay below base
center = np.array([-25.0, safe_y])  # Center of the circular path
angles = np.linspace(0, 2 * np.pi, N, endpoint=False)  # Divide circle evenly
C_points = [center + radius * np.array([np.cos(a), np.sin(a)]) for a in angles]  # List of C positions
# add the first point to the array
C_points.append(C_points[0])

# Precompute inverse kinematics for each C point
θ1_list, θ2_list = [], []  # Lists for input angles
B_points, D_points, valid_Cs = [], [], []  # Lists for positions

# Loop through each C position and compute inverse kinematics
for i, C in enumerate(C_points):
    res = inverse_kinematics(C)
    if res:
        θ1, θ2, B, D = res
        θ1_list.append(θ1)
        θ2_list.append(θ2)
        B_points.append(B)
        D_points.append(D)
        valid_Cs.append(C)
    else:
        print(f"⚠️ Skipped C[{i}] at {C}: B or D above base.")  # Warn if configuration is skipped

# Count how many valid positions we got
steps = len(valid_Cs)
if steps < N:
    print(f"✅ {steps} out of {N} poses met the below-base condition.")  # Summary of successful IK solves

# Convert lists to numpy arrays for convenience
θ1_list = np.array(θ1_list)
θ2_list = np.array(θ2_list)
B_points = np.array(B_points)
D_points = np.array(D_points)
valid_Cs = np.array(valid_Cs)

# Set up the plot
fig, ax = plt.subplots(figsize=(6, 6))  # Create square figure
ax.set_xlim(-140, 60)  # X range
ax.set_ylim(-180, 60)  # Y range
ax.set_aspect('equal')  # Preserve aspect ratio
ax.set_title("5-Bar Linkage (B & D Stay Below A & E)")  # Title of the plot

# Initialize plot lines (linkage and traces)
link_line, = ax.plot([], [], 'o-', lw=3)  # Main linkage configuration
trace_B, = ax.plot([], [], 'b--', label='B trace')  # Path of point B
trace_D, = ax.plot([], [], 'g--', label='D trace')  # Path of point D
C_path, = ax.plot(valid_Cs[:, 0], valid_Cs[:, 1], 'ro-', alpha=0.3, label='C path')
# Path of C

# Draw horizontal reference line for base
ax.axhline(0, color='gray', linestyle='dotted')  # Visual reference for base (Y = 0)
ax.plot([A[0]], [A[1]], 'ko', label='A')  # Fixed point A
ax.plot([E[0]], [E[1]], 'ko', label='E')  # Fixed point E
ax.legend()  # Show legend

# Animation update function: draws each frame of the animation
def update(i):
    Bx, By = B_points[i]
    Cx, Cy = valid_Cs[i]
    Dx, Dy = D_points[i]
    x_vals = [A[0], Bx, Cx, Dx, E[0]]
    y_vals = [A[1], By, Cy, Dy, E[1]]
    link_line.set_data(x_vals, y_vals)  # Update linkage geometry
    trace_B.set_data(B_points[:i+1, 0], B_points[:i+1, 1])  # Update B trace
    trace_D.set_data(D_points[:i+1, 0], D_points[:i+1, 1])  # Update D trace
    return link_line, trace_B, trace_D, C_path  # Return updated artists

# Create the animation from update function
ani = FuncAnimation(fig, update, frames=steps, interval=1000, blit=True)  # 1 frame/sec
plt.tight_layout()  # Adjust layout to prevent clipping

# Export animation to MP4 using FFmpeg
writer = FFMpegWriter(fps=1, metadata={"artist": "5-Bar Linkage"}, bitrate=1800)
ani.save("5bar_simulation_below_base.mp4", writer=writer)  # Save the video
print("✅ Saved: 5bar_simulation_below_base.mp4")  # Confirmation
# Save animation as a GIF
gif_writer = PillowWriter(fps=1)  # Adjust fps as needed
ani.save("5bar_simulation.gif", writer=gif_writer)
print("✅ Saved: 5bar_simulation.gif")
