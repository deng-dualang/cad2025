# pip install numpy matplotlib
import numpy as np
import matplotlib.pyplot as plt

# Link lengths
L1 = L4 = 16.82   # Length of driven links from A to B and E to D
L2 = L3 = 27.5    # Length of middle links from B to C and D to C

# Fixed base coordinates
A = np.array([6.25, 15.0])         # Base point A
E = np.array([-6.25, 15.0])       # Base point E

def forward_kinematics(theta1, theta2):
    """
    Given input angles theta1 (at A) and theta2 (at E), compute position of point C.
    Returns the coordinates of point C.
    """
    # Compute point B from A and theta1
    B = A + np.array([
        L1 * np.cos(theta1),     # x = L1 * cos(θ₁)
        -L1 * np.sin(theta1)     # y = -L1 * sin(θ₁) because θ₁ is clockwise
    ])

    # Compute point D from E and theta2
    D = E + np.array([
        L4 * np.cos(theta2),     # x = L4 * cos(θ₂)
        L4 * np.sin(theta2)      # y = L4 * sin(θ₂) (standard CCW positive)
    ])

    # Distance between B and D
    d = np.linalg.norm(D - B)

    # Check if the circles (around B and D) intersect
    if d > L2 + L3 or d < abs(L2 - L3):
        raise ValueError("No valid intersection: point C cannot be reached with given angles.")

    # Find point C: intersection of circles around B and D
    # Using circle-circle intersection equations (geometry-based)
    a = (L2**2 - L3**2 + d**2) / (2 * d)  # Distance from B to midpoint between intersections
    h = np.sqrt(L2**2 - a**2)            # Height from the base of triangle to intersection

    # Vector from B to D
    vecBD = (D - B) / d

    # Midpoint between intersections
    P = B + a * vecBD

    # Perpendicular vector to vecBD for intersection point
    perp = np.array([-vecBD[1], vecBD[0]])

    # Two possible solutions for point C
    C1 = P + h * perp   # Elbow-up
    C2 = P - h * perp   # Elbow-down
    print(C1)

    return C1, C2, B, D


def inverse_kinematics(C):
    """
    Given a target point C, compute the corresponding input angles theta1 and theta2.
    Returns both θ₁ and θ₂ solutions (up to two valid configurations).
    """
    x, y = C

    # Find point B: intersection of two circles (C as center, radius = L2; A as center, radius = L1)
    BA_candidates = circle_circle_intersection(A, L1, C, L2)

    # Find point D: same process, using E and L4, C and L3
    DE_candidates = circle_circle_intersection(E, L4, C, L3)

    solutions = []

    for B in BA_candidates:
        for D in DE_candidates:
            # Compute θ₁ from A to B
            dx1, dy1 = B - A
            theta1 = np.arctan2(-dy1, dx1)  # Clockwise correction

            # Compute θ₂ from E to D
            dx2, dy2 = D - E
            theta2 = np.arctan2(dy2, dx2)

            solutions.append((theta1, theta2))

    return solutions


def circle_circle_intersection(P0, r0, P1, r1):
    """
    Finds the intersection points of two circles centered at P0 and P1 with radii r0 and r1.
    Returns a list of 0, 1, or 2 intersection points.
    """
    d = np.linalg.norm(P1 - P0)

    # No solution
    if d > r0 + r1 or d < abs(r0 - r1):
        return []

    # Same center
    if d == 0 and r0 == r1:
        return []

    # Compute point 2
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(r0**2 - a**2)

    vec = (P1 - P0) / d
    P2 = P0 + a * vec
    offset = h * np.array([-vec[1], vec[0]])

    intersection1 = P2 + offset
    intersection2 = P2 - offset

    return [intersection1, intersection2]


# 🔧 Example usage:
if __name__ == "__main__":
    # Example input angles in radians
    theta1 = np.radians(0)  # 45° clockwise
    theta2 = np.radians(0)  # 60° counterclockwise

    print("=== Forward Kinematics ===")
    C1, C2, B, D = forward_kinematics(theta1, theta2)
    print(f"Point C solutions: \n  Elbow-up: {C1}\n  Elbow-down: {C2}")

    print("\n=== Inverse Kinematics ===")
    solutions = inverse_kinematics(C1)
    for i, (t1, t2) in enumerate(solutions):
        print(f"Solution {i+1}: θ₁ = {np.degrees(t1):.2f}°, θ₂ = {np.degrees(t2):.2f}°")
