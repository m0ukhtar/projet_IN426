import numpy as np

# Arm lengths from the DH parameters
L1, L2, L3, L4 = 0.43, 0.4, 0.3, 0.1

def compute_fk(theta1, theta2, theta3):
    # Compute trigonometric values
    c1, s1 = np.cos(theta1), np.sin(theta1)
    c2, s2 = np.cos(theta2), np.sin(theta2)
    c3, s3 = np.cos(theta3), np.sin(theta3)

    # Corrected FK position computation
    L34 = L3 + L4
    x = c1 * c2 * (L2 + L34 * c3)
    y = s1 * c2 * (L2 + L34 * c3)
    z = L1 + s2 * (L2 + L34 * c3) + L34 * s3

    return x, y, z

# Test cases from LAB 2: FORWARD KINEMATICS RESULTS
test_cases = [
    (0.0, 0.0, 0.0),           # Expected: (0.525, 0, 0.43)
    (0.45, 1.44, -1.23),       # Expected: (0.233, 0.1127, 0.7743)
    (-0.58, 0.4, -1.5)         # Expected: (0.3165, -0.2073, 0.3463)
]

for theta1, theta2, theta3 in test_cases:
    x, y, z = compute_fk(theta1, theta2, theta3)
    print(f"Angles ({theta1}, {theta2}, {theta3}): x={x:.4f}, y={y:.4f}, z={z:.4f}")