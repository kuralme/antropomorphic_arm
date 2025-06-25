#!/usr/bin/env python3

from antropomorphic_project.generate_matrixes import AntroArmDH
from sympy import N

if __name__ == "__main__":
    armdh = AntroArmDH()

    try:
        theta1 = float(input("Enter theta1 [rad]: "))
        theta2 = float(input("Enter theta2 [rad]: "))
        theta3 = float(input("Enter theta3 [rad]: "))
    except Exception as e:
        print("Invalid input:", e)
        exit(1)
    
    # Robot link lengths
    arm_r2 = 1
    arm_r3 = 1

    # Substitute values into the simplified matrix
    subs = {
        armdh.theta_1: theta1,
        armdh.theta_2: theta2,
        armdh.theta_3: theta3,
        armdh.a_2: arm_r2,
        armdh.a_3: arm_r3
    }
    evaluated = armdh.A03_simplified.subs(subs)
    evaluated = N(evaluated, 5)  # Numeric evaluation for readability

    # Save evaluated matrix as image
    armdh.save_matrix(evaluated, "A03_simplify_evaluated.png")

    # Print position and orientation
    pos = evaluated[:3, 3]
    rot = evaluated[:3, :3]
    print("\nPosition of Frame 3 (x, y, z):")
    print(pos)
    print("\nOrientation of Frame 3 (rotation matrix):")
    print(rot)