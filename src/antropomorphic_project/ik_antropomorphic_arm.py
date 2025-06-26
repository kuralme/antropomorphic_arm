#!/usr/bin/env python3

from math import atan2, sqrt, pi, cos, sin
from dataclasses import dataclass

@dataclass
class EndEffectorWorkingSpace3D:
    Px_ee: float
    Py_ee: float
    Pz_ee: float

class AntroArmIK:
    def __init__(self, DH_parameters):
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):
        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existent param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, elbow_config="down"):
        Px_ee = end_effector_pose.Px_ee
        Py_ee = end_effector_pose.Py_ee
        Pz_ee = end_effector_pose.Pz_ee

        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== ELBOW CONFIG = "+str(elbow_config))
        print("Px_ee = "+str(Px_ee))
        print("Py_ee = "+str(Py_ee))
        print("Pz_ee = "+str(Pz_ee))
        print("r1 = "+str(r1))
        print("r2 = "+str(r2))
        print("r3 = "+str(r3))

        theta_1 = atan2(Py_ee, Px_ee)
        r = sqrt(Px_ee**2 + Py_ee**2)
        s = Pz_ee

        # Law of cosines for theta3
        G = (r**2 + s**2 - r2**2 - r3**2) / (2 * r2 * r3)
        print(f"G = {G}")

        solution_possible = False
        if (G <= 1 + 1e-6) and (G >= -1 - 1e-6):
            G = max(min(G, 1), -1)
            solution_possible = True
        else:
            pass

        theta_array = []

        if solution_possible:

            # Check elbow configuration
            if elbow_config == "down":
                theta_3 = atan2(sqrt(1 - G**2), G)
            else:
                theta_3 = atan2(-sqrt(1 - G**2), G)

            k1 = r2 + r3 * cos(theta_3)
            k2 = r3 * sin(theta_3)
            theta_2 = atan2(s, r) - atan2(k2, k1)

            # Normalize angles to [-pi, pi]
            theta_1 = (theta_1 + pi) % (2*pi) - pi
            theta_2 = (theta_2 + pi) % (2*pi) - pi
            theta_3 = (theta_3 + pi) % (2*pi) - pi

            # Joint limits
            if not (-pi/4 <= theta_2 <= 3*pi/4 and -3*pi/4 <= theta_3 <= 3*pi/4):
                solution_possible = False
                theta_array = []
            else:
                theta_array = [theta_1, theta_2, theta_3]

        return theta_array, solution_possible

if __name__ == '__main__':
    DH_parameters = {
        "r1": 0,
        "r2": 1,
        "r3": 1
    }
    ik_solver = AntroArmIK(DH_parameters)

    try:
        Px_ee = float(input("Enter desired x position of End-Effector: "))
        Py_ee = float(input("Enter desired y position of End-Effector: "))
        Pz_ee = float(input("Enter desired z position of End-Effector: "))
        end_effector_pose = EndEffectorWorkingSpace3D(Px_ee=Px_ee, Py_ee=Py_ee, Pz_ee=Pz_ee)
    except Exception as e:
        print("Invalid input:", e)
        exit(1)
    
    for elbow_config in ["down", "up"]:
        thetas, solution_possible = ik_solver.compute_ik(end_effector_pose, elbow_config=elbow_config)
        print(f"Angles thetas solved = {thetas} , solution possible = {solution_possible}")
