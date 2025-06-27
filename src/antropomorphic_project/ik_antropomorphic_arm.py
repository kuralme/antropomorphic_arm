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

    def compute_ik(self, end_effector_pose, theta2_config="plus", theta3_config="plus"):
        Px_ee = end_effector_pose.Px_ee
        Py_ee = end_effector_pose.Py_ee
        Pz_ee = end_effector_pose.Pz_ee

        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        # print("Input Data===== ELBOW CONFIG = "+str(theta3_config))
        # print("Px_ee = "+str(Px_ee))
        # print("Py_ee = "+str(Py_ee))
        # print("Pz_ee = "+str(Pz_ee))
        # print("r1 = "+str(r1))
        # print("r2 = "+str(r2))
        # print("r3 = "+str(r3))

        r = sqrt(Px_ee**2 + Py_ee**2)
        s = Pz_ee

        # Law of cosines for theta3
        G = (r**2 + s**2 - r2**2 - r3**2) / (2 * r2 * r3)

        # Apply config for theta3
        if theta3_config == "plus":
            theta_3 = atan2(sqrt(1 - G**2), G)
        else:
            theta_3 = atan2(-sqrt(1 - G**2), G)

        k1 = r2 + r3 * cos(theta_3)
        k2 = r3 * sin(theta_3)

        # Apply config for theta2
        if theta2_config == "plus":
            theta_1 = atan2(Py_ee, Px_ee)
            theta_2 = atan2(s, r) - atan2(k2, k1)
        else:
            theta_1 = atan2(-Py_ee, -Px_ee)
            theta_2 = - atan2(s, r) + atan2(k2, k1)

        theta_array = [theta_1, theta_2, theta_3]

        # Solution possible if theta2, theta3 and G are within limits
        solution_possible = (-pi/4 <= theta_2 <= 3*pi/4 and -3*pi/4 <= theta_3 <= 3*pi/4 and abs(G) <= 1)

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

    for theta2_config in ["plus", "minus"]:
        for theta3_config in ["plus", "minus"]:
            thetas, solution_possible = ik_solver.compute_ik(
                end_effector_pose,
                theta2_config=theta2_config,
                theta3_config=theta3_config
            )
            # print(f"theta2_config = {theta2_config}, theta3_config = {theta3_config}")
            print(f"Angles thetas solved = {thetas} , solution possible = {solution_possible}")
