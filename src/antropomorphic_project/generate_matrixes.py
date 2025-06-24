#!/usr/bin/env python3

import os
from sympy import Matrix, cos, sin, pi, Symbol, simplify, trigsimp, preview
from sympy.interactive import printing

class AntroArmDH:
    def __init__(self):
        printing.init_printing(use_latex=True)
        
        # Create output directory if it doesn't exist
        project_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.output_dir = os.path.join(project_dir, "output")
        os.makedirs(self.output_dir, exist_ok=True)

        def dh_matrix(theta, a, alpha, d):
            return Matrix([
                [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]
            ])

        # Symbolic DH parameters
        self.theta_1 = Symbol("theta_1")
        self.theta_2 = Symbol("theta_2")
        self.theta_3 = Symbol("theta_3")
        self.alpha_1 = Symbol("alpha_1")
        self.alpha_2 = Symbol("alpha_2")
        self.alpha_3 = Symbol("alpha_3")
        self.a_1 = Symbol("r_1")
        self.a_2 = Symbol("r_2")
        self.a_3 = Symbol("r_3")
        self.d_1 = Symbol("d_1")
        self.d_2 = Symbol("d_2")
        self.d_3 = Symbol("d_3")

        # Robot parameters
        self.a_1 = 0
        self.alpha_1 = pi/2
        self.alpha_2 = 0
        self.alpha_3 = 0
        self.d_1 = 0
        self.d_2 = 0
        self.d_3 = 0

        # Form transformation matrices
        self.A01 = dh_matrix(self.theta_1, self.a_1, self.alpha_1, self.d_1)
        self.A12 = dh_matrix(self.theta_2, self.a_2, self.alpha_2, self.d_2)
        self.A23 = dh_matrix(self.theta_3, self.a_3, self.alpha_3, self.d_3)
        self.A03 = self.A01 * self.A12 * self.A23
        self.A03_simplified = trigsimp(self.A03)

    def save_matrix(self, matrix, filename, do_simplify=False):
        """Save a symbolic matrix as a PNG file in the output directory."""
        if do_simplify:
            matrix = simplify(matrix)
        filepath = os.path.join(self.output_dir, filename)
        preview(matrix, viewer='file', filename=filepath, dvioptions=['-D', '300'])

    def save_all(self):
        """Save all relevant matrices to files."""
        self.save_matrix(self.A01, "A0_1.png")
        self.save_matrix(self.A12, "A1_2.png")
        self.save_matrix(self.A23, "A2_3.png")
        self.save_matrix(self.A03, "A0_3.png")
        self.save_matrix(self.A03_simplified, "A0_3_simplified.png")
        print('Transformation matrices generated under output dir')

if __name__ == "__main__":
    armdh = AntroArmDH()
    armdh.save_all()