"""
Analytical Inverse Kinematics — exact formulas from ik.md
==========================================================
DH (6 rows): d1=114, a2=400, a3=300, d4=165.4, d5=136, d6=233.5

In all formulas, P_6^0 = P_ee - d6 * a  (wrist center in base frame)
where a = approach vector (3rd column of rotation matrix).
"""

import math
import numpy as np
from typing import List, Tuple


class IKSolver:

    def __init__(self):
        self.d1 = 0.114
        self.a2 = 0.400
        self.a3 = 0.300
        self.d4 = 0.1654
        self.d5 = 0.136
        self.d6 = 0.2335

    # ---- FK (for verification) ----
    @staticmethod
    def _dh_T(a, alpha, d, theta):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct,       -st,        0,       a],
            [st * ca,   ct * ca,  -sa, -sa * d],
            [st * sa,   ct * sa,   ca,  ca * d],
            [0,         0,         0,        1],
        ])
    
    def update_dh(self, d1=None, a2=None, a3=None, d4=None, d5=None, d6=None):
        if d1 is not None: self.d1 = d1
        if a2 is not None: self.a2 = a2
        if a3 is not None: self.a3 = a3
        if d4 is not None: self.d4 = d4
        if d5 is not None: self.d5 = d5
        if d6 is not None: self.d6 = d6

    def forward_kinematics(self, theta):
        dh = [(0,0,0.114),(0,math.pi/2,0),(0.4,0,0),(0.3,0,0.1654),(0,math.pi/2,0.136),(0,-math.pi/2,0.2335)]
        T = np.eye(4)
        T_list = []
        for i, (a, alpha, d) in enumerate(dh):
            Ti = self._dh_T(a, alpha, d, theta[i])
            T = T @ Ti
            T_list.append(T.copy())
        return T, T_list

    # ---- Analytical IK (ik.md 3.1–3.7) ----
    def solve(self, T: np.ndarray) -> List[np.ndarray]:
        # Target pose components
        px, py, pz = float(T[0,3]), float(T[1,3]), float(T[2,3])
        nx, ny, nz = float(T[0,0]), float(T[1,0]), float(T[2,0])
        ox, oy, oz = float(T[0,1]), float(T[1,1]), float(T[2,1])
        ax, ay, az = float(T[0,2]), float(T[1,2]), float(T[2,2])

        solutions = []

        # Wrist center (used in θ₁ and θ₅ formulas)
        Pwx = px - self.d6 * ax
        Pwy = py - self.d6 * ay

        # ---- 3.1 θ₁ ----
        # θ₁ = atan2(Pwy, Pwx) ± acos(d4 / √(Pwx²+Pwy²)) + π/2
        r_xy = math.hypot(Pwx, Pwy)
        if r_xy < 1e-12:
            return []
        d4r = self.d4 / r_xy
        if abs(d4r) > 1.0 + 1e-9:
            return []
        d4r = max(-1.0, min(1.0, d4r))
        phi = math.atan2(Pwy, Pwx)
        delta = math.acos(d4r)

        for sign1 in [+1, -1]:
            th1 = phi + sign1 * delta + math.pi / 2
            c1, s1 = math.cos(th1), math.sin(th1)

            # ---- 3.2 θ₅ ----
            # θ₅ = ± acos((px·s1 - py·c1 - d4) / d6)   ← uses P_ee, not wrist
            arg5 = (px * s1 - py * c1 - self.d4) / self.d6
            if abs(arg5) > 1.0 + 1e-9:
                continue
            arg5 = max(-1.0, min(1.0, arg5))
            th5_pos = math.acos(arg5)

            for s5_sign in [+1, -1]:
                th5 = s5_sign * th5_pos
                s5, c5 = math.sin(th5), math.cos(th5)

                # ---- 3.3 θ₆ ----
                if abs(s5) < 1e-10:
                    th6 = 0.0
                else:
                    th6 = math.atan2(
                        (-ox * s1 + oy * c1) / s5,
                        (nx * s1 - ny * c1) / s5,
                    )
                c6, s6 = math.cos(th6), math.sin(th6)

                # ---- 3.7 P₄¹ ----
                # P4x1 = -d6·(ax·c1 + ay·s1)
                #       + d5·(ox·c1·c6 + oy·s1·c6 + nx·c1·s6 + ny·s1·s6)
                #       + px·c1 + py·s1
                P4x1 = (-self.d6 * (ax * c1 + ay * s1) +
                        self.d5 * (ox * c1 * c6 + oy * s1 * c6 +
                                   nx * c1 * s6 + ny * s1 * s6) +
                        px * c1 + py * s1)

                # P4z1 = -d6·az + d5·(oz·c6 + nz·s6) + pz - d1
                P4z1 = (-self.d6 * az +
                        self.d5 * (oz * c6 + nz * s6) +
                        pz - self.d1)

                # ---- 3.4 θ₃ ----
                cos_t3 = (P4x1**2 + P4z1**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
                if abs(cos_t3) > 1.0 + 1e-9:
                    continue
                cos_t3 = max(-1.0, min(1.0, cos_t3))
                th3_pos = math.acos(cos_t3)

                for s3_sign in [+1, -1]:
                    th3 = s3_sign * th3_pos
                    c3, s3 = math.cos(th3), math.sin(th3)

                    # ---- 3.5 θ₂ ----
                    # Correct: θ₂ = atan2(A·P4z1 - B·P4x1, A·P4x1 + B·P4z1)
                    A = self.a3 * c3 + self.a2
                    B = self.a3 * s3
                    th2 = math.atan2(A * P4z1 - B * P4x1,
                                     A * P4x1 + B * P4z1)
                    c2, s2 = math.cos(th2), math.sin(th2)

                    # ---- 3.6 θ₄ ----
                    # n4x1 = -s5·(ax·c1+ay·s1) + c5·(nx·c1·c6+ny·s1·c6-ox·c1·s6-oy·s1·s6)
                    n4x1 = (-s5 * (ax * c1 + ay * s1) +
                            c5 * (nx * c1 * c6 + ny * s1 * c6 -
                                  ox * c1 * s6 - oy * s1 * s6))
                    # n4z1 = nz·c5·c6 - oz·c5·s6 - az·s5
                    n4z1 = nz * c5 * c6 - oz * c5 * s6 - az * s5

                    th4 = math.atan2(n4z1, n4x1) - th2 - th3

                    sol = np.array([th1, th2, th3, th4, th5, th6])
                    sol = np.arctan2(np.sin(sol), np.cos(sol))
                    solutions.append(sol)

        return solutions


# ---- Test ----
if __name__ == "__main__":
    ik = IKSolver()
    np.set_printoptions(precision=4, suppress=True)

    test_q = np.array([0.5, 0.3, -0.8, 1.2, -0.4, 0.6])
    T_target, _ = ik.forward_kinematics(test_q)
    print(f"Target pos: ({T_target[0,3]:.4f}, {T_target[1,3]:.4f}, {T_target[2,3]:.4f})")

    sols = ik.solve(T_target)
    print(f"\nFound {len(sols)} analytical solutions:")
    for i, sol in enumerate(sols):
        q_deg = np.degrees(sol)
        T_chk, _ = ik.forward_kinematics(sol)
        p_err = np.linalg.norm(T_chk[:3,3] - T_target[:3,3])
        r_err = np.linalg.norm(T_chk[:3,:3] - T_target[:3,:3])
        diff = np.max(np.abs(np.arctan2(np.sin(sol - test_q), np.cos(sol - test_q))))
        tag = " *** MATCH ***" if diff < 1e-4 else ""
        print(f"  Sol {i+1}: [{q_deg[0]:7.2f} {q_deg[1]:7.2f} {q_deg[2]:7.2f} {q_deg[3]:7.2f} {q_deg[4]:7.2f} {q_deg[5]:7.2f}]°  |perr|={p_err:.2e} |rerr|={r_err:.2e}{tag}")
