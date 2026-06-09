"""
Forward Kinematics using Modified DH (Craig's Convention)
=========================================================

Modified DH transformation matrix from frame i-1 to frame i:
    T_{i-1}_i = Rot_x(α_{i-1}) * Trans_x(a_{i-1}) * Rot_z(θ_i) * Trans_z(d_i)

DH table columns: a_{i-1}, α_{i-1}, d_i, θ_i
"""

import sympy as sp
from sympy import sin, cos, symbols, simplify, pi, Matrix

# ============================================================
# Define symbolic variables for joint angles
# ============================================================
th1, th2, th3, th4, th5, th6 = symbols('θ1 θ2 θ3 θ4 θ5 θ6', real=True)

# ============================================================
# DH Parameters (a_{i-1}, α_{i-1}, d_i, θ_i) — Modified DH
# ============================================================
# Row format: [a, α (deg), d, θ]
dh_params_raw = [
    [0,     0,   114,    th1],        # T_0_1
    [0,     90,  0,      th2],        # T_1_2
    [400,   0,   0,      th3],        # T_2_3
    [300,   0,   165.4,  th4],        # T_3_4
    [0,     90,  136,    th5],        # T_4_5
    [0,    -90,  233.5,  th6],        # T_5_6
]

# Convert angles in degrees to radians (for constant offsets)
dh_params = []
for a, alpha_deg, d, theta in dh_params_raw:
    # α: convert degrees → radians (symbolic if contains symbols)
    if isinstance(alpha_deg, (int, float)):
        alpha = sp.rad(alpha_deg)  # sympy rad = pi/180 * deg
    else:
        alpha = alpha_deg

    if isinstance(theta, (int, float)):
        theta_rad = sp.rad(theta)
    else:
        # theta expression may contain symbols + constants like "th2 - 90"
        # Replace numeric constants in degrees with radians
        theta_rad = theta
        # Substitute 90 → pi/2, -90 → -pi/2
        theta_rad = theta_rad.subs(90, sp.pi/2)
        theta_rad = theta_rad.subs(-90, -sp.pi/2)

    dh_params.append([a, alpha, d, theta_rad])


def mdh_transform(a, alpha, d, theta):
    """
    Compute the 4x4 homogeneous transformation matrix for a single
    Modified DH parameter set.

    T = Rot_x(α) * Trans_x(a) * Rot_z(θ) * Trans_z(d)

    Returns a sympy 4x4 Matrix.
    """
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)

    T = Matrix([
        [ct,        -st,        0,      a           ],
        [st * ca,    ct * ca,  -sa,    -sa * d     ],
        [st * sa,    ct * sa,   ca,     ca * d     ],
        [0,          0,         0,      1           ],
    ])
    return T


# ============================================================
# Compute all individual transformation matrices
# ============================================================
T_matrices = []
for i, (a, alpha, d, theta) in enumerate(dh_params):
    T_i = mdh_transform(a, alpha, d, theta)
    T_matrices.append(T_i)
    print(f"--- T_{i}_{i+1} ---")
    sp.pprint(simplify(T_i))
    print()

# ============================================================
# Compute overall transformation T_0_7 (base → end-effector)
# ============================================================
T = sp.eye(4)
for i, T_i in enumerate(T_matrices):
    T = T * T_i

T_simplified = simplify(T)

print("=" * 80)
print("Overall Transformation Matrix T_0_7 (Base → End-Effector):")
print("=" * 80)
sp.pprint(T_simplified)

# ============================================================
# Extract position and orientation
# ============================================================
px = T_simplified[0, 3]
py = T_simplified[1, 3]
pz = T_simplified[2, 3]

print("\n" + "=" * 80)
print("End-Effector Position (x, y, z):")
print("=" * 80)
print(f"Px =")
sp.pprint(px)
print(f"\nPy =")
sp.pprint(py)
print(f"\nPz =")
sp.pprint(pz)

# ============================================================
# Export to file (Markdown format)
# ============================================================
output_lines = []
output_lines.append("# Forward Kinematics — Modified DH Method\n")
output_lines.append("## DH Parameter Table\n")
output_lines.append("| Joint i | a_{i-1} | α_{i-1} (°) | d_i | θ_i |")
output_lines.append("|---------|----------|-------------|-----|-----|")

# Pretty-print the DH table
for i, (a, alpha_deg, d, theta_expr) in enumerate(dh_params_raw):
    alpha_str = str(alpha_deg)
    d_str = str(d)
    theta_str = str(theta_expr)
    a_str = str(a)
    output_lines.append(f"| {i+1} | {a_str} | {alpha_str} | {d_str} | {theta_str} |")

output_lines.append("")
output_lines.append("> **Note:** Angles are in degrees. `θ1`–`θ6` are the joint variables.\n")

output_lines.append("## Individual Transformation Matrices\n")

for i, T_i in enumerate(T_matrices):
    output_lines.append(f"### $T_{{{i}}}^{{{i+1}}}$\n")
    output_lines.append("$$\n" + sp.latex(simplify(T_i), mode='plain') + "\n$$\n")
    output_lines.append("")

output_lines.append("## Overall Transformation Matrix $T_0^7$\n")
output_lines.append("$$\n" + sp.latex(T_simplified, mode='plain') + "\n$$\n")
output_lines.append("")

output_lines.append("## End-Effector Position\n")
output_lines.append(f"$$P_x = {sp.latex(px)}$$\n")
output_lines.append(f"$$P_y = {sp.latex(py)}$$\n")
output_lines.append(f"$$P_z = {sp.latex(pz)}$$\n")

# Write to file (Markdown)
output_path_md = r"E:\qyx\robot_v7\forward_kinematics_output.md"
with open(output_path_md, 'w', encoding='utf-8') as f:
    f.write('\n'.join(output_lines))

print(f"\n[INFO] Markdown output written to: {output_path_md}")

# ============================================================
# Export to TXT file (plain text with Unicode pretty-print)
# Only final transformation matrix, no intermediate ones
# ============================================================
txt_lines = []
sep = "=" * 80

txt_lines.append(sep)
txt_lines.append("Forward Kinematics — Modified DH Method (Craig's Convention)")
txt_lines.append(sep)
txt_lines.append("")

# DH table
txt_lines.append("DH Parameter Table:")
txt_lines.append(f"{'Joint i':<8} {'a_{i-1}':<10} {'α_{i-1}(°)':<12} {'d_i':<10} {'θ_i':<15}")
txt_lines.append("-" * 55)
for i, (a, alpha_deg, d, theta_expr) in enumerate(dh_params_raw):
    txt_lines.append(f"{i+1:<8} {str(a):<10} {str(alpha_deg):<12} {str(d):<10} {str(theta_expr):<15}")
txt_lines.append("")
txt_lines.append("Note: Angles are in degrees. θ1–θ6 are the joint variables.")
txt_lines.append("")

# Overall transformation matrix T_0^7
txt_lines.append(sep)
txt_lines.append("Overall Transformation Matrix T_0^7 (Base → End-Effector)")
txt_lines.append(sep)
txt_lines.append("")
txt_lines.append(sp.pretty(T_simplified, use_unicode=True, num_columns=1000, wrap_line=False))
txt_lines.append("")

# Rotation sub-matrix R
R = T_simplified[0:3, 0:3]

txt_lines.append(sep)
txt_lines.append("Rotation Matrix R (3×3)")
txt_lines.append(sep)
txt_lines.append("")
for row in range(3):
    for col in range(3):
        txt_lines.append(f"  R[{row},{col}] = {sp.pretty(R[row, col], use_unicode=True, num_columns=1000, wrap_line=False)}")
txt_lines.append("")

# End-effector position
txt_lines.append(sep)
txt_lines.append("End-Effector Position")
txt_lines.append(sep)
txt_lines.append("")
txt_lines.append(f"  Px = {sp.pretty(px, use_unicode=True, num_columns=1000, wrap_line=False)}")
txt_lines.append(f"  Py = {sp.pretty(py, use_unicode=True, num_columns=1000, wrap_line=False)}")
txt_lines.append(f"  Pz = {sp.pretty(pz, use_unicode=True, num_columns=1000, wrap_line=False)}")
txt_lines.append("")

# r² = Px² + Py² + Pz²
r2 = simplify(px**2 + py**2 + pz**2)
txt_lines.append(sep)
txt_lines.append("r² = Px² + Py² + Pz²")
txt_lines.append(sep)
txt_lines.append("")
txt_lines.append(f"  r² = {sp.pretty(r2, use_unicode=True, num_columns=1000, wrap_line=False)}")

# Write to TXT file
output_path_txt = r"E:\qyx\robot_v7\forward_kinematics_output.txt"
with open(output_path_txt, 'w', encoding='utf-8') as f:
    f.write('\n'.join(txt_lines))

print(f"[INFO] TXT output written to: {output_path_txt}")
