"""
v5 双臂符号正向运动学 (sympy)
左臂: l0..l5, 右臂: r0..r5
"""
import sympy as sp


def Rz(q):
    return sp.Matrix([
        [sp.cos(q), -sp.sin(q), 0, 0],
        [sp.sin(q),  sp.cos(q), 0, 0],
        [        0,          0, 1, 0],
        [        0,          0, 0, 1]
    ])


def Rx(q):
    return sp.Matrix([
        [1,        0,         0, 0],
        [0, sp.cos(q), -sp.sin(q), 0],
        [0, sp.sin(q),  sp.cos(q), 0],
        [0,        0,         0, 1]
    ])


def transl(x, y, z):
    return sp.Matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


def build_chain(joint_params, q_syms, ee_x=0.128):
    """构建运动学链, 返回 T_base_ee 符号矩阵"""
    T = sp.eye(4)
    for i, (pos, rpy) in enumerate(joint_params):
        # offset
        T_off = transl(*pos)
        if rpy[0] != 0:
            T_off = T_off * Rx(rpy[0])
        if rpy[1] != 0:
            # Ry rotation via euler
            T_off = T_off * sp.Matrix([
                [sp.cos(rpy[1]), 0, sp.sin(rpy[1]), 0],
                [0, 1, 0, 0],
                [-sp.sin(rpy[1]), 0, sp.cos(rpy[1]), 0],
                [0, 0, 0, 1]
            ])
        if rpy[2] != 0:
            T_off = T_off * Rz(rpy[2])
        T = T * T_off * Rz(q_syms[i])

    # EE offset
    T = T * transl(0, 0, ee_x)
    return T


# ---- 符号变量 ----
l0, l1, l2, l3, l4, l5 = sp.symbols('l0 l1 l2 l3 l4 l5')
r0, r1, r2, r3, r4, r5 = sp.symbols('r0 r1 r2 r3 r4 r5')

# ---- 左臂参数 (URDF origin: pos + rpy) ----
left_params = [
    ([0.105,  0.26,   0.2],   [0, 0, 0]),
    ([0,      0.0905, 0.058], [-sp.pi/2, 0, 0]),
    ([0,     -0.4,    0.072], [0, 0, 0]),
    ([0,     -0.339, -0.15],  [sp.pi/2, 0, 0]),
    ([0,      0.075,  0.058], [-sp.pi/2, 0, 0]),
    ([0,     -0.083,  0.059], [sp.pi/2, 0, 0]),
]

# ---- 右臂参数 (Y 镜像 + Z 部分取反) ----
right_params = [
    ([0.105, -0.26,    0.2],   [0, 0, 0]),
    ([0,     -0.0905,  0.058], [-sp.pi/2, 0, 0]),
    ([0,     -0.4,    -0.072], [0, 0, 0]),
    ([0,     -0.339,   0.15],  [sp.pi/2, 0, 0]),
    ([0,     -0.076,   0.058], [-sp.pi/2, 0, 0]),
    ([0,     -0.083,  -0.058], [sp.pi/2, 0, 0]),
]

# ---- 计算 ----
print("计算左臂符号 FK...")
T_left = build_chain(left_params, [l0, l1, l2, l3, l4, l5])

print("计算右臂符号 FK...")
T_right = build_chain(right_params, [r0, r1, r2, r3, r4, r5])

print("简化中...")

with open("FK_matrix_v5.txt", "w", encoding="utf-8") as f:
    f.write("=" * 80 + "\n")
    f.write("v5 双臂末端符号正向运动学 (4x4 齐次矩阵)\n")
    f.write("参考坐标系: updown_link\n")
    f.write("=" * 80 + "\n\n")

    # ---- 左臂 ----
    f.write("【左臂】关节: l0=leftjoint1 .. l5=leftjoint6\n")
    f.write("-" * 80 + "\n")
    for i in range(4):
        for j in range(4):
            elem = sp.simplify(T_left[i, j])
            f.write(f"T_left[{i+1},{j+1}] = {elem}\n")
            f.write("-" * 70 + "\n")

    f.write("\n\n")

    # ---- 右臂 ----
    f.write("【右臂】关节: r0=rightjoint1 .. r5=rightjoint6\n")
    f.write("-" * 80 + "\n")
    for i in range(4):
        for j in range(4):
            elem = sp.simplify(T_right[i, j])
            f.write(f"T_right[{i+1},{j+1}] = {elem}\n")
            f.write("-" * 70 + "\n")

    # ---- EE位置向量 (简洁形式) ----
    f.write("\n\n=== 末端位置 (仅平移部分, 简化) ===\n\n")
    f.write("【左臂 EE 位置】\n")
    for axis, k in zip(['X', 'Y', 'Z'], [0, 1, 2]):
        elem = sp.simplify(T_left[k, 3])
        f.write(f"  {axis} = {elem}\n")
    f.write("\n【右臂 EE 位置】\n")
    for axis, k in zip(['X', 'Y', 'Z'], [0, 1, 2]):
        elem = sp.simplify(T_right[k, 3])
        f.write(f"  {axis} = {elem}\n")

print("完成! 结果写入 FK_matrix_v5.txt")
