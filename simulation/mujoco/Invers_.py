import sympy as sp

# 1. 定义 7 个符号变量，代表 7 个关节状态
# q0, q1, q6 是平移量 (米)；q2, q3, q4, q5 是旋转角 (弧度)
q0, q1, q2, q3, q4, q5, q6 = sp.symbols('q0 q1 q2 q3 q4 q5 q6')

# 定义通用的绕 Z 轴旋转齐次矩阵 Rz(q)
def Rz(q):
    return sp.Matrix([
        [sp.cos(q), -sp.sin(q), 0, 0],
        [sp.sin(q),  sp.cos(q), 0, 0],
        [        0,          0, 1, 0],
        [        0,          0, 0, 1]
    ])

# 2. 根据公式构建各连杆的变换矩阵
# T0 (leftarmbase)
T0 = sp.Matrix([
    [1, 0, 0, -0.16145],
    [0, 1, 0, 0.3282 + q0],
    [0, 0, 1, 0.146],
    [0, 0, 0, 1]
])

# T1 (leftjoint1)
T1 = sp.Matrix([
    [1, 0, 0, 0.15658 + q1],
    [0, 1, 0, -0.096099],
    [0, 0, 1, 0.086],
    [0, 0, 0, 1]
])

# T2 (leftjoint2)
T2_offset = sp.Matrix([
    [ 0, 0, 1, 0.67335],
    [ 0, 1, 0, 0.10135],
    [-1, 0, 0, 0.04],
    [ 0, 0, 0, 1]
])
T2 = T2_offset * Rz(q2)

# T3 (leftjoint3)
T3_offset = sp.Matrix([
    [0, 1, 0, 0.01],
    [0, 0, 1, 0.0058],
    [1, 0, 0, 0.098],
    [0, 0, 0, 1]
])
T3 = T3_offset * Rz(q3)

# T4 (leftjoint4)
T4_offset = sp.Matrix([
    [0, 0, 1, 0.3418],
    [1, 0, 0, -0.01],
    [0, 1, 0, -0.082],
    [0, 0, 0, 1]
])
T4 = T4_offset * Rz(q4)

# T5 (leftjoint5)
T5_offset = sp.Matrix([
    [0, 1, 0, 0.01],
    [0, 0, 1, -0.0022],
    [1, 0, 0, 0.098],
    [0, 0, 0, 1]
])
T5 = T5_offset * Rz(q5)

# T6 (leftjoint6)
T6_offset = sp.Matrix([
    [1, 0,  0, 0.13],
    [0, 0, -1, 0],
    [0, 1,  0, 0.1435],
    [0, 0,  0, 1]
])
T6_joint = sp.Matrix([
    [1, 0, 0, q6],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T6 = T6_offset * T6_joint

# 末端工具偏置
T_ee_offset = sp.Matrix([
    [1, 0, 0, 0.132],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# 3. 连乘求解总的齐次变换矩阵
print("正在进行符号矩阵连乘计算...")
T_base_ee = T0 * T1 * T2 * T3 * T4 * T5 * T6 * T_ee_offset

print("正在计算完整 4x4 矩阵...")

# 打开一个文本文件，将整个矩阵写入
with open("FK_matrix.txt", "w", encoding="utf-8") as f:
    f.write("=== 左臂末端完整 4x4 齐次变换矩阵 (未化简) ===\n\n")
    
    # 遍历 4x4 矩阵的每一个元素
    for i in range(4):
        for j in range(4):
            # 获取矩阵元素
            element = T_base_ee[i, j]
            # 对其进行三角函数化简 (如果嫌慢可以把 sp.simplify 删掉)
            element_simp = sp.simplify(element) 
            f.write(f"T[{i+1}][{j+1}] = {element_simp}\n")
            f.write("-" * 80 + "\n")

print("完整的 4x4 矩阵已导出到当前目录下的 'FK_matrix.txt' 文件中！")