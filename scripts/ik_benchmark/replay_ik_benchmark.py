#!/usr/bin/env python3
"""
IK Benchmark 3D Replay Viewer

基于 rerun-sdk + yourdfpy，将 ik_benchmark 生成的 .jsonl 压测结果
在 Rerun 时间轴上进行 3D 可视化回放。

可视化内容：
  - FK 原始末端位姿（绿色=左臂，橙色=右臂，小球 + 坐标轴）
  - 扰动后的 IK 目标位姿（青色=左臂，黄色=右臂，大球 + 坐标轴）
  - IK 求解成功的机器人姿态（灰色实体机器人）
  - IK 求解成功后的实际末端位姿（紫色=左臂，红色=右臂，中球 + 坐标轴）
  - 误差线：目标 → 实际

用法：
  python3 replay_ik_benchmark.py result.jsonl
  python3 replay_ik_benchmark.py result.jsonl --count 10
  python3 replay_ik_benchmark.py result.jsonl --count 20 --seed 42

依赖：
  pip install rerun-sdk yourdfpy
"""

import argparse
import json
import os
import random
import subprocess
import sys
from pathlib import Path

import numpy as np
import rerun as rr
import yourdfpy


# ── package:// URI 解析 ─────────────────────────────────────────────────────

def _resolve_package_path(fname: str) -> str:
    prefix = "package://"
    if not fname.startswith(prefix):
        return fname
    rel = fname[len(prefix):]
    pkg_name = rel.split("/")[0]
    rest = rel[len(pkg_name) + 1:]
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", pkg_name],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            return os.path.join(result.stdout.strip(), "share", pkg_name, rest)
    except Exception:
        pass
    return fname


# ── yourdfpy 桥接 ────────────────────────────────────────────────────────────

def load_urdf(urdf_path: str) -> yourdfpy.URDF:
    p = Path(urdf_path)
    if not p.exists():
        print(f"错误：URDF 文件不存在: {urdf_path}", file=sys.stderr)
        sys.exit(1)
    print(f"加载 URDF: {p}")
    return yourdfpy.URDF.load(str(p), filename_handler=_resolve_package_path)


def get_link_geometries(urdf: yourdfpy.URDF):
    """遍历场景图，返回 [(node, T, verts, faces)]"""
    sg = urdf.scene.graph
    scene = urdf.scene
    result = []
    for node in sorted(sg.nodes):
        ret = sg.get(node)
        if ret is None:
            continue
        T, geom_name = ret
        if geom_name is None or geom_name not in scene.geometry:
            continue
        geom = scene.geometry[geom_name]
        mesh = geom.to_mesh() if hasattr(geom, "to_mesh") else geom
        if not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
            continue
        verts = mesh.vertices @ T[:3, :3].T + T[:3, 3]
        faces = mesh.faces
        result.append((node, T.copy(), verts.copy(), faces.copy()))
    return result


# MoveIt JMG 关节顺序（与 ik_benchmark.cpp copyJointGroupPositions 一致）
# dual_arm_with_base 组不含 turn，共 13 个关节
MOVEIT_NAMES = [
    "updown",
    "leftjoint1", "leftjoint2", "leftjoint3",
    "leftjoint4", "leftjoint5", "leftjoint6",
    "rightjoint1", "rightjoint2", "rightjoint3",
    "rightjoint4", "rightjoint5", "rightjoint6",
]


def apply_joints(urdf: yourdfpy.URDF, joints: list):
    """按名称映射将关节角应用到 yourdfpy URDF"""
    yourdfpy_names = [j.name for j in urdf.actuated_joints]
    name_to_cfg_idx = {n: i for i, n in enumerate(yourdfpy_names)}
    urdf.cfg[:] = 0.0
    for mi, name in enumerate(MOVEIT_NAMES):
        if mi < len(joints) and name in name_to_cfg_idx:
            urdf.cfg[name_to_cfg_idx[name]] = float(joints[mi])
    urdf.update_cfg(urdf.cfg)


def log_robot(urdf: yourdfpy.URDF, entity_prefix: str, color: list[int]):
    """将机器人几何体发送到 Rerun"""
    link_geoms = get_link_geometries(urdf)
    for node, _T, verts, faces in link_geoms:
        path = f"{entity_prefix}/{node}"
        n_verts = len(verts)
        vertex_colors = np.tile(np.array(color, dtype=np.uint8), (n_verts, 1))
        rr.log(
            path,
            rr.Mesh3D(
                vertex_positions=verts.astype(np.float32),
                triangle_indices=faces.astype(np.uint32),
                vertex_colors=vertex_colors,
            ),
        )


# ── 位姿辅助 ────────────────────────────────────────────────────────────────

def quat_to_rotmat(qx, qy, qz, qw):
    """四元数 → 3x3 旋转矩阵"""
    n = (qx**2 + qy**2 + qz**2 + qw**2) ** 0.5
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ])


def pose_to_transform(pose: dict) -> np.ndarray:
    """{pos:[x,y,z], quat:[qx,qy,qz,qw]} → 4x4 齐次矩阵"""
    px, py, pz = pose["pos"]
    qx, qy, qz, qw = pose["quat"]
    R = quat_to_rotmat(qx, qy, qz, qw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [px, py, pz]
    return T


def log_end_effector(entity_prefix: str, pose: dict, color: list[int],
                     radius: float = 0.008, axis_length: float = 0.04):
    """记录末端位姿：小球 + 坐标轴"""
    tf = pose_to_transform(pose)
    px, py, pz = pose["pos"]
    R = tf[:3, :3]
    t = tf[:3, 3]

    rr.log(
        f"{entity_prefix}/point",
        rr.Points3D(
            positions=[[px, py, pz]],
            radii=[radius],
            colors=[color],
        ),
    )

    # RGB 坐标轴
    axis_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    for axis_idx in range(3):
        end = t + R[:, axis_idx] * axis_length
        rr.log(
            f"{entity_prefix}/axis_{axis_idx}",
            rr.LineStrips3D(
                [[t.tolist(), end.tolist()]],
                colors=[axis_colors[axis_idx]],
                radii=0.001,
            ),
        )


def jsonl_pose_to_dict(raw) -> dict | None:
    """兼容新旧 JSONL 格式"""
    if raw is None:
        return None
    # 新格式: {pos:[x,y,z], quat:[qx,qy,qz,qw]}
    if isinstance(raw, dict) and "pos" in raw and "quat" in raw:
        return raw
    # 旧格式: [x,y,z] (只有位置)
    if isinstance(raw, list) and len(raw) == 3:
        return {"pos": raw, "quat": [0, 0, 0, 1]}
    return None


# ── JSONL 读取 ───────────────────────────────────────────────────────────────

def load_jsonl(path: str) -> list[dict]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("header"):
                continue
            records.append(obj)
    print(f"加载 {len(records)} 条记录 from {path}")
    return records


# ── URDF 查找 ────────────────────────────────────────────────────────────────

def find_urdf() -> str | None:
    candidates = [
        "/tmp/_alfa_bench.urdf",
        "alfa_robot.urdf",
        "robot.urdf",
    ]
    for c in candidates:
        if Path(c).exists():
            return c
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "alfa_robot_description"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            pkg_dir = result.stdout.strip()
            for rel in [
                "share/alfa_robot_description/urdf/alfa_robot.urdf",
                "share/alfa_robot_description/urdf/alfa_robot/alfa_robot.urdf",
            ]:
                p = Path(pkg_dir) / rel
                if p.exists():
                    return str(p)
    except Exception:
        pass
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "alfa_robot_description"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode == 0:
            pkg_dir = result.stdout.strip()
            xacro_src = Path(pkg_dir) / "share/alfa_robot_description/urdf/alfa_robot.urdf.xacro"
            if xacro_src.exists():
                out = "/tmp/_alfa_bench.urdf"
                print(f"运行 xacro 生成 URDF: {out}")
                subprocess.run(
                    f"xacro {xacro_src} > {out} 2>/dev/null",
                    shell=True, check=True, timeout=10,
                )
                return out
    except Exception:
        pass
    return None


# ── 主流程 ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="IK Benchmark 3D 回放查看器")
    parser.add_argument("jsonl", help="ik_benchmark 生成的 .jsonl 文件路径")
    parser.add_argument("--urdf", default=None, help="URDF 文件路径（自动检测则省略）")
    parser.add_argument(
        "--save", default=None,
        help="保存到 .rrd 文件而不启动查看器（例如 --save output.rrd）",
    )
    parser.add_argument(
        "--count", type=int, default=None,
        help="随机抽取 N 条记录进行回放（默认全部加载）",
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="随机种子（配合 --count 使用，保证可复现）",
    )
    args = parser.parse_args()

    # 查找 URDF
    urdf_path = args.urdf or find_urdf()
    if not urdf_path:
        print(
            "错误：找不到 URDF 文件。请用 --urdf 指定路径，"
            "或 source ROS2 工作空间后重试。",
            file=sys.stderr,
        )
        sys.exit(1)

    # 加载数据
    records = load_jsonl(args.jsonl)
    if not records:
        print("错误：没有有效数据行", file=sys.stderr)
        sys.exit(1)

    # 随机抽样
    if args.count is not None and args.count < len(records):
        rng = random.Random(args.seed)
        records = rng.sample(records, args.count)
        records.sort(key=lambda r: r["index"])
        print(f"随机抽取 {args.count} 条记录（seed={args.seed}）")

    # 初始化 Rerun
    if args.save:
        rr.init("IK_Benchmark_Viewer", spawn=False)
        rr.save(args.save)
        print(f"数据将保存到: {args.save}")
    else:
        rr.init("IK_Benchmark_Viewer", spawn=True)
        print("Rerun 查看器已启动（关闭查看器窗口即可退出）")

    # 加载两个独立的 URDF 实例（target / actual 各一份）
    urdf_actual = load_urdf(urdf_path)

    # 逐帧回放
    for rec in records:
        idx = rec["index"]
        rr.set_time("step", sequence=idx)

        solver = rec["solver"]
        status = rec["status"]
        ok = (status != "failed")

        # ── 1. FK 原始末端位姿（小球：绿色=左臂，橙色=右臂）────────────────
        left_fk  = jsonl_pose_to_dict(rec.get("left_fk_pose"))
        right_fk = jsonl_pose_to_dict(rec.get("right_fk_pose"))
        if left_fk:
            log_end_effector("fk/left", left_fk, color=[0, 200, 0],
                             radius=0.005, axis_length=0.03)
        else:
            rr.log("fk/left", rr.Clear(recursive=True))
        if right_fk:
            log_end_effector("fk/right", right_fk, color=[200, 130, 0],
                             radius=0.005, axis_length=0.03)
        else:
            rr.log("fk/right", rr.Clear(recursive=True))

        # ── 2. 扰动后的 IK 目标位姿（大球：青色=左臂，黄色=右臂）──────────
        left_target  = jsonl_pose_to_dict(rec.get("left_target_pose"))
        right_target = jsonl_pose_to_dict(rec.get("right_target_pose"))
        if left_target:
            log_end_effector("target/left", left_target, color=[0, 255, 255],
                             radius=0.010, axis_length=0.04)
        else:
            rr.log("target/left", rr.Clear(recursive=True))
        if right_target:
            log_end_effector("target/right", right_target, color=[255, 255, 0],
                             radius=0.010, axis_length=0.04)
        else:
            rr.log("target/right", rr.Clear(recursive=True))

        # ── 3. IK 求解后的机器人姿态（灰色实体机器人）──────────────────────
        if ok and "solved_joints" in rec and rec["solved_joints"]:
            apply_joints(urdf_actual, rec["solved_joints"])
            log_robot(urdf_actual, "actual_robot", [180, 180, 180, 255])

            # IK 求解后的实际末端位姿（中球：紫色=左臂，红色=右臂）
            left_actual  = jsonl_pose_to_dict(rec.get("left_actual_pose"))
            right_actual = jsonl_pose_to_dict(rec.get("right_actual_pose"))
            if left_actual:
                log_end_effector("actual/left", left_actual, color=[180, 0, 255],
                                 radius=0.007, axis_length=0.04)
            else:
                rr.log("actual/left", rr.Clear(recursive=True))
            if right_actual:
                log_end_effector("actual/right", right_actual, color=[255, 0, 80],
                                 radius=0.007, axis_length=0.04)
            else:
                rr.log("actual/right", rr.Clear(recursive=True))

            # 误差线：目标 → 实际
            # 位姿数据在 base_link 坐标系，需要转到 world 坐标系
            T_base = urdf_actual.scene.graph.get("base_link")[0]
            base_pos = T_base[:3, 3]
            base_rot = T_base[:3, :3]

            for side, tgt_pose, act_pose in [
                ("left", left_target, left_actual),
                ("right", right_target, right_actual),
            ]:
                if tgt_pose and act_pose:
                    t_world = (base_rot @ np.array(tgt_pose["pos"]) + base_pos).tolist()
                    a_world = (base_rot @ np.array(act_pose["pos"]) + base_pos).tolist()
                    dist = np.linalg.norm(np.array(t_world) - np.array(a_world))
                    if dist > 1e-6:
                        rr.log(
                            f"error_line/{side}",
                            rr.LineStrips3D(
                                strips=[[t_world, a_world]],
                                colors=[255, 50, 50, 200],
                            ),
                        )
                    else:
                        rr.log(f"error_line/{side}", rr.Clear(recursive=True))
                else:
                    rr.log(f"error_line/{side}", rr.Clear(recursive=True))
        else:
            rr.log("actual_robot", rr.Clear(recursive=True))
            rr.log("actual/left", rr.Clear(recursive=True))
            rr.log("actual/right", rr.Clear(recursive=True))
            rr.log("error_line/left", rr.Clear(recursive=True))
            rr.log("error_line/right", rr.Clear(recursive=True))

        # ── 4. 文本数据面板 ─────────────────────────────────────────────────
        metrics = (
            f"#{idx}  solver: {solver}\n"
            f"status: {status}\n"
            f"solve_ms: {rec['solve_ms']:.1f}\n"
            f"pos_error: {rec['pos_error']:.6f} m"
        )
        rr.log("metrics", rr.TextDocument(metrics, media_type="text/plain"))

    print(f"回放完成：{len(records)} 帧")


if __name__ == "__main__":
    main()