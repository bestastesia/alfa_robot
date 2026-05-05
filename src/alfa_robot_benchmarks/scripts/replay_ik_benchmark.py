#!/usr/bin/env python3
"""
IK Benchmark 3D Replay Viewer

基于 rerun-sdk + yourdfpy，将 ik_benchmark 生成的 .jsonl 压测结果
在 Rerun 时间轴上进行 3D 可视化回放。

用法：
  python3 replay_ik_benchmark.py result.jsonl
  python3 replay_ik_benchmark.py result.jsonl --urdf /path/to/alfa_robot.urdf
  python3 replay_ik_benchmark.py result.jsonl --count 10          # 随机抽 10 条
  python3 replay_ik_benchmark.py result.jsonl --count 20 --seed 42  # 可复现抽样

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
    """
    将 package://alfa_robot_description/meshes/xxx.STL
    解析为 install 目录下的绝对路径。
    yourdfpy 以关键字参数调用：filename_handler(fname=...)
    """
    prefix = "package://"
    if not fname.startswith(prefix):
        return fname

    rel = fname[len(prefix):]  # alfa_robot_description/meshes/...
    pkg_name = rel.split("/")[0]
    rest = rel[len(pkg_name) + 1 :]

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


# ── yourdfpy → Rerun 的桥接 ────────────────────────────────────────────────

def load_urdf(urdf_path: str) -> yourdfpy.URDF:
    """加载 URDF 并返回 yourdfpy 对象，自动解析 package:// URI"""
    p = Path(urdf_path)
    if not p.exists():
        print(f"错误：URDF 文件不存在: {urdf_path}", file=sys.stderr)
        sys.exit(1)
    print(f"加载 URDF: {p}")
    return yourdfpy.URDF.load(str(p), filename_handler=_resolve_package_path)


def get_link_geometries(urdf: yourdfpy.URDF):
    """
    遍历 yourdfpy 场景图，返回 [(node_name, transform_4x4, trimesh_mesh), ...]。
    所有几何体已转换为三角网格，顶点已变换到 world 坐标系。
    """
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

        # 变换顶点到 world 坐标系
        verts = mesh.vertices @ T[:3, :3].T + T[:3, 3]
        faces = mesh.faces

        result.append((node, T.copy(), verts.copy(), faces.copy()))

    return result


def apply_joints(urdf: yourdfpy.URDF, joints: list):
    """
    将关节角数组应用到 URDF 并更新 FK。
    JSONL 中的 target_joints/solved_joints 按 MoveIt JMG 的关节顺序排列，
    yourdfpy 的 cfg 按 actuated_joints 的顺序排列，两者不一致。
    必须按名称映射，不能直接用索引赋值。
    """
    yourdfpy_names = [j.name for j in urdf.actuated_joints]
    # MoveIt JMG 关节顺序（与 ik_benchmark.cpp 中 copyJointGroupPositions 一致）
    # dual_arm_with_base 组不含 turn，共 15 个关节
    moveit_names = [
        "updown",
        "leftarmbase", "leftjoint1", "leftjoint2", "leftjoint3",
        "leftjoint4", "leftjoint5", "leftjoint6",
        "rightarmbase", "rightjoint1", "rightjoint2", "rightjoint3",
        "rightjoint4", "rightjoint5", "rightjoint6",
    ]

    # 建立名称→yourdfpy索引的映射
    name_to_cfg_idx = {n: i for i, n in enumerate(yourdfpy_names)}

    # 重置 cfg 为 0（不在 moveit_names 中的关节如 plate 保持 0）
    urdf.cfg[:] = 0.0
    for mi, name in enumerate(moveit_names):
        if mi < len(joints) and name in name_to_cfg_idx:
            urdf.cfg[name_to_cfg_idx[name]] = float(joints[mi])
    urdf.update_cfg(urdf.cfg)


def log_robot(urdf: yourdfpy.URDF, entity_prefix: str, color: list[int]):
    """
    将当前关节配置下的机器人几何体发送到 Rerun。
    entity_prefix: "target_robot" 或 "actual_robot"
    color: RGBA 列表 [R, G, B, A]，值域 0-255
    """
    link_geoms = get_link_geometries(urdf)

    for node, _T, verts, faces in link_geoms:
        path = f"{entity_prefix}/{node}"

        # 顶点着色
        n_verts = len(verts)
        vertex_colors = np.tile(
            np.array(color, dtype=np.uint8), (n_verts, 1)
        )

        rr.log(
            path,
            rr.Mesh3D(
                vertex_positions=verts.astype(np.float32),
                triangle_indices=faces.astype(np.uint32),
                vertex_colors=vertex_colors,
            ),
        )


# ── JSONL 读取 ─────────────────────────────────────────────────────────────

def load_jsonl(path: str) -> list[dict]:
    """读取 .jsonl 文件，跳过 header 行，返回数据行列表"""
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


# ── 主流程 ──────────────────────────────────────────────────────────────────

def find_urdf() -> str | None:
    """自动查找 URDF 文件"""
    # 1. 常见路径
    candidates = [
        "/tmp/_alfa_bench.urdf",
        "alfa_robot.urdf",
        "robot.urdf",
    ]
    for c in candidates:
        if Path(c).exists():
            return c
    # 2. ROS2 包目录
    import subprocess
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
    # 3. 尝试 xacro 生成
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

    # 加载两个独立的 URDF 实例（target / actual 各一份，避免 FK 互相干扰）
    urdf_target = load_urdf(urdf_path)
    urdf_actual = load_urdf(urdf_path)

    # 逐帧回放
    for rec in records:
        idx = rec["index"]
        rr.set_time("step", sequence=idx)

        # ── 1. 目标机器人（幽灵态：半透明红色）────────────────────────────
        apply_joints(urdf_target, rec["target_joints"])
        log_robot(urdf_target, "target_robot", [255, 60, 60, 100])

        # ── 2. 实际解算机器人（实体态：不透明灰色）────────────────────────
        status = rec["status"]
        if status != "failed":
            apply_joints(urdf_actual, rec["solved_joints"])
            log_robot(urdf_actual, "actual_robot", [180, 180, 180, 255])
        else:
            # 失败时清除 actual_robot 实体
            rr.log("actual_robot", rr.Clear(recursive=True))

        # ── 3. 误差线（左臂目标 → 左臂实际，world 坐标系）────────────────
        if status != "failed":
            # left_target_pos / left_actual_pos 是 base_link 坐标系数据，
            # 需要转回 world 坐标系才能与 3D 模型对齐
            T_base = urdf_actual.scene.graph.get("base_link")[0]
            base_pos = T_base[:3, 3]
            base_rot = T_base[:3, :3]
            lt_world = (base_rot @ np.array(rec["left_target_pos"]) + base_pos).tolist()
            la_world = (base_rot @ np.array(rec["left_actual_pos"]) + base_pos).tolist()
            dist = np.linalg.norm(np.array(lt_world) - np.array(la_world))
            if dist > 1e-6:
                rr.log(
                    "error_line/left_arm",
                    rr.LineStrips3D(
                        strips=[[[lt_world[0], lt_world[1], lt_world[2]],
                                 [la_world[0], la_world[1], la_world[2]]]],
                        colors=[255, 50, 50, 200],
                    ),
                )
            else:
                rr.log("error_line/left_arm", rr.Clear(recursive=True))
        else:
            rr.log("error_line/left_arm", rr.Clear(recursive=True))

        # ── 4. 文本数据面板 ───────────────────────────────────────────────
        metrics = (
            f"#{idx}  solver: {rec['solver']}\n"
            f"status: {status}\n"
            f"solve_ms: {rec['solve_ms']:.1f}\n"
            f"pos_error: {rec['pos_error']:.6f} m"
        )
        rr.log("metrics", rr.TextDocument(metrics, media_type="text/plain"))

    print(f"回放完成：{len(records)} 帧")


if __name__ == "__main__":
    main()
