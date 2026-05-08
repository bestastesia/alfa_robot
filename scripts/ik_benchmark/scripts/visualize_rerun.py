#!/usr/bin/env python3
"""
IK Benchmark 3D 可视化回放 — Rerun.io

读取 ik_benchmark 输出的 JSONL 文件，利用 Rerun 时间轴逐帧回放：
  - 每个样本显示: FK 原始末端、扰动后的目标位姿、IK 求解后的实际末端
  - 双臂: 左臂/右臂分别显示
  - 拖动时间轴即可回放

用法:
  python3 scripts/ik_benchmark/scripts/visualize_rerun.py /tmp/ik_benchmark.jsonl
  python3 scripts/ik_benchmark/scripts/visualize_rerun.py result.jsonl --connect  # 连接已有 viewer
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rerun as rr


def pose_from_json(p: dict) -> tuple[np.ndarray, np.ndarray]:
    """从 JSON pose 解析出 (position, quaternion_xyzw)"""
    pos = np.array(p["position"], dtype=np.float64)
    quat = np.array(p["orientation"], dtype=np.float64)  # [x, y, z, w]
    return pos, quat


def log_pose(entity_path: str, pos: np.ndarray, quat_xyzw: np.ndarray, label: str, color=None):
    """向 Rerun 记录一个位姿（箭头 + 点）"""
    # Rerun 用 [w, x, y, z] 格式
    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

    rr.log(
        entity_path + "/arrow",
        rr.Arrows3D(
            vectors=[0, 0, 0.05],  # 小箭头表示朝向
            origins=pos,
            colors=color or [200, 200, 200],
        ),
    )
    rr.log(
        entity_path + "/point",
        rr.Points3D(
            positions=[pos],
            colors=color or [200, 200, 200],
            radii=0.008,
        ),
    )
    rr.log(
        entity_path,
        rr.TextLog(label),
    )


def log_transform(entity_path: str, pos: np.ndarray, quat_xyzw: np.ndarray):
    """记录一个 rigid transform，Rerun 用这个做坐标系显示"""
    quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=pos,
            rotation=rr.Quaternion(xyzw=quat_xyzw.tolist()),
        ),
    )


def main():
    parser = argparse.ArgumentParser(description="IK Benchmark Rerun 可视化")
    parser.add_argument("jsonl_path", type=str, help="ik_benchmark 输出的 JSONL 文件")
    parser.add_argument("--connect", action="store_true", help="连接已有 Rerun viewer")
    parser.add_argument("--save", type=str, default="", help="保存为 .rrd 文件")
    args = parser.parse_args()

    # 读取 JSONL
    records = []
    with open(args.jsonl_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("header"):
                continue
            records.append(rec)

    if not records:
        print("No records found in", args.jsonl_path)
        return

    is_dual = "target_pose2" in records[0]
    print(f"Loaded {len(records)} samples, dual_arm={is_dual}")

    # 初始化 Rerun
    if args.save:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.save(args.save)
    elif args.connect:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.connect()
    else:
        rr.init("ik_benchmark", recording_id=Path(args.jsonl_path).stem)
        rr.spawn()

    # 设置时间轴
    rr.set_time_sequence("sample", 0)

    # 颜色定义
    COLOR_FK    = [0, 200, 100]    # 绿色: FK 原始位姿
    COLOR_TARGET = [255, 165, 0]   # 橙色: 扰动后目标位姿
    COLOR_ACTUAL = [50, 100, 255]  # 蓝色: IK 求解后实际位姿
    COLOR_FAIL   = [255, 50, 50]   # 红色: 求解失败

    for i, rec in enumerate(records):
        rr.set_time_sequence("sample", i)

        success = rec["result"]["success"] if "result" in rec else rec.get("success", False)
        solver = rec.get("solver", "")
        pos_err = rec.get("result", rec).get("pos_error", 0.0)
        ori_err = rec.get("result", rec).get("ori_error", 0.0)
        solve_ms = rec.get("result", rec).get("solve_ms", 0.0)

        # 状态文字
        status = "OK" if success else "FAIL"
        info = f"{solver} | {status} | {solve_ms:.1f}ms"
        if success:
            info += f" | pos_err={pos_err:.6f}m ori_err={ori_err:.6f}rad"
        rr.log("info", rr.TextLog(info))

        # ── 左臂 ──
        if "fk_pose" in rec:
            fk_pos, fk_quat = pose_from_json(rec["fk_pose"])
            log_pose("left/fk", fk_pos, fk_quat, "FK", COLOR_FK)

        if "target_pose" in rec:
            tgt_pos, tgt_quat = pose_from_json(rec["target_pose"])
            log_pose("left/target", tgt_pos, tgt_quat, "Target", COLOR_TARGET)

        if success and "actual_pose" in rec:
            act_pos, act_quat = pose_from_json(rec["actual_pose"])
            log_pose("left/actual", act_pos, act_quat, "Actual", COLOR_ACTUAL)
        else:
            # 求解失败: 用 target 位置标红
            if "target_pose" in rec:
                tgt_pos, tgt_quat = pose_from_json(rec["target_pose"])
                log_pose("left/actual", tgt_pos, tgt_quat, "FAILED", COLOR_FAIL)

        # ── 右臂（双臂模式）──
        if is_dual:
            if "fk_pose2" in rec:
                fk2_pos, fk2_quat = pose_from_json(rec["fk_pose2"])
                log_pose("right/fk", fk2_pos, fk2_quat, "FK", COLOR_FK)

            if "target_pose2" in rec:
                tgt2_pos, tgt2_quat = pose_from_json(rec["target_pose2"])
                log_pose("right/target", tgt2_pos, tgt2_quat, "Target", COLOR_TARGET)

            if success and "actual_pose2" in rec:
                act2_pos, act2_quat = pose_from_json(rec["actual_pose2"])
                log_pose("right/actual", act2_pos, act2_quat, "Actual", COLOR_ACTUAL)
            else:
                if "target_pose2" in rec:
                    tgt2_pos, tgt2_quat = pose_from_json(rec["target_pose2"])
                    log_pose("right/actual", tgt2_pos, tgt2_quat, "FAILED", COLOR_FAIL)

        # ── 连线: target → actual（误差可视化）──
        if success and "target_pose" in rec and "actual_pose" in rec:
            tgt_pos = np.array(rec["target_pose"]["position"])
            act_pos = np.array(rec["actual_pose"]["position"])
            rr.log(
                "left/error_line",
                rr.LineStrips3D(
                    strips=[[tgt_pos.tolist(), act_pos.tolist()]],
                    colors=[COLOR_ACTUAL],
                ),
            )
            if is_dual and "target_pose2" in rec and "actual_pose2" in rec:
                tgt2_pos = np.array(rec["target_pose2"]["position"])
                act2_pos = np.array(rec["actual_pose2"]["position"])
                rr.log(
                    "right/error_line",
                    rr.LineStrips3D(
                        strips=[[tgt2_pos.tolist(), act2_pos.tolist()]],
                        colors=[COLOR_ACTUAL],
                    ),
                )

    print(f"Done. {len(records)} samples logged to Rerun.")


if __name__ == "__main__":
    main()
