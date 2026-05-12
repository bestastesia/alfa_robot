#!/usr/bin/python3
"""Validate kinematic semantics of the ALFA v5 parametric Pinocchio model.

This is the T-0017 motion-control check. It always validates the generated URDF
structure against the calibrated YAML baseline. If Pinocchio is available, it
also runs FK, frame Jacobian, and a small damped-least-squares IK smoke test.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "v5_baseline_stub.yaml"
DEFAULT_URDF = ROOT / "generated" / "alfa_v5_parametric.urdf"
DEFAULT_REPORT = ROOT / "generated" / "kinematic_semantics_report.md"


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open() as stream:
        return yaml.safe_load(stream)


def parse_vector(text: str | None, size: int = 3) -> np.ndarray:
    if not text:
        return np.zeros(size)
    values = np.array([float(value) for value in text.split()], dtype=float)
    if values.shape != (size,):
        raise ValueError(f"expected vector size {size}, got {text!r}")
    return values


def mirror_y(values: np.ndarray) -> np.ndarray:
    mirrored = values.astype(float).copy()
    mirrored[1] *= -1.0
    return mirrored


def rot_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    return rot_z(float(rpy[2])) @ rot_y(float(rpy[1])) @ rot_x(float(rpy[0]))


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ])


def transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def revolute_transform(axis: np.ndarray, value: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = axis_angle(axis, value)
    return result


def expected_joint_xyz(joint: dict[str, Any], mirror: float) -> np.ndarray:
    key = "xyz_mirror" if "xyz_mirror" in joint else "xyz"
    values = np.array(joint[key], dtype=float)
    if key == "xyz_mirror":
        values[1] *= mirror
    return values


def expected_joint_rpy(joint: dict[str, Any], mirror: float) -> np.ndarray:
    if "rpy_mirror" in joint:
        values = np.array(joint["rpy_mirror"], dtype=float)
        values[0] *= mirror
        values[2] *= mirror
        return values
    return np.array(joint.get("rpy", [0.0, 0.0, 0.0]), dtype=float)


def parse_urdf_joints(path: Path) -> dict[str, JointSpec]:
    root = ET.parse(path).getroot()
    result: dict[str, JointSpec] = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        result[name] = JointSpec(
            name=name,
            parent=parent,
            child=child,
            origin_xyz=parse_vector(origin.attrib.get("xyz") if origin is not None else None),
            origin_rpy=parse_vector(origin.attrib.get("rpy") if origin is not None else None),
            axis=parse_vector(axis.attrib.get("xyz") if axis is not None else None),
            lower=float(limit.attrib["lower"]) if limit is not None and "lower" in limit.attrib else 0.0,
            upper=float(limit.attrib["upper"]) if limit is not None and "upper" in limit.attrib else 0.0,
        )
    return result


def assert_close(name: str, actual: np.ndarray, expected: np.ndarray, tol: float, errors: list[str]) -> None:
    if not np.allclose(actual, expected, atol=tol, rtol=0.0):
        errors.append(f"{name}: expected {expected.tolist()}, got {actual.tolist()}")


def run_generator(config: Path, urdf: Path) -> None:
    cmd = [
        sys.executable,
        str(ROOT / "tools" / "generate_model.py"),
        "--config",
        str(config),
        "--output",
        str(urdf),
        "--check",
    ]
    subprocess.run(cmd, check=True)


def validate_static_semantics(config: dict[str, Any], urdf: Path, tol: float) -> list[str]:
    errors: list[str] = []
    joints = parse_urdf_joints(urdf)

    expected_revolute = []
    for side_name, arm in config["arms"].items():
        prefix = arm["prefix"]
        mirror = float(arm["mirror_y"])
        mount_name = f"{prefix}_mount"
        if mount_name not in joints:
            errors.append(f"missing fixed mount joint: {mount_name}")
        else:
            mount = joints[mount_name]
            expected_parent = config["mount"]["parent_link"]
            expected_child = f"{prefix}_link0"
            expected_mount = np.array(config["mount"][f"{side_name}_xyz"], dtype=float)
            if mount.parent != expected_parent or mount.child != expected_child:
                errors.append(f"{mount_name}: expected {expected_parent}->{expected_child}, got {mount.parent}->{mount.child}")
            assert_close(f"{mount_name}.origin_xyz", mount.origin_xyz, expected_mount, tol, errors)

        for index in range(1, 7):
            name = f"{prefix}_joint{index}"
            expected_revolute.append(name)
            if name not in joints:
                errors.append(f"missing revolute joint: {name}")
                continue
            spec = joints[name]
            expected_parent = f"{prefix}_link{index - 1}"
            expected_child = f"{prefix}_link{index}"
            if spec.parent != expected_parent or spec.child != expected_child:
                errors.append(f"{name}: expected {expected_parent}->{expected_child}, got {spec.parent}->{spec.child}")
            joint_cfg = config["kinematics"][f"joint{index}"]
            assert_close(f"{name}.origin_xyz", spec.origin_xyz, expected_joint_xyz(joint_cfg, mirror), tol, errors)
            assert_close(f"{name}.origin_rpy", spec.origin_rpy, expected_joint_rpy(joint_cfg, mirror), tol, errors)
            assert_close(f"{name}.axis", spec.axis, np.array(joint_cfg["axis"], dtype=float), tol, errors)
            expected_limit = np.array(joint_cfg["limit"], dtype=float)
            actual_limit = np.array([spec.lower, spec.upper], dtype=float)
            assert_close(f"{name}.limit", actual_limit, expected_limit, tol, errors)

        tool_name = f"{prefix}_tool0_fixed"
        if tool_name not in joints:
            errors.append(f"missing tool fixed joint: {tool_name}")
        else:
            tool = joints[tool_name]
            if tool.parent != f"{prefix}_link6" or tool.child != f"{prefix}_tool0":
                errors.append(f"{tool_name}: expected {prefix}_link6->{prefix}_tool0, got {tool.parent}->{tool.child}")
            assert_close(
                f"{tool_name}.origin_xyz",
                tool.origin_xyz,
                np.array(config["kinematics"]["tool0_fixed"]["xyz"], dtype=float),
                tol,
                errors,
            )

    revolute_count = sum(1 for name in joints if name in expected_revolute)
    if revolute_count != 12:
        errors.append(f"expected 12 v5 revolute joints, got {revolute_count}")

    for index in range(0, 7):
        left = f"left_v5_link{index}"
        right = f"right_v5_link{index}"
        if left == right:
            continue
    return errors


def arm_chain_from_urdf(joints: dict[str, JointSpec], prefix: str) -> list[JointSpec]:
    return [joints[f"{prefix}_joint{index}"] for index in range(1, 7)]


def fk_position(joints: dict[str, JointSpec], prefix: str, q: np.ndarray) -> np.ndarray:
    mount = joints[f"{prefix}_mount"]
    tool = joints[f"{prefix}_tool0_fixed"]
    pose = transform(mount.origin_xyz, mount.origin_rpy)
    for value, joint in zip(q, arm_chain_from_urdf(joints, prefix)):
        pose = pose @ transform(joint.origin_xyz, joint.origin_rpy)
        pose = pose @ revolute_transform(joint.axis, float(value))
    pose = pose @ transform(tool.origin_xyz, tool.origin_rpy)
    return pose[:3, 3].copy()


def numeric_jacobian(joints: dict[str, JointSpec], prefix: str, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    base = fk_position(joints, prefix, q)
    jac = np.zeros((3, 6))
    for index in range(6):
        q_step = q.copy()
        q_step[index] += eps
        jac[:, index] = (fk_position(joints, prefix, q_step) - base) / eps
    return jac


def damped_position_ik(joints: dict[str, JointSpec], prefix: str, target: np.ndarray) -> tuple[bool, float]:
    q = np.zeros(6)
    err_norm = math.inf
    for _ in range(120):
        err = target - fk_position(joints, prefix, q)
        err_norm = float(np.linalg.norm(err))
        if err_norm < 1e-4:
            return True, err_norm
        jac = numeric_jacobian(joints, prefix, q)
        damping = 1e-5
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(3), err)
        q += np.clip(0.35 * dq, -0.15, 0.15)
    return False, err_norm


def validate_lightweight_kinematics(urdf: Path, report: list[str]) -> list[str]:
    errors: list[str] = []
    joints = parse_urdf_joints(urdf)
    report.append("- Lightweight FK/Jacobian/position-IK checks: enabled without Pinocchio")
    for prefix, delta in (("left_v5", np.array([0.025, 0.02, 0.015])), ("right_v5", np.array([0.025, -0.02, 0.015]))):
        q0 = np.zeros(6)
        xyz = fk_position(joints, prefix, q0)
        if not finite(xyz):
            errors.append(f"{prefix} FK produced non-finite position")
            continue
        jac = numeric_jacobian(joints, prefix, q0)
        rank = int(np.linalg.matrix_rank(jac, tol=1e-7))
        report.append(f"- {prefix} neutral FK xyz={xyz.round(6).tolist()}, position Jacobian shape={jac.shape}, rank={rank}")
        if jac.shape != (3, 6) or not finite(jac):
            errors.append(f"{prefix} numeric Jacobian invalid: shape={jac.shape}, finite={finite(jac)}")
        if rank < 3:
            errors.append(f"{prefix} position Jacobian rank too low at neutral: {rank}")
        converged, err_norm = damped_position_ik(joints, prefix, xyz + delta)
        report.append(f"- {prefix} lightweight position IK smoke: converged={converged}, err_norm={err_norm:.3e}")
        if not converged:
            errors.append(f"{prefix} lightweight IK did not converge: err={err_norm:.3e}")
    return errors


def finite(value: np.ndarray) -> bool:
    return bool(np.all(np.isfinite(value)))


def pinocchio_semantics(urdf: Path, report: list[str], tol: float) -> list[str]:
    errors: list[str] = []
    import pinocchio as pin

    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    q0 = pin.neutral(model)
    pin.forwardKinematics(model, data, q0)
    pin.updateFramePlacements(model, data)

    report.append(f"- Pinocchio loaded: nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={model.nframes}")
    if model.nq != 12 or model.nv != 12:
        errors.append(f"expected nq=nv=12 for dual six-axis arms, got nq={model.nq}, nv={model.nv}")

    for name in [f"left_v5_joint{i}" for i in range(1, 7)] + [f"right_v5_joint{i}" for i in range(1, 7)]:
        if not model.existJointName(name):
            errors.append(f"Pinocchio joint missing: {name}")

    for frame_name in ("left_v5_tool0", "right_v5_tool0"):
        if not model.existFrame(frame_name):
            errors.append(f"Pinocchio frame missing: {frame_name}")
            continue
        frame_id = model.getFrameId(frame_name)
        pose = data.oMf[frame_id]
        report.append(f"- neutral {frame_name} xyz={pose.translation.round(6).tolist()}")
        jac = pin.computeFrameJacobian(model, data, q0, frame_id, pin.LOCAL_WORLD_ALIGNED)
        if jac.shape != (6, model.nv) or not finite(jac):
            errors.append(f"{frame_name} Jacobian invalid: shape={jac.shape}, finite={finite(jac)}")
        linear_rank = int(np.linalg.matrix_rank(jac[:3, :], tol=1e-7))
        report.append(f"- neutral {frame_name} Jacobian shape={jac.shape}, linear_rank={linear_rank}")
        if linear_rank < 3:
            errors.append(f"{frame_name} linear Jacobian rank too low at neutral: {linear_rank}")

    for frame_name, delta in (("left_v5_tool0", np.array([0.03, 0.02, 0.02])), ("right_v5_tool0", np.array([0.03, -0.02, 0.02]))):
        if not model.existFrame(frame_name):
            continue
        frame_id = model.getFrameId(frame_name)
        target = data.oMf[frame_id].copy()
        target.translation += delta
        q = q0.copy()
        converged = False
        err_norm = math.inf
        for _ in range(80):
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            current = data.oMf[frame_id]
            err6 = pin.log6(current.inverse() * target).vector
            err_norm = float(np.linalg.norm(err6))
            if err_norm < 1e-4:
                converged = True
                break
            jac = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL)
            damping = 1e-6
            dq = -jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(6), err6)
            q = pin.integrate(model, q, 0.25 * dq)
        report.append(f"- IK smoke {frame_name}: converged={converged}, err_norm={err_norm:.3e}")
        if not converged:
            errors.append(f"IK smoke did not converge for {frame_name}: err={err_norm:.3e}")

    tau = pin.rnea(model, data, q0, np.zeros(model.nv), np.zeros(model.nv))
    if tau.shape != (model.nv,) or not finite(tau):
        errors.append(f"RNEA output invalid: shape={tau.shape}, finite={finite(tau)}")
    else:
        report.append(f"- RNEA neutral gravity torque norm={float(np.linalg.norm(tau)):.6f}")

    return errors


def write_report(path: Path, report: list[str], errors: list[str], pinocchio_available: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# T-0017 Pinocchio 运动学语义校验报告",
        "",
        "## 结果",
        "",
        f"- Pinocchio available: `{pinocchio_available}`",
        f"- Status: `{'PASS' if not errors else 'FAIL'}`",
        "",
        "## 检查项",
        "",
        *report,
        "",
        "## 错误",
        "",
    ]
    if errors:
        lines.extend(f"- {error}" for error in errors)
    else:
        lines.append("- 无")
    lines.extend([
        "",
        "## 给机械工程师",
        "",
        "- 当前参数化模型根为 `world`，未包含 ROS2 主模型的 `base_link -> pitch -> turn -> updown` 底座链。",
        "- 运控侧已校验命名、父子拓扑、mount、joint origin/rpy/axis/limit 与 YAML baseline 一致。",
        "- 如果后续机械继续改臂长、offset、axis 或 tool0，请先更新 `configs/v5_baseline_stub.yaml`，再运行本脚本。",
        "- 当前模板仍只使用 `inertia_diag`，YAML 中 `full_inertia` 尚未进入 URDF 模板；若要做高可信动力学，请机械/仿真侧升级模板。",
    ])
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--skip-generate", action="store_true")
    parser.add_argument("--require-pinocchio", action="store_true")
    parser.add_argument("--tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    if not args.skip_generate:
        run_generator(args.config, args.urdf)

    config = load_yaml(args.config)
    report = [
        f"- Generated URDF: `{args.urdf}`",
        "- Static URDF/YAML semantic checks: joint names, parent/child chain, mount, origin xyz/rpy, axis, limits",
    ]
    errors = validate_static_semantics(config, args.urdf, args.tolerance)
    errors.extend(validate_lightweight_kinematics(args.urdf, report))

    pinocchio_available = importlib.util.find_spec("pinocchio") is not None
    if pinocchio_available:
        errors.extend(pinocchio_semantics(args.urdf, report, args.tolerance))
    else:
        message = "Pinocchio is not installed; FK/Jacobian/IK/RNEA numeric checks were skipped."
        report.append(f"- WARNING: {message}")
        if args.require_pinocchio:
            errors.append(message)

    write_report(args.report, report, errors, pinocchio_available)
    print(f"[OK] wrote kinematic semantics report: {args.report}")
    if errors:
        for error in errors:
            print(f"[ERROR] {error}")
        return 1
    print("[OK] kinematic semantics validation passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
