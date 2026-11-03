#!/usr/bin/env python3
"""Overlay mass/COM/inertia from a robot_description bundle onto the active cuRobo proxy."""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import numpy as np
from scipy.spatial.transform import Rotation


DYNAMIC_LINKS = ("arm_carriage", *(f"{side}_link{index}" for side in ("left", "right") for index in range(1, 8)),
                 "left_tool0", "right_tool0")


def numbers(text):
    return np.fromstring(text or "0 0 0", sep=" ")


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = numbers(element.get("xyz", "0 0 0"))
        transform[:3, :3] = Rotation.from_euler("xyz", numbers(element.get("rpy", "0 0 0"))).as_matrix()
    return transform


def frames(robot):
    pending = list(robot.findall("joint"))
    children = {joint.find("child").get("link") for joint in pending}
    output = {link.get("name"): np.eye(4) for link in robot.findall("link") if link.get("name") not in children}
    while pending:
        ready = [joint for joint in pending if joint.find("parent").get("link") in output]
        if not ready:
            raise ValueError("disconnected URDF")
        for joint in ready:
            output[joint.find("child").get("link")] = output[joint.find("parent").get("link")] @ origin(joint.find("origin"))
            pending.remove(joint)
    return output


def inertia_from_urdf(link):
    inertial = link.find("inertial")
    if inertial is None:
        return None
    mass = float(inertial.find("mass").get("value"))
    pose = origin(inertial.find("origin"))
    item = inertial.find("inertia")
    tensor = np.array([[float(item.get("ixx")), float(item.get("ixy")), float(item.get("ixz"))],
                       [float(item.get("ixy")), float(item.get("iyy")), float(item.get("iyz"))],
                       [float(item.get("ixz")), float(item.get("iyz")), float(item.get("izz"))]])
    return mass, pose[:3, 3], pose[:3, :3] @ tensor @ pose[:3, :3].T


def tool_inertia_from_mjcf(path, name):
    body = next(item for item in ET.parse(path).getroot().iter("body") if item.get("name") == name)
    inertial = body.find("inertial")
    values = numbers(inertial.get("fullinertia"))
    tensor = np.array([[values[0], values[3], values[4]], [values[3], values[1], values[5]],
                       [values[4], values[5], values[2]]])
    quat = numbers(inertial.get("quat", "1 0 0 0"))
    rotation = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
    return float(inertial.get("mass")), numbers(inertial.get("pos")), rotation @ tensor @ rotation.T


def write_inertia(link, mass, center, tensor):
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=" ".join(f"{value:.16g}" for value in center), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.16g}")
    ET.SubElement(inertial, "inertia", ixx=f"{tensor[0,0]:.16g}", ixy=f"{tensor[0,1]:.16g}",
                  ixz=f"{tensor[0,2]:.16g}", iyy=f"{tensor[1,1]:.16g}",
                  iyz=f"{tensor[1,2]:.16g}", izz=f"{tensor[2,2]:.16g}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--proxy-urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--joint-torque-profile", type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256(args.bundle.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory() as directory, ZipFile(args.bundle) as archive:
        names = {item.filename for item in archive.infolist()}
        for required in ("robot_description/urdf/robot.urdf", "robot_description/mujoco/robot.xml"):
            if required not in names:
                raise FileNotFoundError(required)
        archive.extract("robot_description/urdf/robot.urdf", directory)
        archive.extract("robot_description/mujoco/robot.xml", directory)
        source_urdf = Path(directory) / "robot_description/urdf/robot.urdf"
        source_mjcf = Path(directory) / "robot_description/mujoco/robot.xml"
        source = ET.parse(source_urdf)
        target = ET.parse(args.proxy_urdf)
        source_links = {link.get("name"): link for link in source.getroot().findall("link")}
        target_links = {link.get("name"): link for link in target.getroot().findall("link")}
        torque_profile = None
        if args.joint_torque_profile:
            torque_profile = json.loads(args.joint_torque_profile.read_text())
            joints = {joint.get("name"): joint for joint in target.getroot().findall("joint")}
            for number, profile in torque_profile["joint_groups"].items():
                for side in ("left", "right"):
                    joints[f"{side}_joint{number}"].find("limit").set(
                        "effort", str(profile["start_stop_peak_torque_nm"]))
        source_frames, target_frames = frames(source.getroot()), frames(target.getroot())
        records = []
        for name in DYNAMIC_LINKS:
            if name.endswith("tool0"):
                values = tool_inertia_from_mjcf(source_mjcf, name)
            else:
                values = inertia_from_urdf(source_links[name])
            if values is None:
                raise ValueError(f"source has no inertia for {name}")
            mass, center, tensor = values
            source_world = source_frames[name]
            target_world = target_frames[name]
            world_center = source_world[:3, :3] @ center + source_world[:3, 3]
            target_center = target_world[:3, :3].T @ (world_center - target_world[:3, 3])
            rotation = target_world[:3, :3].T @ source_world[:3, :3]
            target_tensor = rotation @ tensor @ rotation.T
            eigenvalues = np.linalg.eigvalsh(target_tensor)
            if mass <= 0 or np.any(eigenvalues <= 0):
                raise ValueError(f"invalid dynamics for {name}: mass={mass}, inertia={eigenvalues}")
            write_inertia(target_links[name], mass, target_center, target_tensor)
            records.append({"link": name, "mass_kg": mass, "com_m": target_center.tolist(),
                            "inertia_eigenvalues_kg_m2": eigenvalues.tolist()})
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(target)
        target.write(args.output, encoding="utf-8", xml_declaration=True)
    document = {
        "schema_version": 1, "source_bundle": args.bundle.name, "source_bundle_sha256": digest,
        "source_urdf": "robot_description/urdf/robot.urdf",
        "source_mjcf": "robot_description/mujoco/robot.xml",
        "proxy_urdf": str(args.proxy_urdf), "output_urdf": str(args.output),
        "method": "zero-pose link-frame dynamics overlay; proxy kinematics/collision unchanged",
        "links": records,
        "joint_torque_profile": (torque_profile["source"] if torque_profile else None),
        "joint_effort_semantics": ("manufacturer start/stop peak output torque"
                                   if torque_profile else "inherited from proxy URDF"),
        "limitations": ["source bundle worktree is dirty relative to embedded commit",
                        "payload mass is not present in the URDF and must be supplied separately",
                        ("catalog torque profile must match installed joint nameplates and drive configuration"
                         if torque_profile else "source effort limits are not hardware verified")],
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "manifest": str(args.manifest),
                      "links": len(records), "mass_kg": sum(item["mass_kg"] for item in records)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
