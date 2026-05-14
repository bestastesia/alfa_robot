#!/usr/bin/env python3
"""Adjust v5 arm big/little cylinder DH ratio while keeping total length constant.

Usage (from project root /mnt/mydisk/ALFA/alfa_robot):
  python3 scripts/adjust_arm_ratio.py              # default 1:1
  python3 scripts/adjust_arm_ratio.py 2            # big:little = 2:1
  python3 scripts/adjust_arm_ratio.py 1.5          # big:little = 1.5:1
  python3 scripts/adjust_arm_ratio.py --big 0.4 --little 0.2   # explicit lengths
  python3 scripts/adjust_arm_ratio.py --build      # also run colcon build after

Since both arms share the same xacro macro, all changes apply to both sides.
Total (big + little) is preserved by default; use --big/--little to override.
"""

import re
import subprocess
import sys
import os

XACRO_PATH = os.path.join(os.path.dirname(__file__),
                          "..", "ros2_ws", "src", "alfa_robot_description",
                          "urdf", "alfa_robot.urdf.xacro")

# Original SolidWorks values
ORIGINAL_BIG = 0.384
ORIGINAL_LITTLE = 0.211
ORIGINAL_TOTAL = ORIGINAL_BIG + ORIGINAL_LITTLE  # 0.595


def apply_lengths(text: str, big: float, little: float) -> str:
    big_h = f"{big / 2:.4f}"
    little_h = f"{little / 2:.4f}"
    big_s = f"{big:.4f}"
    little_s = f"{little:.4f}"

    # ---- big_arm (inside v5_arm macro) ----
    # The big_arm link block is the ONLY place in the xacro that has:
    #   <link name="${side}_v5_big_arm">
    # We replace values within that block only.

    # big_arm inertial origin z (e.g. xyz="0 0 0.192" → xyz="0 0 {big_h}")
    # This is the first <origin> inside big_arm's <inertial>
    text = re.sub(
        r'(<link name="\$\{side\}_v5_big_arm">\s*<inertial>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, big_h),
        text, count=1)

    # big_arm visual origin z + cylinder length
    text = re.sub(
        r'(<link name="\$\{side\}_v5_big_arm">.*?<visual>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, big_h),
        text, count=1, flags=re.DOTALL)
    text = re.sub(
        r'(<link name="\$\{side\}_v5_big_arm">.*?<visual>.*?<cylinder radius="0\.05" length=")([^"]+)(")',
        rf'\g<1>{big_s}\3',
        text, count=1, flags=re.DOTALL)

    # big_arm collision origin z + cylinder length
    text = re.sub(
        r'(<link name="\$\{side\}_v5_big_arm">.*?<collision>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, big_h),
        text, count=1, flags=re.DOTALL)
    text = re.sub(
        r'(<link name="\$\{side\}_v5_big_arm">.*?<collision>.*?<cylinder radius="0\.05" length=")([^"]+)(")',
        rf'\g<1>{big_s}\3',
        text, count=1, flags=re.DOTALL)

    # big_arm_2_fixed joint z-offset
    text = re.sub(
        r'(<joint name="\$\{side\}_v5_big_arm_2_fixed".*?<origin xyz=")0 0 (\d+\.?\d*)(")',
        rf'\g<1>0 0 {big_s}\3',
        text, count=1, flags=re.DOTALL)

    # ---- little_arm ----
    text = re.sub(
        r'(<link name="\$\{side\}_v5_little_arm">\s*<inertial>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, little_h),
        text, count=1)

    text = re.sub(
        r'(<link name="\$\{side\}_v5_little_arm">.*?<visual>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, little_h),
        text, count=1, flags=re.DOTALL)
    text = re.sub(
        r'(<link name="\$\{side\}_v5_little_arm">.*?<visual>.*?<cylinder radius="0\.05" length=")([^"]+)(")',
        rf'\g<1>{little_s}\3',
        text, count=1, flags=re.DOTALL)

    text = re.sub(
        r'(<link name="\$\{side\}_v5_little_arm">.*?<collision>\s*<origin xyz=")([^"]+)(")',
        lambda m: _set_z(m, little_h),
        text, count=1, flags=re.DOTALL)
    text = re.sub(
        r'(<link name="\$\{side\}_v5_little_arm">.*?<collision>.*?<cylinder radius="0\.05" length=")([^"]+)(")',
        rf'\g<1>{little_s}\3',
        text, count=1, flags=re.DOTALL)

    # joint4 z-offset
    text = re.sub(
        r'(<joint name="\$\{side\}_v5_joint4".*?<origin xyz=")0 0 (\d+\.?\d*)(")',
        rf'\g<1>0 0 {little_s}\3',
        text, count=1, flags=re.DOTALL)

    return text


def _set_z(match, new_z: str):
    """Replace the z component in xyz="x y z"."""
    parts = match.group(2).split()
    parts[-1] = new_z
    return match.group(1) + ' '.join(parts) + match.group(3)


def main():
    ratio = None
    big = None
    little = None
    do_build = False

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '--big' and i + 1 < len(args):
            big = float(args[i + 1]); i += 2
        elif args[i] == '--little' and i + 1 < len(args):
            little = float(args[i + 1]); i += 2
        elif args[i] == '--build':
            do_build = True; i += 1
        else:
            try:
                ratio = float(args[i])
            except ValueError:
                print(f"Unknown argument: {args[i]}"); sys.exit(1)
            i += 1

    if big is not None and little is not None:
        pass
    elif ratio is not None:
        little = ORIGINAL_TOTAL / (ratio + 1)
        big = ratio * little
    else:
        little = ORIGINAL_TOTAL / 2.0
        big = little

    ratio_display = big / little if little != 0 else float('inf')

    with open(XACRO_PATH, 'r') as f:
        content = f.read()

    content = apply_lengths(content, big, little)

    with open(XACRO_PATH, 'w') as f:
        f.write(content)

    print(f"big={big:.4f}m  little={little:.4f}m  ratio={ratio_display:.2f}:1  "
          f"total={big+little:.4f}m (orig {ORIGINAL_TOTAL:.4f}m)")

    if do_build:
        ws = os.path.join(os.path.dirname(__file__), "..", "ros2_ws")
        print("Building...")
        r = subprocess.run(["colcon", "build", "--packages-select",
                           "alfa_robot_description", "alfa_robot_moveit_config"],
                          cwd=ws)
        if r.returncode == 0:
            print("Build OK")
        else:
            print("Build FAILED"); sys.exit(1)


if __name__ == '__main__':
    main()
