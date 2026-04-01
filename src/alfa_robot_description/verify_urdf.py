#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import sys

def verify_xacro(file_path):
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        print(f"✓ {file_path} - XML语法正确")

        # 统计links和joints
        links = root.findall('.//{http://wiki.ros.org/xacro}macro/link') + root.findall('.//link')
        joints = root.findall('.//{http://wiki.ros.org/xacro}macro/joint') + root.findall('.//joint')

        print(f"  - Links: {len(links)}")
        print(f"  - Joints: {len(joints)}")

        # 列出所有link和joint名称
        link_names = []
        for link in links:
            name = link.get('name')
            if name and '${prefix}' in name:
                link_names.append(name.replace('${prefix}', ''))

        joint_names = []
        for joint in joints:
            name = joint.get('name')
            if name and '${prefix}' in name:
                joint_names.append(name.replace('${prefix}', ''))

        if link_names:
            print(f"\n  Links: {', '.join(sorted(set(link_names)))}")
        if joint_names:
            print(f"\n  Joints: {', '.join(sorted(set(joint_names)))}")

        return True
    except ET.ParseError as e:
        print(f"✗ {file_path} - XML语法错误:")
        print(f"  {e}")
        return False
    except Exception as e:
        print(f"✗ {file_path} - 错误: {e}")
        return False

if __name__ == "__main__":
    files = [
        "urdf/alfa_robot.urdf.xacro",
        "urdf/alfa_robot/alfa_robot_macro.xacro",
        "urdf/alfa_robot/alfa_robot_macro.ros2_control.xacro",
    ]

    all_ok = True
    for f in files:
        if not verify_xacro(f):
            all_ok = False
        print()

    if all_ok:
        print("✓ 所有文件验证通过")
        sys.exit(0)
    else:
        print("✗ 存在错误")
        sys.exit(1)
