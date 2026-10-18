import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    def relocate(value):
        marker = "/generated/"
        if marker in value:
            return root / "generated" / value.split(marker, 1)[1]
        return Path(value)
    for directory in (root / "generated").iterdir():
        if not directory.is_dir():
            continue
        for path in directory.rglob("*.urdf"):
            tree = ET.parse(path)
            for mesh in tree.getroot().findall(".//mesh"):
                filename = mesh.attrib["filename"]
                if "/generated/" in filename:
                    destination = relocate(filename)
                    if not destination.is_file():
                        raise FileNotFoundError(destination)
                    mesh.set("filename", str(destination))
            tree.write(path, encoding="utf-8", xml_declaration=True)
        for path in (*directory.rglob("*.yml"), *directory.rglob("*.yaml")):
            data = yaml.safe_load(path.read_text())
            kin = data.get("robot_cfg", data).get("kinematics")
            if kin is None:
                continue
            for key in ("urdf_path", "asset_root_path"):
                value = kin.get(key, "")
                if "/generated/" in value:
                    kin[key] = str(relocate(value))
            path.write_text(yaml.safe_dump(data, sort_keys=False))
    vendor = root / "vendor/alfa_robot_analytic_ik"
    binary = root / "generated/v3_analytic_071cb95/libv3_analytic_bridge.so"
    subprocess.run([
        args.compiler, "-O3", "-std=c++17", "-shared", "-fPIC",
        "-I/usr/include/eigen3", "-I" + str(vendor / "include"),
        str(root / "tools/v3_analytic_bridge.cpp"),
        str(vendor / "src/v3_redundant_analytic_ik.cpp"),
        str(vendor / "src/analytic_ik.cpp"), "-o", str(binary),
    ], check=True)
    print(json.dumps({"workspace": str(root), "analytic_library": str(binary)}))


if __name__ == "__main__":
    main()
