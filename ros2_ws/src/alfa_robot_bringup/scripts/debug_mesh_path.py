#!/usr/bin/env python3
# #region agent log
"""One-off diagnostic: check alfa_robot_description mesh/collision paths at runtime."""
import json
import os

LOG_PATH = "/home/kkozia/franka_ros2/.cursor/debug.log"

def main():
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("alfa_robot_description")
    except Exception as e:
        entry = {"hypothesisId": "H2", "message": "get_package_share_directory failed", "data": {"error": str(e)}, "sessionId": "debug-session"}
        with open(LOG_PATH, "a") as f:
            f.write(json.dumps(entry) + "\n")
        return

    mesh_base = os.path.join(pkg, "meshes", "current_robot")
    collision_file = os.path.join(mesh_base, "collision", "rightjoint1.STL")
    list_collision = []
    list_visual = []
    if os.path.isdir(mesh_base):
        for name in sorted(os.listdir(mesh_base)):
            sub = os.path.join(mesh_base, name)
            if os.path.isdir(sub):
                if name == "collision":
                    list_collision = sorted(os.listdir(sub))
                elif name == "visual":
                    list_visual = sorted(os.listdir(sub))

    entry = {
        "hypothesisId": "H1",
        "message": "mesh_path_check",
        "data": {
            "pkg_share": pkg,
            "mesh_base_exists": os.path.isdir(mesh_base),
            "collision_file_exists": os.path.isfile(collision_file),
            "collision_dir_list": list_collision,
            "visual_dir_list": list_visual,
        },
        "sessionId": "debug-session",
    }
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps(entry) + "\n")

if __name__ == "__main__":
    main()
# #endregion
