# robot_v3.0.1 双臂网格

- `left/`：来自 `/mnt/mydisk/ALFA/backpack/robot_v3.0.1/meshes` 的低面数碰撞网格。
- `right/`：左侧碰撞网格沿机械臂局部 `Y=0` 平面镜像，并同步修正三角面绕序。
- `visual/left/`：来自 `/mnt/mydisk/ALFA/backpack/robot_v3.0.1_visual/meshes` 的高精度外观网格。
- `visual/right/`：左侧外观网格沿机械臂局部 `Y=0` 平面镜像，并同步修正三角面绕序。
- 关节参数采用外观版 URDF；现有低模网格经过逐链接刚体配准后仅用于碰撞检测。
- 末端工具坐标暂时与第 7 关节坐标重合。
