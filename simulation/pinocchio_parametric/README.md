# ALFA v5 Pinocchio 参数化模型骨架

此目录对应协作任务 T-0016，用于离线生成参数化 URDF，并在安装 Pinocchio/MeshCat 后进行模型加载与浏览器可视化验证。

重要边界：这里不是 ROS2 主模型事实源，不直接修改 `alfa_robot_description`。机械工程师完成 T-0015 前，`configs/v5_baseline_stub.yaml` 中的 baseline 仍视为待校准。

## 文件结构

- `configs/v5_baseline_stub.yaml`：当前 v5 参数化 baseline 草案，字段显式暴露给机械工程师校准。
- `templates/alfa_v5_parametric.urdf.j2`：Jinja2 URDF 模板。
- `tools/generate_model.py`：生成 URDF、XML 检查、可选 Pinocchio 加载、可选 MeshCat 可视化。
- `generated/`：临时生成 URDF 输出目录，默认不应提交生成物。
- `MECHANICAL_HANDOFF.md`：运行生成脚本后自动生成的机械交接清单。

## 快速运行

```bash
cd /mnt/mydisk/ALFA/alfa_robot
python3 simulation/pinocchio_parametric/tools/generate_model.py --check
```

如果环境安装了 Pinocchio，`--check` 会自动加载模型并打印 `nq/nv/frame` 信息；未安装时会跳过 Pinocchio 检查，但仍完成 URDF 生成和 XML 基础检查。

安装 MeshCat 后可运行：

```bash
python3 simulation/pinocchio_parametric/tools/generate_model.py --check --visualize
```

## 运动学语义校验

T-0017 运控侧提供了独立校验脚本：

```bash
/usr/bin/python3 simulation/pinocchio_parametric/tools/validate_kinematics.py
```

脚本会重新生成 URDF，并校验：

- `left_v5_*` / `right_v5_*` joint、link、tool frame 命名和父子链；
- mount、joint origin/rpy、axis、limit 是否与 YAML baseline 一致；
- 不依赖 Pinocchio 的轻量 FK、position Jacobian、position IK smoke test；
- 如果本机安装 Pinocchio，还会额外加载 Pinocchio model，检查 frame Jacobian、IK smoke 和 RNEA 输出。

报告输出到：`generated/kinematic_semantics_report.md`。

## 当前骨架语义

- 生成双臂模型：`left_v5_*` 与 `right_v5_*`。
- 每臂 6 个 revolute joint，命名保持 `*_joint1..6`。
- 末端 frame 保持 `*_tool0`，方便后续运控侧校验 FK/Jacobian/IK/RNEA。
- 右臂默认按 Y 镜像规则生成；机械工程师需确认真实模型是否严格满足。

## 给机械工程师

请优先校准 `configs/v5_baseline_stub.yaml`：

1. `mount.*_xyz`：左右臂安装基准点。
2. `kinematics.joint*`：各关节 origin/rpy/axis/limit。
3. `links.*`：质量、COM、惯量。
4. `visual`：如果后续需要近似几何体尺寸，可以替换 primitive 尺寸或增加 mesh 映射。

运行脚本后会生成 `MECHANICAL_HANDOFF.md`，其中列出当前需要机械侧确认的字段。
