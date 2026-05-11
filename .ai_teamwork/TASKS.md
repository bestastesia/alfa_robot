# 当前任务

## 使用方式

- 用户或项目经理把任务写到这里。
- AI 开工前看自己要做哪一条。
- 任务不要写太复杂，能说明目标、范围、交付物即可。

## 任务状态

- TODO：待做
- DOING：进行中
- BLOCKED：卡住
- DONE：完成

## 当前任务列表

| ID | 状态 | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- | --- |
| T-0002 | DONE | Git 操作工程师 | 提交 AI 轻量协作机制更改 | `AGENTS.md`, `CLAUDE.md`, `.ai_teamwork/`（含 `engineers/`） | 已提交协作相关文件，未带入 `lidar_ws/log/*` 等未归属改动 |
| T-0003 | DONE | 机械工程师 | 拆卸/移除当前 4 代机械臂配置 | `ros2_ws/src/alfa_robot_description/`、MoveIt/bringup 控制配置 | 已从主 URDF/MoveIt/RViz 路径真实移除左右机械臂 link/joint；未删除 STL/mesh；simulation 未改 |
| T-0004 | DONE | 运控工程师 | 定义原机械臂位置安装六轴机械臂的接口需求 | 明确安装基准、joint/link 命名建议、控制关节数量、ros2_control/MoveIt 约束、需要机械工程师交付的模型信息 | 已输出 `.ai_teamwork/V5_PROXY_INTERFACE.md`；本任务不改模型 |
| T-0005 | DONE | 机械工程师 | 按运控接口需求替换为六轴机械臂模型 | `ros2_ws/src/alfa_robot_description/`、MoveIt/bringup 控制配置 | 已实现 v5 proxy 双六轴机械臂；左右安装到 `updown` 两侧，挂点为 `x=0, y=±0.32, z=0.18`；simulation 未改 |

## 已完成任务

| ID | 负责人/角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- |
| T-0001 | 项目经理 | 建立轻量 AI 协作区 | `AGENTS.md`, `CLAUDE.md`, `.ai_teamwork/` | 已从重流程改为轻量协作 |
| T-0002 | Git 操作工程师 | 提交 AI 轻量协作机制更改 | `AGENTS.md`, `CLAUDE.md`, `.ai_teamwork/`（含 `engineers/`） | 仅提交协作相关文件 |
| T-0004 | 运控工程师 | 定义原机械臂位置安装六轴机械臂的接口需求 | `.ai_teamwork/V5_PROXY_INTERFACE.md` | 已明确 v5 传统 6 轴、2+1+3、Z 轴 joint1、T 型电机、proxy 技术栈与机械交付清单 |

## 待用户/PM 补充

| ID | 建议角色 | 任务 | 范围 | 备注 |
| --- | --- | --- | --- | --- |
