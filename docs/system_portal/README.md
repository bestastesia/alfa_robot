# ALFA Robot 运控系统地图

这是一个纯静态、多页面的项目导航站，用来快速回答：

- 系统和外界如何交互？
- 内部有哪些主流程？
- 每个 ROS2 包消费什么、产出什么、当前状态如何？
- 哪些部分是稳定接口、过渡主包、实验工具、mock 或外部依赖？

## 打开方式

直接用浏览器打开：

```bash
xdg-open docs/system_portal/index.html
```

或者启动一个本地静态服务：

```bash
cd /mnt/mydisk/ALFA/alfa_robot/docs/system_portal
python3 -m http.server 8765
```

然后访问：`http://127.0.0.1:8765/`

## 文件结构

```text
docs/system_portal/
├── index.html                # 系统与外界交互总览
├── flows.html                # 业务/算法/碰撞/孪生流程
├── packages.html             # 包列表、搜索、过滤
├── package.html              # 单包详情页，通过 ?id=package_id 进入
├── status.html               # 包状态看板
├── architecture.html         # 架构评审：现状问题全景、依赖图、问题清单
├── target_architecture.html  # 目标架构：包职责矩阵、依赖方向、代码搬迁表
├── refactor_plan.html        # 重构路线：增量迁移批次和验证口径
└── assets/
    ├── data.js               # 唯一主要数据源，新增包/流程时优先改这里
    ├── architecture_data.js  # 架构评审专用数据源（问题、目标架构、迁移计划）
    ├── app.js                # 页面渲染逻辑
    └── site.css              # 视觉样式
```

## 更新规则

项目新增包、接口或流程时，优先更新 `assets/data.js`：

- 新包：追加到 `packages`。
- 新流程：追加到 `systemFlows`。
- 外部交互变化：更新 `externalActors`。
- 包状态变化：更新 `maturity`、`status` 和 `statusNotes`。

页面会自动重新渲染卡片、流程、搜索和状态看板。

架构评审相关内容（问题清单、目标架构、依赖规则、迁移批次）在 `assets/architecture_data.js`；
重构推进或问题关闭时更新对应条目即可，三个架构页面会自动重新渲染。
