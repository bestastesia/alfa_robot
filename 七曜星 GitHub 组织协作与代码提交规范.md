# 七曜星 GitHub 组织协作与代码提交规范

适用范围：物流机器人项目所有代码仓库、算法模块、ROS 2 package、仿真代码、部署脚本与技术文档

------

## 1. 文档目的

为了规范物流机器人项目的代码管理、多人协作、版本发布和系统集成流程，现统一制定本开发协作规范。

本项目涉及感知、抓取策略、机械臂控制、导航、任务状态机、仿真、部署等多个模块。各模块之间存在较强依赖关系，因此必须避免以下问题：

1. 不同同事各自维护一套代码，后期难以合并；
2. 主分支代码不可运行，影响演示和调试；
3. 接口随意修改，导致其他模块无法编译；
4. 没有版本记录，现场测试后无法回滚；
5. 代码审查缺失，bug 进入核心系统；
6. 总仓库与模块仓库代码重复，后期版本混乱。

本规范的核心原则是：

> **模块独立开发，系统统一集成；分支管理开发过程，tag 固定稳定版本；所有核心代码必须经过 PR 审查后合并。**

------

# 2. 总体代码组织方式

本项目采用：

```text
GitHub Organization
  ├── 多个模块仓库
  ├── 一个系统集成仓库
  ├── 一个统一项目看板
  └── 若干开发团队和权限组
```

推荐的仓库结构如下：

```text
logistics-robot-org/
├── robot_interfaces        # 自定义 msg / srv / action
├── robot_common            # 通用工具库、坐标变换、日志、参数工具
├── robot_perception        # 相机、点云、纸箱识别、6D pose 估计
├── robot_grasp_strategy    # 启发式抓取策略、双臂推荐、可达性筛选
├── robot_motion_control    # 机械臂规划、轨迹执行、夹爪控制
├── robot_navigation        # 雷达导航、底盘定位、移动控制
├── robot_task_fsm          # 任务状态机、Wall FSM、异常恢复 FSM
├── robot_bringup           # launch 文件、参数文件、整机启动配置
├── robot_sim               # 仿真环境、mock 节点、测试场景
├── robot_deploy            # Docker、部署脚本、工控机启动配置
├── robot_docs              # 架构文档、接口文档、开发规范
└── robot_system            # 整机集成仓库，锁定各模块版本
```

其中，`robot_system` 是整机集成仓库，不直接复制其他模块代码。它的作用是记录当前整机版本依赖哪些模块、哪些 tag、哪些配置文件。

------

# 3. Repo 与 Package 的划分原则

## 3.1 Repo 的作用

Repo 是 GitHub 上的代码仓库，用于管理一个相对独立的代码资产。

本项目中，一个 repo 不等于一个 ROS 节点，也不等于一个函数。Repo 应该对应一个相对稳定的能力域，例如感知、导航、抓取策略、状态机等。

判断是否应该单独建 repo 的标准：

```text
如果几个模块经常一起修改、一起测试、一起发布，则放在同一个 repo。
如果几个模块负责人不同、生命周期不同、依赖边界清晰，则拆成不同 repo。
```

例如：

```text
robot_perception repo
├── box_detector package
├── pose_estimator package
├── camera_adapter package
└── perception_debug_tools package
```

这几个 package 都属于感知能力域，可以放在同一个 `robot_perception` 仓库中。

## 3.2 Package 的作用

Package 是 ROS 2 或软件系统中的模块单位。

例如 `robot_task_fsm` 仓库内部可以包含：

```text
robot_task_fsm/
├── mission_fsm
├── wall_fsm
├── grasp_cycle_fsm
├── navigation_fsm_adapter
└── recovery_fsm
```

Repo 管理代码资产边界，package 管理软件结构边界。

------

# 4. 各仓库职责划分

## 4.1 `robot_interfaces`

该仓库用于存放系统内所有公共接口，包括：

```text
msg/
srv/
action/
interface docs/
```

包括但不限于：

```text
BoxPose.msg
BoxWallState.msg
GraspCandidate.msg
DualArmGraspCommand.msg
RobotTaskState.msg
NavigationCommand.msg
MotionExecutionResult.msg
```

管理要求：

1. 该仓库必须高度稳定；
2. 所有接口变更必须经过核心负责人审查；
3. 不允许随意修改已有字段名称和语义；
4. 对已有接口的破坏性变更必须升级版本；
5. 修改接口后必须同步更新接口文档；
6. 修改接口后必须通知所有依赖该接口的模块负责人。

接口变更属于高风险操作。

------

## 4.2 `robot_common`

该仓库存放跨模块通用代码，例如：

```text
坐标系转换工具
时间戳同步工具
日志封装
参数加载工具
错误码定义
通用数据结构
可视化调试工具
```

管理要求：

1. 只放真正通用的代码；
2. 不允许把业务逻辑塞进 common；
3. 修改 common 需要考虑所有依赖模块；
4. 公共函数必须有注释和测试。

------

## 4.3 `robot_perception`

该仓库存放感知相关代码，包括：

```text
相机驱动适配
点云预处理
纸箱检测
纸箱面识别
6D pose 估计
感知结果发布
感知调试可视化
```

输出接口应尽量稳定，例如：

```text
所有可见纸箱的 6D pose
纸箱最大可见面的长宽
纸箱朝向
置信度
时间戳
坐标系 frame_id
```

感知模块不应直接决定抓哪个箱子。它只负责提供环境和目标信息。

------

## 4.4 `robot_grasp_strategy`

该仓库存放启发式抓取策略，包括：

```text
纸箱墙分区
左侧三列 / 右侧两列策略
上优先 / 左优先规则
双臂抓取目标分配
可达性前筛
抓取候选排序
异常情况下的重选策略
```

该模块的输入来自感知模块，输出给运动控制或状态机模块。

该模块只负责“推荐抓取哪个箱子”，不直接执行机械臂控制。

------

## 4.5 `robot_motion_control`

该仓库存放机械臂和夹爪相关代码，包括：

```text
双臂运动规划
轨迹生成
轨迹执行
夹爪控制
抓取动作 primitive
放置动作 primitive
运动失败检测
机械臂状态反馈
```

该模块应向上层提供稳定的动作接口，例如：

```text
execute_grasp(left_target, right_target)
move_to_pregrasp_pose()
execute_place()
abort_motion()
get_motion_status()
```

------

## 4.6 `robot_navigation`

该仓库存放底盘和导航相关代码，包括：

```text
雷达定位
地图构建或加载
导航目标点规划
底盘运动控制
靠近箱墙
侧向移动
换墙操作
导航状态反馈
```

导航模块应负责机器人在不同货箱墙之间的位置调整，并向任务状态机提供状态反馈。

------

## 4.7 `robot_task_fsm`

该仓库存放任务状态机，包括：

```text
Mission FSM
Wall FSM
Grasp Cycle FSM
Navigation FSM Adapter
Recovery FSM
```

该模块负责组织完整任务流程，例如：

```text
进入任务
导航到箱墙
感知箱墙
选择抓取目标
调用机械臂执行
判断是否完成当前墙
切换下一面墙
异常恢复
任务结束
```

状态机模块不应直接实现底层算法，而是负责调用各模块能力。

------

## 4.8 `robot_bringup`

该仓库存放启动配置，包括：

```text
launch 文件
参数文件
整机启动配置
模块组合启动配置
RViz 配置
调试配置
```

例如：

```text
full_system.launch.py
perception_only.launch.py
navigation_only.launch.py
dual_arm_test.launch.py
wall_fsm_test.launch.py
```

------

## 4.9 `robot_sim`

该仓库存放仿真和测试环境，包括：

```text
仿真世界
纸箱墙生成器
mock 感知节点
mock 机械臂节点
mock 导航节点
回放数据
仿真测试脚本
```

仿真模块的目标是让系统在没有真实硬件时也能进行逻辑测试和回归测试。

------

## 4.10 `robot_system`

该仓库是系统集成仓库。

它不复制其他模块代码，而是记录当前整机版本依赖的模块版本。

推荐结构：

```text
robot_system/
├── logistics_robot.repos
├── README.md
├── docker/
├── scripts/
├── launch/
├── params/
├── tests/
└── docs/
```

其中 `logistics_robot.repos` 用于记录各模块仓库及其版本：

```yaml
repositories:
  src/robot_interfaces:
    type: git
    url: git@github.com:logistics-robot-org/robot_interfaces.git
    version: v0.1.0

  src/robot_perception:
    type: git
    url: git@github.com:logistics-robot-org/robot_perception.git
    version: v0.2.1

  src/robot_grasp_strategy:
    type: git
    url: git@github.com:logistics-robot-org/robot_grasp_strategy.git
    version: v0.1.3

  src/robot_task_fsm:
    type: git
    url: git@github.com:logistics-robot-org/robot_task_fsm.git
    version: v0.1.0
```

整机版本发布时，只需要更新 `.repos` 中各模块的版本号。

------

# 5. 分支管理规范

## 5.1 主要分支

每个核心仓库统一使用以下分支规则：

```text
main
feature/xxx
bugfix/xxx
hotfix/xxx
release/vx.y
```

含义如下：

| 分支           | 用途                               |
| -------------- | ---------------------------------- |
| `main`         | 稳定主分支，必须保持可编译、可运行 |
| `feature/xxx`  | 新功能开发分支                     |
| `bugfix/xxx`   | 普通 bug 修复分支                  |
| `hotfix/xxx`   | 现场测试或演示前紧急修复分支       |
| `release/vx.y` | 版本冻结和发布准备分支             |

## 5.2 `main` 分支规则

`main` 是稳定分支，必须满足：

1. 可以正常编译；
2. 可以通过基本测试；
3. 不允许提交临时代码；
4. 不允许直接 push；
5. 所有修改必须通过 PR 合并；
6. 合并前必须至少经过一名 reviewer 审查；
7. 关键仓库需要核心负责人审查。

严禁直接向 `main` 分支提交代码。

------

## 5.3 功能分支命名规范

功能开发使用：

```text
feature/功能名称
```

示例：

```text
feature/dual-arm-assignment
feature/left-wall-grasp-policy
feature/wall-fsm-basic-flow
feature/nav-to-box-wall
feature/perception-debug-viewer
```

bug 修复使用：

```text
bugfix/问题名称
```

示例：

```text
bugfix/pose-frame-transform
bugfix/grasp-candidate-empty
bugfix/nav-timeout-handling
```

紧急修复使用：

```text
hotfix/问题名称
```

示例：

```text
hotfix/demo-gripper-timeout
hotfix/field-test-launch-error
```

发布分支使用：

```text
release/v0.2
release/v0.3
release/v1.0
```

------

# 6. Commit 提交规范

## 6.1 Commit Message 格式

所有 commit message 采用以下格式：

```text
<type>(<scope>): <description>
```

例如：

```text
feat(grasp): add dual-arm box assignment strategy
fix(perception): correct box pose frame transform
docs(fsm): update wall fsm transition diagram
test(strategy): add unit tests for left-column priority rule
refactor(common): simplify transform utility interface
```

## 6.2 type 类型说明

| type       | 含义                       |
| ---------- | -------------------------- |
| `feat`     | 新功能                     |
| `fix`      | bug 修复                   |
| `docs`     | 文档修改                   |
| `style`    | 代码格式修改，不改变逻辑   |
| `refactor` | 重构，不新增功能、不修 bug |
| `test`     | 测试代码                   |
| `chore`    | 构建、配置、依赖等杂项     |
| `perf`     | 性能优化                   |
| `ci`       | CI 配置修改                |
| `revert`   | 回滚提交                   |

## 6.3 scope 示例

| scope        | 对应内容           |
| ------------ | ------------------ |
| `interfaces` | msg / srv / action |
| `perception` | 感知               |
| `grasp`      | 抓取策略           |
| `motion`     | 机械臂控制         |
| `nav`        | 导航               |
| `fsm`        | 状态机             |
| `sim`        | 仿真               |
| `deploy`     | 部署               |
| `docs`       | 文档               |
| `common`     | 公共工具           |

## 6.4 Commit 示例

推荐：

```text
feat(grasp): support left-three-column grasp priority
fix(fsm): handle empty grasp candidate list
docs(system): add repository management guideline
test(perception): add mock point cloud test case
refactor(nav): split docking logic from base controller
```

不推荐：

```text
update
fix bug
修改了一下
临时提交
final version
demo 能跑的版本
```

------

# 7. Pull Request 规范

所有核心代码必须通过 Pull Request 合并。

## 7.1 PR 标题规范

PR 标题格式：

```text
[type][module] 简要说明
```

示例：

```text
[feat][grasp] add dual-arm assignment for left wall phase
[fix][fsm] handle navigation timeout in wall fsm
[docs][system] add GitHub collaboration guideline
[test][sim] add mock perception node for integration test
```

## 7.2 PR 描述模板

每个 PR 必须说明以下内容：

```markdown
## 1. 修改内容

简要说明本次 PR 改了什么。

## 2. 修改原因

说明为什么需要这个修改，对应哪个 issue 或需求。

## 3. 影响范围

说明可能影响哪些模块：
- interfaces
- perception
- grasp_strategy
- motion_control
- navigation
- task_fsm
- bringup
- sim
- deploy

## 4. 测试情况

说明已经做过哪些测试：
- [ ] 本地编译通过
- [ ] 单元测试通过
- [ ] launch 测试通过
- [ ] 仿真测试通过
- [ ] 实机测试通过
- [ ] 不涉及测试

## 5. 风险说明

说明是否存在潜在风险、未完成部分或需要 reviewer 重点关注的地方。

## 6. 关联 Issue

Closes #xx
Related to #xx
```

## 7.3 PR 合并要求

普通 PR：

```text
至少 1 名 reviewer approve
CI 通过
无 unresolved conversation
```

核心模块 PR：

```text
至少 2 名 reviewer approve
其中 1 名必须是 core-maintainer
CI 通过
必要时完成仿真测试或实机测试
```

涉及接口修改的 PR：

```text
必须由 core-maintainers 审查
必须更新接口文档
必须通知受影响模块负责人
必须说明是否为破坏性变更
```

------

# 8. Issue 管理规范

Issue 用于管理需求、bug、任务和讨论，不应只在微信群或口头沟通中安排任务。

## 8.1 Issue 类型

统一使用以下类型：

| 类型       | 用途           |
| ---------- | -------------- |
| `feature`  | 新功能需求     |
| `bug`      | bug 报告       |
| `task`     | 开发任务       |
| `design`   | 架构设计讨论   |
| `docs`     | 文档任务       |
| `test`     | 测试任务       |
| `deploy`   | 部署任务       |
| `question` | 需要讨论的问题 |

## 8.2 Issue 标题规范

```text
[模块] 简要说明
```

示例：

```text
[grasp] 支持左侧三列纸箱墙的双臂抓取推荐
[fsm] Wall FSM 增加导航到下一墙面的状态
[perception] 纸箱 6D pose 输出增加 frame_id
[deploy] 工控机启动脚本支持自动拉起 full_system.launch.py
```

## 8.3 Issue 内容模板

```markdown
## 背景

说明为什么需要这个任务。

## 目标

说明完成后应该达到什么效果。

## 输入

说明依赖哪些数据、接口或模块。

## 输出

说明该任务完成后应该输出什么。

## 验收标准

- [ ] 标准 1
- [ ] 标准 2
- [ ] 标准 3

## 相关模块

- perception
- grasp_strategy
- motion_control
- navigation
- task_fsm

## 备注

其他说明。
```

## 8.4 Issue 与 PR 的关系

一个 PR 应尽量对应一个或多个明确 issue。

推荐：

```text
Issue #23: 支持左侧三列纸箱墙抓取策略
PR #31: implement left-three-column grasp policy
```

PR 描述中应写：

```text
Closes #23
```

这样 PR 合并后，Issue 会自动关闭。

------

# 9. Tag 与版本发布规范

## 9.1 Tag 的作用

Tag 用于标记稳定版本。

Branch 是开发线，会继续变化；Tag 是版本锚点，不应该变化。

例如：

```text
robot_grasp_strategy v0.1.0
robot_perception v0.2.1
robot_system v0.3.0-demo
```

## 9.2 版本号规范

采用语义化版本：

```text
MAJOR.MINOR.PATCH
```

例如：

```text
v0.1.0
v0.1.1
v0.2.0
v1.0.0
```

含义：

| 版本位 | 含义                   |
| ------ | ---------------------- |
| MAJOR  | 大版本，存在破坏性变更 |
| MINOR  | 新增功能，兼容旧版本   |
| PATCH  | bug 修复，不改变接口   |

早期可以加后缀：

```text
v0.1.0-alpha.1
v0.2.0-demo.1
v0.3.0-field.1
v1.0.0
```

## 9.3 模块版本发布流程

模块仓库发布流程：

```text
feature 分支开发
  ↓
提交 PR
  ↓
代码审查
  ↓
CI 通过
  ↓
合并到 main
  ↓
打 tag
  ↓
更新 CHANGELOG
```

例如：

```text
robot_grasp_strategy 发布 v0.2.0
```

说明该版本已经稳定，可以被 `robot_system` 集成。

## 9.4 整机版本发布流程

整机版本由 `robot_system` 发布。

流程：

```text
各模块发布稳定 tag
  ↓
robot_system 更新 logistics_robot.repos
  ↓
拉取所有指定版本模块
  ↓
colcon build
  ↓
运行集成测试
  ↓
仿真测试或实机测试
  ↓
robot_system 打 tag
```

例如：

```text
robot_system v0.3.0-demo.1
```

表示这是一个可用于 demo 的整机版本。

------

# 10. 接口变更规范

接口包括：

```text
ROS msg
ROS srv
ROS action
topic 名称
service 名称
action 名称
参数文件字段
坐标系约定
错误码
状态码
```

接口变更必须谨慎。

## 10.1 非破坏性变更

例如：

```text
新增字段，但旧代码不受影响
新增 topic
新增 service
新增状态码
```

这类变更需要：

```text
PR 审查
更新文档
通知相关模块负责人
```

## 10.2 破坏性变更

例如：

```text
删除字段
修改字段名称
修改字段单位
修改 topic 名称
修改 service 请求/响应结构
修改坐标系定义
修改状态机状态语义
```

这类变更必须：

```text
创建 design issue
说明修改原因
列出影响模块
给出迁移方案
经过 core-maintainers 审查
必要时升级 major/minor 版本
```

## 10.3 接口文档要求

每个接口必须说明：

```text
字段名称
字段类型
单位
坐标系
时间戳含义
发布频率
谁发布
谁订阅
异常情况
```

例如：

```markdown
## BoxPoseArray

发布方：robot_perception  
订阅方：robot_grasp_strategy / robot_task_fsm  
Topic: /perception/box_poses  
Frame: camera_link 或 base_link  
Frequency: 5-10 Hz  

字段说明：
- boxes: 当前视野中的纸箱列表
- pose: 纸箱中心或可见面中心的 6D pose
- width: 可见面的宽度，单位 m
- height: 可见面的高度，单位 m
- confidence: 检测置信度
```

------

# 11. 代码风格规范

## 11.1 基本要求

所有代码必须满足：

1. 命名清晰；
2. 不提交无用注释；
3. 不提交大段调试 print；
4. 不提交本地路径；
5. 不提交密钥、token、账号密码；
6. 不提交大文件数据集；
7. 不提交编译产物；
8. 不提交临时文件。

禁止提交：

```text
build/
install/
log/
__pycache__/
.vscode/
.DS_Store
*.bag
*.mp4
*.zip
*.pth
*.onnx
```

特殊模型文件或数据文件应放入专门的数据管理位置，不直接进入核心代码仓库。

## 11.2 Python 规范

建议：

```text
函数名：snake_case
类名：CamelCase
常量：UPPER_CASE
模块名：snake_case
```

示例：

```python
class GraspCandidateSelector:
    def select_candidates(self, box_poses):
        pass
```

## 11.3 C++ 规范

建议：

```text
类名：CamelCase
函数名：snake_case 或 lowerCamelCase，项目内统一
变量名：snake_case
常量：kConstantName 或 UPPER_CASE，项目内统一
```

核心 C++ 代码必须避免：

```text
裸指针滥用
无检查数组访问
阻塞式死循环
未处理异常
硬编码路径
魔法数字
```

------

# 12. 测试规范

## 12.1 测试类型

本项目测试分为：

```text
单元测试
模块测试
接口测试
launch 测试
仿真测试
实机测试
回归测试
```

## 12.2 每个模块最低测试要求

每个核心模块至少应有：

```text
能编译
能启动
有 README
有基本测试样例
有 mock 输入或测试数据
```

例如 `robot_grasp_strategy` 至少需要：

```text
输入一组 box poses
输出两个推荐抓取候选
能够处理空输入
能够处理只有一个箱子的情况
能够处理左右手分配
能够处理不可达候选过滤
```

## 12.3 PR 测试说明

每个 PR 必须说明测试情况。

例如：

~~~markdown
## 测试情况

- [x] colcon build 通过
- [x] 单元测试通过
- [x] 使用 mock perception 测试通过
- [ ] 未进行实机测试

测试命令：

```bash
colcon build --packages-select robot_grasp_strategy
colcon test --packages-select robot_grasp_strategy
---

# 13. CI 规范

后续每个核心仓库应配置 GitHub Actions。

最低 CI 要求：

```text
代码能编译
测试能运行
lint 不报严重错误
~~~

模块仓库 CI：

```text
checkout
install dependencies
colcon build
colcon test
```

系统仓库 CI：

```text
读取 logistics_robot.repos
拉取所有模块
colcon build
运行集成测试
运行 launch smoke test
```

PR 未通过 CI，不允许合并。

------

# 14. 文档规范

每个仓库至少包含：

```text
README.md
CHANGELOG.md
CONTRIBUTING.md
接口说明文档
运行说明
测试说明
```

## 14.1 README 必须包含

~~~markdown
# 仓库名称

## 1. 功能说明

说明该仓库负责什么。

## 2. 输入输出

说明订阅什么 topic，发布什么 topic，调用什么 service/action。

## 3. 依赖模块

说明依赖哪些仓库或 package。

## 4. 编译方式

```bash
colcon build --packages-select xxx
~~~

## 5. 启动方式

```bash
ros2 launch xxx xxx.launch.py
```

## 6. 测试方式

```bash
colcon test --packages-select xxx
```

## 7. 负责人

维护人：xxx

```
## 14.2 CHANGELOG

每次发布 tag 前必须更新 `CHANGELOG.md`。

格式：

```markdown
# Changelog

## v0.2.0

### Added
- 支持左侧三列纸箱墙抓取策略
- 新增双臂候选分配逻辑

### Fixed
- 修复空候选列表导致状态机阻塞的问题

### Changed
- 调整抓取候选排序逻辑

### Known Issues
- 暂未支持严重遮挡情况下的候选重选
```

------

# 15. 权限与 Team 管理

GitHub Organization 中建议设置以下 team：

```text
core-maintainers
perception-team
grasp-strategy-team
motion-team
navigation-team
task-fsm-team
sim-deploy-team
interns
```

权限建议：

| Team                  | 权限                                       |
| --------------------- | ------------------------------------------ |
| `core-maintainers`    | 所有核心仓库 maintain/admin                |
| `perception-team`     | `robot_perception` write/maintain          |
| `grasp-strategy-team` | `robot_grasp_strategy` write/maintain      |
| `motion-team`         | `robot_motion_control` write/maintain      |
| `navigation-team`     | `robot_navigation` write/maintain          |
| `task-fsm-team`       | `robot_task_fsm` write/maintain            |
| `sim-deploy-team`     | `robot_sim`、`robot_deploy` write/maintain |
| `interns`             | 指定仓库 write，不允许直接合并 main        |

所有核心仓库应开启：

```text
main 分支保护
禁止直接 push
必须 PR 合并
必须 reviewer approve
必须 CI 通过
禁止 force push
```

------

# 16. Project 看板管理

组织级 Project 用于管理跨仓库任务。

建议建立：

```text
Logistics Robot Development Board
```

字段包括：

```text
Status
Module
Priority
Milestone
Owner
Risk
Hardware Needed
```

## 16.1 Status

```text
Backlog
Ready
In Progress
Review
Testing
Done
Blocked
```

## 16.2 Module

```text
Interfaces
Common
Perception
Grasp Strategy
Motion Control
Navigation
Task FSM
Bringup
Simulation
Deploy
Docs
```

## 16.3 Priority

```text
P0：阻塞系统运行，必须优先解决
P1：影响核心功能，需要尽快完成
P2：普通功能或优化
P3：低优先级改进
```

## 16.4 Milestone

示例：

```text
M1: 基础架构搭建
M2: 单模块可运行
M3: 仿真联调
M4: 实机联调
M5: Demo 版本
M6: 现场测试版本
```

------

# 17. 推荐开发流程

标准开发流程如下：

```text
1. 创建 Issue
2. 分配负责人
3. 从 main 拉 feature 分支
4. 在 feature 分支开发
5. 本地编译和测试
6. 提交 PR
7. Reviewer 审查
8. 修改问题
9. CI 通过
10. 合并到 main
11. 必要时打 tag
12. robot_system 更新集成版本
```

具体命令示例：

```bash
git checkout main
git pull origin main

git checkout -b feature/dual-arm-assignment

# 开发代码
git add .
git commit -m "feat(grasp): add dual-arm assignment strategy"

git push origin feature/dual-arm-assignment
```

然后在 GitHub 上创建 PR。

------

# 18. 整机集成流程

整机集成由 `robot_system` 管理。

流程如下：

```text
1. 各模块开发并合并到 main
2. 各模块发布稳定 tag
3. robot_system 更新 .repos 文件
4. 拉取所有模块指定版本
5. colcon build
6. 运行集成测试
7. 运行仿真测试
8. 必要时运行实机测试
9. robot_system 打整机版本 tag
```

示例：

```text
robot_interfaces      v0.1.0
robot_perception      v0.2.1
robot_grasp_strategy  v0.2.0
robot_motion_control  v0.1.3
robot_navigation      v0.1.2
robot_task_fsm        v0.2.0
robot_bringup         v0.1.1

robot_system          v0.2.0-demo.1
```

这表示 `robot_system v0.2.0-demo.1` 对应一组明确的模块版本，可以复现、部署和回滚。

------

# 19. 后续阶段规划

## 阶段 0：GitHub 组织初始化

目标：先建立规范和基本仓库。

任务：

```text
创建 GitHub Organization
创建核心 team
创建基础 repo
添加 README 模板
添加 PR 模板
添加 Issue 模板
添加 .gitignore
添加基础分支保护
```

建议优先创建：

```text
robot_interfaces
robot_common
robot_perception
robot_grasp_strategy
robot_motion_control
robot_navigation
robot_task_fsm
robot_bringup
robot_system
robot_docs
```

------

## 阶段 1：接口优先冻结

目标：先稳定模块之间的通信接口。

重点完成：

```text
感知输出接口
抓取策略输入输出接口
运动控制 action/service 接口
导航状态接口
任务状态机状态接口
错误码和状态码
坐标系约定
```

该阶段最重要的是 `robot_interfaces`。

完成标准：

```text
所有核心模块负责人确认接口
接口文档完成
基本 msg/srv/action 可编译
依赖模块能够引用接口
```

------

## 阶段 2：单模块开发

目标：各模块可以独立运行和测试。

模块目标：

```text
robot_perception:
  能输出 mock 或真实纸箱 pose

robot_grasp_strategy:
  能根据 box poses 输出双臂抓取候选

robot_motion_control:
  能接收抓取目标并执行 mock 或真实动作

robot_navigation:
  能完成导航目标接收和状态反馈

robot_task_fsm:
  能串联 mock 感知、mock 抓取、mock 导航、mock 运动
```

完成标准：

```text
每个模块有 README
每个模块能 colcon build
每个模块有最小测试
每个模块有 launch 或测试入口
```

------

## 阶段 3：仿真联调

目标：在没有完整硬件的情况下验证任务流程。

重点：

```text
mock perception
mock motion
mock navigation
wall fsm
grasp cycle fsm
异常恢复流程
```

完成标准：

```text
能够模拟一面箱墙
能够完成候选选择
能够完成一次双臂抓取流程
能够判断当前墙是否完成
能够切换到下一面墙
```

------

## 阶段 4：实机联调

目标：将仿真流程迁移到真实机器人。

重点：

```text
真实相机输入
真实纸箱 pose
真实机械臂执行
真实导航反馈
系统级日志
异常处理
安全停止
```

完成标准：

```text
单次抓取成功
连续多次抓取成功
一面箱墙流程跑通
异常情况下可以安全退出
所有日志可回放
```

------

## 阶段 5：Demo 版本发布

目标：形成一个可演示、可复现的整机版本。

需要完成：

```text
robot_system 锁定所有模块 tag
统一 launch 启动
统一参数配置
一键部署脚本
演示说明文档
已知问题列表
回滚方案
```

发布版本示例：

```text
robot_system v0.3.0-demo.1
```

------

# 20. 禁止事项

以下行为原则上禁止：

```text
直接 push main
未经 review 合并核心代码
在微信群里口头改接口但不更新文档
把本地临时代码提交到 main
把模型文件、大数据文件、编译产物提交到仓库
在多个仓库复制同一份代码
在 robot_system 里复制模块源码
使用 final、new、test、demo 等混乱分支名
提交账号、密码、token、密钥
接口变更不通知依赖模块
```

------

# 21. 推荐事项

推荐所有同事遵循：

```text
开发前先看 issue
开发前先同步 main
一个 PR 只做一类事情
PR 尽量小而清晰
接口变更必须写文档
重要逻辑必须写注释
核心算法必须有测试输入和输出示例
每次现场测试前打 tag
每次 demo 前由 robot_system 锁版本
```

------

# 22. 最终原则

本项目代码管理遵循以下原则：

```text
Repo 是资产边界
Package 是代码结构
Team 是责任边界
Project 是任务视图
Branch 是开发过程
Tag 是版本锚点
PR 是质量门禁
robot_system 是整机版本入口
```

更简单地说：

> **每个模块独立开发，但不能各自为战；所有模块最终都要通过 `robot_system` 统一集成、统一测试、统一发布。**

本规范从发布之日起开始执行。后续可根据项目规模、人员变化和现场测试情况继续迭代。