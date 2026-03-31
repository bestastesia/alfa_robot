# Alfa Robot 重构前架构梳理

## 1. 包级职责

`/home/kzoia/alfa_robot_ws/src` 下当前可以分成 4 个层次：

1. `alfa_robot_description`
   机器人模型单一事实源，负责 URDF/xacro、link/joint 拓扑、`ros2_control` 接口声明、硬件插件选择参数。

2. `alfa_robot_bringup`
   运行时装配层，负责启动 `robot_state_publisher`、`ros2_control_node`、controller spawner、GUI 桥接节点和仿真入口。

3. `alfa_robot_hardware`
   实机控制核心，作为 `ros2_control` 的 `SystemInterface` 插件被 `controller_manager` 动态加载，内部再通过 `CanBus` 访问 RMD 和 CANopen 电机。

4. `alfa_robot_moveit_config`
   规划层，主要面向 MoveIt。它不在当前实机基础控制闭环的关键路径上，但维护了一套独立的控制器/ros2_control 配置，后续重构时需要防止和 `description`/`bringup` 漂移。

## 2. 实际运行入口

实机主入口不是 `hardware` 包，而是 `bringup`：

1. `alfa_robot_bringup/launch/alfa_robot.launch.py`
   通用入口，可切换 mock/实机/GUI 控制。

2. `alfa_robot_bringup/launch/alfa_robot_gui_control.launch.py`
   面向实机 GUI 调试，固定使用真实硬件插件和 `all_position_controller`。

启动顺序：

1. `robot_state_publisher` 通过 xacro 生成并发布 `/robot_description`
2. `controller_manager/ros2_control_node` 从 `/robot_description` 读取 URDF
3. `controller_manager` 根据 URDF 中 `<ros2_control>` 加载硬件插件
4. spawner 依次拉起 `joint_state_broadcaster` 和业务 controller
5. GUI 模式下再由桥接节点把 `joint_state_publisher_gui` 的关节位置转发到 controller 命令话题

## 3. 控制主链路

### 3.1 配置装配链

`alfa_robot_description/urdf/alfa_robot.urdf.xacro`
-> 包含几何拓扑 `alfa_robot_macro.xacro`
-> 包含控制接口 `alfa_robot_macro.ros2_control.xacro`
-> 根据 launch 参数选择：

- `mock_components/GenericSystem`
- `gazebo_ros2_control/GazeboSystem`
- `gz_ros2_control/GazeboSimSystem`
- `alfa_robot_hardware/AlfaRobotHW`

这里决定了后续到底加载 mock、仿真还是真实硬件。

### 3.2 运行时数据流

控制数据在实机模式下的真实路径是：

`controller command`
-> `forward_command_controller` / 其他 controller
-> `AlfaRobotHW::write()`
-> `CanBus::writeOnce()`
-> RMD 0xA4 / CANopen PDO
-> 电机
-> 电机反馈
-> `CanBus::readOnce()`
-> `AlfaRobotHW::read()`
-> `joint_state_broadcaster`
-> `/joint_states`

这里的关键点是：控制器并不知道总线协议，它只和 `ros2_control` 的 command/state interface 打交道。

## 4. 硬件层内部结构

### 4.1 `AlfaRobotHW` 当前承担的职责

这个类不只是一个薄适配层，实际上混合了 5 类职责：

1. 解析 hardware parameter
2. joint 分类和索引管理
3. `ros2_control` state/command interface 导出
4. 生命周期管理和安全停机
5. 少量业务语义补偿
   包括 `turn` 软件零点、CANopen 3:1 减速比换算、首帧命令初始化、轨迹日志服务线程

### 4.2 `CanBus` 当前承担的职责

`CanBus` 是真正的通信与执行核心，也混合了多层职责：

1. SocketCAN 打开/关闭
2. RMD 协议编解码
3. CANopen SDO/PDO/NMT/CiA402 状态机
4. 低通滤波
5. 轨迹平滑
6. 轨迹日志缓存与 CSV 导出
7. 线程同步

这意味着当前 `hardware` 包不是简单的“驱动”，而是把协议层、运动平滑层、安全层和服务层都塞在了一起。

## 5. 当前 joint 分层

系统把 joint 分成 3 类，分类函数在 `AlfaRobotHW` 中硬编码：

1. RMD 位置关节
   `turn`
   `leftjoint2 leftjoint3 leftjoint4`
   `rightjoint2 rightjoint3 rightjoint4`

2. CANopen 位置关节
   `updown`
   `leftarmbase leftjoint1`
   `rightarmbase rightjoint1`

3. legacy/占位关节
   主要是四个轮子，用 velocity 接口做回显模拟。

注意：

- `turn` 虽然属于 RMD，但激活时会读取当前位置并当作软件零点。
- `leftarmbase/rightarmbase` 在读写时都有 3:1 换算。
- 四轮没有真实总线实现，只在 `read()` 里按命令做状态积分。

## 6. 关键静态映射

### 6.1 RMD joint -> motor/bus

- `turn` -> motor 1, `BASE`
- `leftjoint2/3/4` -> motor 1/2/3, `LEFT`
- `rightjoint2/3/4` -> motor 4/5/6, `RIGHT`

### 6.2 CANopen joint -> node id

- `updown` -> 1
- `leftarmbase` -> 2
- `leftjoint1` -> 3
- `rightarmbase` -> 4
- `rightjoint1` -> 5

这些映射现在全部硬编码在 C++ 源码里，不在 YAML/Xacro 中统一维护。

## 7. 控制器层角色

`bringup/config` 里有两套 controller 配置：

1. `alfa_robot_controllers.yaml`
   面向全机关节，主要用于 mock、GUI 调试和仿真。

2. `alfa_robot_controllers_can.yaml`
   面向实机最小闭环，只控制 `leftjoint2-4` 和 `rightjoint2-4`。

因此，当前“控制实现”不是唯一模式，而是至少有两种：

1. 最小实机控制链
   只关心 6 个 arm rotary joints

2. 全机关节位置控制链
   包含 `turn/updown/armbase/joint1`，这部分在真实硬件上依赖 CANopen 和部分补偿逻辑

## 8. 真实不能破坏的行为边界

如果要重构而不影响控制实现，下面这些行为必须保持等价：

1. `ros2_control` 对外接口名不能变
   joint 名、interface 名、controller 订阅话题语义都不能随意改。

2. `description` 中的 joint 列表和 `hardware` 中的分类逻辑必须一致
   否则 controller 能发命令，但硬件插件不一定导出对应接口。

3. `turn` 的软件零点补偿必须保留
   否则启用后当前位置会突然偏移。

4. `leftarmbase/rightarmbase` 的 3:1 换算必须保留
   否则指令位移和反馈位移会放大 3 倍。

5. 首次 `read()` 后用当前位置初始化 position command 的逻辑必须保留
   否则激活瞬间可能跳变。

6. `on_deactivate()` 的 safe shutdown 语义必须保留
   尤其是在实机控制时，不能直接把资源析构和停机动作拆散。

7. `CanBus::writeOnce()` 里的滤波/平滑是否启用、参数默认值和执行顺序必须明确冻结
   这是当前命令整形行为的一部分，不是“附加功能”。

8. `controller_manager` 的启动顺序不能随意改
   当前依赖 `robot_state_publisher` 先发布 `/robot_description`。

## 9. 当前主要耦合点

这是后续重构最容易出问题的地方：

1. joint 名字被散落在 4 处
   `description xacro`
   `hardware 分类函数`
   `controller yaml`
   `moveit_config`

2. 硬件参数来源分散
   launch -> xacro `<param>` -> `hardware_parameters` -> `AlfaRobotHW` 内默认值/解析逻辑

3. 同一个包同时含有实时控制逻辑和非实时服务逻辑
   例如轨迹日志服务线程和 CAN 实时收发在同一插件内。

4. 协议和运动语义耦合
   平滑、滤波、安全位置、减速比换算都放在总线层附近，不利于后续替换底层协议。

5. MoveIt 自带一份单独的 `alfa_robot.ros2_control.xacro`
   它并不直接复用 `description` 里的控制接口定义，这会导致后续配置漂移。

## 10. 推荐的重构切分方式

建议按“职责边界”而不是按文件数量切：

### 第一层：保持外部接口不变

先冻结以下外部契约：

- joint 名
- interface 名
- controller YAML 对外话题
- hardware plugin 名 `alfa_robot_hardware/AlfaRobotHW`
- launch 参数名

这一步不做行为变化，只做内部解耦。

### 第二层：拆 `hardware` 包内部职责

建议最少拆成以下组件：

1. `JointRegistry`
   统一保存 joint 分类、索引、总线/node/motor 映射、减速比和安全位姿配置。

2. `HardwareStateBuffer`
   专门负责 command/state 缓冲、首帧初始化、NaN 防护、速度加速度差分。

3. `RmdTransport`
   只处理 RMD socket、帧收发、协议编解码。

4. `CanopenTransport`
   只处理 CANopen socket、SDO/PDO/CiA402 和同步周期。

5. `CommandShaper`
   统一封装低通滤波与限速限加速度逻辑。

6. `SafetyManager`
   处理软件零点、safe shutdown、激活和停机策略。

7. `DiagnosticsService`
   处理轨迹日志服务节点和 CSV 导出。

### 第三层：统一配置源

把以下静态信息收敛成一份单一配置源：

- joint 分类
- joint 对应总线/node/motor id
- gear ratio / scale
- safe position
- controller 分组

可以是 YAML，也可以是一个集中式 xacro + YAML 组合，但不能再分散硬编码。

### 第四层：处理 MoveIt 配置漂移

`alfa_robot_moveit_config` 当前带有自己的 `alfa_robot.ros2_control.xacro`，建议只保留规划所需的最小配置，避免再维护一份“伪真机控制定义”。

## 11. 推荐的重构顺序

1. 先提取 joint/总线/比例尺配置，不改行为
2. 再把 `AlfaRobotHW` 与 `CanBus` 的职责拆开，但保留当前 pluginlib 入口类名
3. 再把轨迹日志和 diagnostics 挪出实时路径
4. 最后再考虑统一 bringup/controller/moveit 配置

不要一开始就改 launch、controller 名字和 joint 名字，这会让验证面同时爆炸。

## 12. 重构后的最小回归检查

每做一轮重构都至少验证这些点：

1. xacro 仍能正常展开
2. `controller_manager` 能加载 `AlfaRobotHW`
3. `ros2 control list_hardware_interfaces` 中 joint/interface 数量不变
4. `joint_state_broadcaster` 能正常发布
5. RMD 6 个关节命令顺序和旧版一致
6. CANopen 5 个关节读写量纲不变
7. `turn` 上电后零点行为不变
8. deactivate 时 safe shutdown 仍执行
9. GUI bridge 到 controller 的命令顺序不变

## 13. 一个务实结论

这套代码最适合的重构目标不是“重写控制”，而是先把现有控制语义固化，再把：

- 结构模型
- 控制接口
- 总线协议
- 命令整形
- 安全/诊断

分层拆开。

只要外部 `ros2_control` 契约不动，并把 `turn` 零点、CANopen 3:1 换算、首帧初始化、safe shutdown 这些语义原样保留，第一阶段重构就可以做到“架构变了，但控制表现不变”。
