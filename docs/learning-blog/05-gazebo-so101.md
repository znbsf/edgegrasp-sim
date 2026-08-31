# 第 5 章：SO-101、Gazebo 与 controller——“动了”还不是“抓住了”

## 学习目标

- 认识项目使用的三个上游仓库、固定提交与用途；
- 理解 URDF、ros2_control、controller、FJT 和 Gazebo physics 的连接；
- 区分 controller 成功、接触样本和独立 physics grasp；
- 解释为什么 contact wrench 是诊断值，不是已标定真实力。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| URDF/xacro | 机器人连杆、关节、几何和接口描述 |
| ros2_control | 把统一 controller 接口接到模拟或硬件 backend 的框架 |
| FJT | FollowJointTrajectory；让一组关节按时间轨迹运动的 Action |
| Gazebo Harmonic | 本项目 WSL 运行过的仿真器系列；记录版本为 Gazebo Sim 8.11.0 |
| contact sample | 某个仿真时刻报告的一组碰撞接触 |
| controller active | controller 已加载并可接目标，不代表它已完成安全路径 |
| primitive proxy | 用 box 等简单碰撞体代替复杂 mesh |

## 前置知识

- 知道 plan-only 与执行分开；
- 知道机械臂由多个关节组成；
- 知道仿真器的接触与真实传感器标定不是一回事。

## 系统图/流程图

~~~mermaid
flowchart LR
    U[pinned SO-101 URDF/xacro] --> R[robot_state_publisher]
    U --> C[ros2_control]
    W[Gazebo world + proxy collisions] --> C
    C --> A[arm_controller FJT]
    C --> G[gripper_controller FJT]
    C --> J[/joint_states]
    A --> P[simulated joint motion]
    G --> P
    W --> K[contact topics / cube pose]
    K --> O[read-only physics observer]
    O --> V{both pads same sample<br/>+ 20 mm lift<br/>+ 0.5 s retention}
~~~

图源：[upstream manifest](../upstream-manifest.json)、[runbook controller gate](../ubuntu-jazzy-runbook.md#a-controller-only-runtime-gate) 和 GraspPhysicsEvidence.action。

替代文本：固定 SO-101 描述连接 robot_state_publisher 与 ros2_control，arm/gripper FJT 驱动 Gazebo 关节，Gazebo 另行输出接触和 cube pose；独立观察器组合双指垫、20 mm 抬升和 0.5 秒保持。

## 实际操作

### 先固定上游，而不是复制代码

| 上游 | pin / license | 用途 | 当前边界 |
| --- | --- | --- | --- |
| adoodevv/so101_ros2 | 0305e03ab54e64aae9263fcbf339622e654012f3 / BSD-3-Clause | Gazebo、gz_ros2_control、URDF、controller、MoveIt 配置 | scoped Jazzy runtime；全碰撞与真机未验证 |
| TheRobotStudio/SO-ARM100 | 7629d2ad9853d10fb903093a33ef6114099d97e5 / Apache-2.0 | 几何、URDF/MJCF、mesh、标定约定交叉检查 | static verified；Windows MuJoCo runtime blocked |
| legalaspro/so101-ros-physical-ai | 58318c905a2c61289fa907de85cb8473322fbe68 / Apache-2.0 | mock/real、MoveIt、MCAP、硬件安全参考 | 远程固定文件证据；未 clone、未 build、未运行 |

Feetech 子模块 pin 为 4c0fdbfe16c84c686f8ace09526c52d98d0110ca。完整 machine-readable 来源是 upstream-manifest.json。

### 隔离 graph 中先看接口

以下命令来自 runbook，本轮未执行：

~~~bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 action list -t | grep follow_joint_trajectory
ros2 action info /arm_controller/follow_joint_trajectory
ros2 action info /gripper_controller/follow_joint_trajectory
~~~

期望三个 controller active，joint_states 含五个 arm joint 加 gripper，并看到两个 control_msgs/action/FollowJointTrajectory endpoint。

SO-101 arm 精确顺序：

~~~text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
~~~

gripper controller 只含 gripper。真实路径应使用 EdgeGrasp ExecuteTrajectory gate；直接发 FJT 只属于 runbook 明确标记的 simulation bypass diagnostic。

## 期望输出（已登记 runtime 示例）

上面的只读接口查询只能确认 controller/interface inventory。下面的接触、抬升与保持结果来自仓库已有的隔离 Gazebo artifacts，不是执行五条查询命令就会得到的输出。

限定 runtime 已观察到：

- joint_state_broadcaster、arm_controller、gripper_controller active；
- 六个 joint state；
- arm/gripper FJT action 接受保守 gated goal；
- controller terminal 与 command identity 可关联；
- Candidate024 成功记录中，独立观察器要求两个配置的 distal-pad token 在同一个 contact sample 中出现，cube 至少抬升 20 mm，并完成 0.5 s retention。

Controller status 4 或 FJT error_code=0 只说明该命令终态成功。Candidate003、005、010、011、012、014、022 都曾出现协议或 controller 完成、但物理合同失败的事实。

## 踩坑记录

### 上游 README 与实际 YAML 冲突

固定 adoodevv README 仍写着不完整的 MoveIt 与 ForwardCommandController/Float64MultiArray gripper 路线，但固定 ros2_controllers.yaml 和 moveit_controllers.yaml 实际映射两个 FJT controller。EdgeGrasp 采用固定 YAML，并把 README 冲突保留为上游文档风险；源码审计和可贡献的最小修复见 [第 11 章](11-open-source-contribution.md#3-候选-bgripper-controller-的文档与-yaml-合同冲突)。

### active controller 被写成抓取成功

active 只证明接口可用。Candidate021 已建立 1.062 s bilateral contact，却因 free-space goal tolerance 把受接触限制的 close 判为 FJT failure，lift 根本没有 dispatch；这正说明 controller 语义与物理语义不同。

### 分时的单侧接触被拼成双侧

观察器要求每个配置 pad token 在同一个 sample 出现。左侧在 t1、右侧在 t2 不能组合成 bilateral contact。

### wrench/penetration 数值被当作标定力

Candidate006 起加入 depth、wrench maxima 和 joint effort，只用于诊断。没有传感器标定、摩擦模型验证或 force-closure 分析，不能把它们写成真实牛顿力。

### 日志没有 DART 错误就宣称碰撞正确

Candidate024 的相关日志未出现选定的 mesh/geometry construction diagnostic；这只说明没有观察到那些特定日志。机器人仍使用保守 primitive proxies，只有一个 base proxy 有独立行为证据。

### MuJoCo 资产可解析就写成动力学运行

SO-ARM100 的 13 个 mesh 和关节/actuator 名称做过 static check，但 Windows MuJoCo 3.3.7/3.12.0 因 DLL 初始化失败，2.3.7 又不接受新 kv schema；没有成功 stepping。

## 排查过程

| 观察 | 先归类 | 下一步 |
| --- | --- | --- |
| controller inactive | interface/controller | 不发 goal，检查 ros2_control 配置 |
| FJT rejected/aborted | controller | 看 exact joint order、tolerance、start state 与 terminal |
| FJT success，cube 不动 | controller PASS + physics negative | 查接触、cube pose、stage 时序 |
| 只有单 pad 或分时接触 | contact negative | 不升级为 bilateral；检查几何/对齐 |
| bilateral contact，lift < 20 mm | transient physics negative | 看接触是否早于 lift 丢失、姿态/力矩 |
| lift 达标但 retention 不满 | physics negative | 保留峰值，不能写成 verified grasp |

排查必须同时保存 command timeline 和 cube/contact source timestamps，避免把 close 之前的接触归到 lift 期间。

## 最终证据

- Gazebo Harmonic 8.11.0、ROS 2 Jazzy 的 scoped graph 观察到三个 controller、六个 joint states 与两个 FJT action。
- 固定上游 commit、license、用途和限制记录在 upstream-manifest.json。
- Candidate024 固定控制共有 11 次 runtime attempt：10 次 physics success，另 1 次在运动前 stale SAFE_STOP；此外 3 个代表姿态各执行一次并成功。它们均由独立 observer 判定，而不是由 FJT 或 GraspSequence 自报。
- 其余候选保留了大量 controller/sequence 完成但 physics false 的负证据。

证据层：controller 与窄范围 simulation physics，二者不可互换。

## 仍不能宣称的边界

- 没有真实 SO-101、校准、EEPROM/udev、温度或真实 gripper mapping 证据。
- 没有 force closure、校准力、全 mesh collision fidelity 或物理确定性。
- 三个代表姿态各一次，不是每姿态重复性或工作空间成功率。
- camera topic 出现不等于内参、外参或定位已验证。
- active controller、FJT success、可加载模型都不是硬件抓取。

## 小练习

1. 给出 controller success 但 physics failure 的两个可能场景。
2. 为什么 bilateral contact 必须是 same-sample？
3. 把 Candidate021 的结果分别写成 controller 结论和 physics 结论。
4. 解释为什么“日志没报错”只是弱证据。
