# 第 3 章：MoveIt plan-only——把“想出路线”和“真的执行”分开

## 学习目标

- 理解 IK、motion planning、RobotTrajectory 和 controller execution 的区别；
- 解释 EdgeGrasp 为什么禁用 MoveGroup 直接执行；
- 读懂 REAL_MOVEGROUP_NORMAL 的零执行证据；
- 知道一个 plan-only PASS 还需要哪些轨迹验证。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| IK | 给定末端姿态，求一组可能的关节角 |
| planning group | 一起参与规划的关节集合；当前第一阶段只允许 arm |
| planning pipeline | 规划流程；项目运行中出现 OMPL、Pilz、STOMP |
| RobotTrajectory | MoveIt 的关节轨迹，不是 core 的 Cartesian MotionPlan |
| plan_only | 只求路线，不让 MoveGroup 执行 |
| trajectory digest | 对规范化轨迹计算的身份摘要，用来关联“规划的”和“送去 gate 的”是否同一条 |
| error_code.val == 1 | MoveIt SUCCESS；仍需独立检查轨迹内容 |

## 前置知识

- 完成第 2 章，理解 Action result 和 correlation；
- 知道机械臂末端姿态包含 position 与 orientation；
- 知道“算法返回成功”也可能携带格式不合法或不符合本项目限制的数据。

## 系统图/流程图

~~~mermaid
flowchart LR
    A[PlanTarget immutable pose] --> B[带时间戳 tf2 到 base_link]
    B --> C[/compute_ik<br/>fresh joint-state seed]
    C --> D[MoveGroup<br/>plan_only=true]
    D --> E[RobotTrajectory validator]
    E --> F{本次模式}
    F -->|独立 plan-only probe| G[记录摘要并丢弃<br/>execution counters = 0]
    F -->|明确授权的 typed runtime| H[ExecuteTrajectory action]
    H --> I[trajectory_gate]
    I --> J[arm_controller FJT]
~~~

图源：[ros_ws README Safety and planning paths](../../ros_ws/README.md#safety-and-planning-paths) 与 [runbook C/D](../ubuntu-jazzy-runbook.md#c-moveit-plan-only-diagnostics)。

替代文本：PlanTarget 经 tf2、IK、MoveGroup plan-only 和独立轨迹校验；独立 probe 会丢弃轨迹并保持零执行，只有另一个明确授权的 typed runtime 才能进入 ExecuteTrajectory 和 FJT。

## 实际操作

以下命令是现有 runbook 的复现路径，本轮未执行。

先启动 EdgeGrasp thin overlay，并核对 MoveGroup 不能直接执行：

~~~bash
ros2 launch edgegrasp_ros edgegrasp_proxy_move_group.launch.py \
  robot_name:=so101 use_camera:=false use_gazebo:=true \
  use_sim_time:=true use_rviz:=false

ros2 param get /move_group allow_trajectory_execution
ros2 param get /move_group disable_capabilities
ros2 service list -t | grep plan_kinematic_path
~~~

必须看到 allow_trajectory_execution=false、两个 execute capability 被禁用，并在日志中确认 Pilz ValidateSolution 已加载。

对新 Candidate，不要先执行；使用隔离的三段 plan-only runner。以 Candidate024 为例：

~~~bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
bash scripts/run_grasp_candidate_plan_only.sh \
  151 \
  /home/edgegrasp/ros2_ws/test_results/candidate024_plan_only_UNIQUE \
  candidate024-face-aligned-plan-only \
  so101_grasp_geometry_candidate024_face_aligned_q0p40.json \
  10 0.0 candidate012_control_mu1p0 0.0 \
  so101_side_grasp_candidate024_face_aligned.json \
  scene_candidate024_face_aligned.json \
  table_cube_candidate024_face_aligned.sdf
~~~

要求：

- 使用新的 ROS_DOMAIN_ID；
- artifact 目录必须不存在；
- graph 中不启动 adapter、trajectory gate、sequence、ExecuteTrajectory client 或 FJT client；
- 只调用 /plan_kinematic_path；
- 每一条返回轨迹校验后立即丢弃。

## 期望输出

Candidate024 固定姿态 plan-only artifact 的关键期望：

~~~text
attempts: 10/10
segments: 30/30 accepted
MoveIt error code: 1
trajectory validation: PASS
trajectory publication count: 0
ExecuteTrajectory goal count: 0
FJT goal count: 0
execution attempted: false
~~~

REAL_MOVEGROUP_NORMAL 的 20-case target matrix 进一步记录：

- 10/10 distinct reachable hypotheses 通过，30/30 arm segments 接受；
- 4 个无效 scene 在 ROS 启动前拒绝；
- 6 个远目标被 MoveIt 拒绝；
- false accept、false reject、unverified 和 motion-side-effect 均为 0。

来源：[Candidate024 fixed observation](../observations/2026-08-29-candidate024-face-aligned-runtime.json) 与 [target matrix](../observations/2026-08-29-candidate024-target-matrix-plan-only.json)。

## 踩坑记录

### GetMotionPlan 没有 PlanningOptions

/plan_kinematic_path 的 GetMotionPlan service 只有 motion_plan_request；不能照搬 MoveGroup action 的 planning_options 字段。调用前要用 ros2 interface show 核对实际 schema。

### 只看 error_code.val

SUCCESS 之后还要检查：

- joint_names 精确顺序；
- 每点 position/velocity/acceleration 数组长度；
- 数值有限；
- time_from_start 为正且严格递增；
- 固定基座 multi-DOF points 为空；
- 起始状态、关节限制、项目保守速度/加速度限制；
- trajectory digest 与后续 typed command 相同。

### 把轨迹发布器留在 plan-only graph

只要 graph 中存在能自动消费规划结果的节点，就不能仅靠“我没有手动点执行”证明零执行。独立 runner 通过没有 publisher/action client 和显式计数共同证明。

### 把一个 IK 失败写成“机械臂撞了”

Candidate009 的负 tool-X offsets 在 approach-to-descend 返回 NO_IK_SOLUTION，且从未执行。这是可达性/IK 边界，不是运动碰撞或物理失败。

### 用固定 base 原点生成 shoulder_pan 目标

最初的 Candidate024 translation generator 把 cube 与各阶段 pose 做相同 XYZ 平移，10/10 所谓正例都在第一段 NO_IK_SOLUTION。固定 URDF 显示 shoulder_pan 原点偏离 base 原点，局部 +Z 在 base frame 中映射为 -Z。改为绕精确关节原点/轴旋转后，10/10 distinct hypotheses 才通过。

## 排查过程

~~~mermaid
flowchart TD
    A[PlanTarget 未通过] --> B{scene contract 先通过吗?}
    B -->|否| C[修 scene/hash/ACM；不要启动 MoveIt]
    B -->|是| D{tf2 / fresh joint state?}
    D -->|否| E[记录 frame/time/start-state 拒绝]
    D -->|是| F{IK 成功?}
    F -->|否| G[NO_IK_SOLUTION；作为零执行负证据]
    F -->|是| H{MoveIt error = SUCCESS?}
    H -->|否| I[记录 planner error；零 dispatch]
    H -->|是| J{trajectory validator PASS?}
    J -->|否| K[拒绝矛盾/畸形轨迹]
    J -->|是| L[plan-only 记录摘要并丢弃]
~~~

图源：[adapter_node.py](../../ros_ws/src/edgegrasp_moveit_adapter/edgegrasp_moveit_adapter/adapter_node.py)、[runbook C/D](../ubuntu-jazzy-runbook.md#c-moveit-plan-only-diagnostics)、[Candidate009](../observations/2026-08-27-candidate009-plan-only-runtime.json) 与 [Candidate024](../observations/2026-08-29-candidate024-target-matrix-plan-only.json) observations。

替代文本：排查从 scene、tf2/joint state、IK、MoveIt result 到独立轨迹校验逐层进行；只有全部通过才形成 plan-only 记录，轨迹仍被丢弃。

不要一次改场景、目标姿态、planner 和容差。Candidate 路线采用单变量控制：static/scene gate → isolated plan-only → 仅在预注册条件满足后进行一次 typed runtime。

## 最终证据

- thin overlay 复用固定上游 MoveIt 配置，只增加 EdgeGrasp 自有 distal-pad boxes、ValidateSolution 和 direct-execution 禁用。
- Candidate024 20-case matrix 是 REAL_MOVEGROUP_NORMAL：真实 MoveGroup 正常 GetMotionPlan、轨迹验证/丢弃、显式零执行。
- Candidate009 的 NO_IK_SOLUTION 与 Candidate024 旧 translation generator 都被保留为失败假设，而不是从历史中删除。
- MoveIt RobotTrajectory 从未被强行转换成 core Cartesian MotionPlan。

证据层：real MoveGroup normal plan-only；不包含 cancel 健康。

## 仍不能宣称的边界

- plan-only PASS 不是 controller execution、contact、lift、retention 或 grasp。
- 20-case matrix 不是整个工作空间覆盖。
- 三十条不同 trajectory digest 不等于 byte-identical planner determinism。
- 代理碰撞几何不是全机器人 mesh fidelity 或最小间隙证明。
- REAL_MOVEGROUP_NORMAL 不证明 real MoveGroup 的 cancel、timeout、late result 或 shutdown 健康。

## 小练习

1. 为什么一个 SUCCESS RobotTrajectory 仍需独立 validator？
2. 写出证明“零执行”至少需要的四个计数/结构条件。
3. Candidate009 应归为 IK 失败、controller 失败还是物理失败？为什么？
4. 用自己的话解释 shoulder_pan generator 为什么必须绕关节原点而不是 base 原点。
