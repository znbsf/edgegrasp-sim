# 第 4 章：PlanningScene 与 ACM——允许指垫接触，不等于取消碰撞

## 学习目标

- 理解 Gazebo world 与 MoveIt PlanningScene 为什么必须来自同一场景合同；
- 理解 ACM 是“成对碰撞规则”，不是全局关闭碰撞；
- 读懂 Candidate004 的选择性指垫接触证据；
- 学会先确认 scene，再规划，并把 revalidation 与首次 readiness 分开。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| PlanningScene | MoveIt 眼中的机器人和障碍物世界 |
| scene contract | 一份固定命名、frame、尺寸、pose 和 digest 的 JSON 事实 |
| SDF | Gazebo world/model 使用的 XML 格式 |
| ACM | Allowed Collision Matrix；指定“哪两个对象允许接触” |
| collision proxy | 用盒子等保守几何替代复杂 mesh |
| OBB SAT | 对有方向盒子做分离轴检测，比只看轴对齐 AABB 更精确 |
| revalidation | scene 首次确认后，周期性再查询它是否仍一致 |

## 前置知识

- 已理解 plan-only；
- 知道同一个 cube 必须在 Gazebo 和 MoveIt 中拥有一致的位置与尺寸；
- 知道“目标物允许接触”不能等于“所有机器人连杆都可穿过目标”。

## 系统图/流程图

~~~mermaid
flowchart TB
    J[scene.json] --> G[generate_scene_sdf.py]
    G --> S[table_cube.sdf<br/>Gazebo world]
    J --> L[planning_scene_loader]
    L --> M[MoveIt PlanningScene]
    M --> A[selective ACM]
    A --> P1[fixed finger pad ↔ target: allowed]
    A --> P2[moving finger pad ↔ target: allowed]
    A --> F1[gripper_link ↔ target: forbidden]
    A --> F2[moving jaw parent ↔ target: forbidden]
    M --> R[ready=true + frame + digest + exact objects]
~~~

图源：ros_ws/src/edgegrasp_ros/config/scene.json、planning_scene_loader.py、[runbook C1/D4](../ubuntu-jazzy-runbook.md#c1-load-and-confirm-the-shared-planningscene) 和 [Candidate004 observation](../observations/2026-08-27-candidate004-selective-acm-plan-only.json)。

替代文本：同一 scene JSON 一路生成 Gazebo SDF，一路加载到 MoveIt；选择性 ACM 只允许两个指垫 child links 接触 target，掌部和父连杆仍禁止。

## 实际操作

以下只记录现有 runbook 步骤，本轮没有启动 MoveIt 或 Gazebo。

在唯一 Gazebo graph 和 MoveGroup 已就绪后，启动 scene loader：

~~~bash
ros2 launch edgegrasp_ros planning_scene.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link include_optional_cube:=false

ros2 service list -t | grep -F '/get_planning_scene [moveit_msgs/srv/GetPlanningScene]'
ros2 topic echo --once --full-length /edgegrasp/planning_scene_status
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 24}}"
~~~

mask 24 表示 WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY。继续前至少核对：

~~~text
ready = true
reason = confirmed
frame = base_link
scene_digest = expected digest
objects include edgegrasp_table
~~~

若测试需要目标 cube，显式使用 include_optional_cube=true；若进入预先定义的 target-contact 阶段，必须通过专用服务切换，并等待新的 confirmed status。不能删除 table，也不能借移除 cube 隐藏非目标碰撞。

## 期望输出（已登记 artifact 示例）

上面的命令只完成 scene inventory/confirmation，不能单独产生下面的 plan-only 结论。以下结果来自已登记的 Candidate004 artifact；要重新获得它，还必须按第 3 章的隔离 plan-only 流程运行对应候选并保存新 observation。

Candidate004 运行时 ACM 允许关系：

| 与 target cube 的 pair | allowed |
| --- | --- |
| edgegrasp_fixed_finger_pad_link | true |
| edgegrasp_moving_finger_pad_link | true |
| gripper_link | false |
| moving_jaw_so101_v1_link | false |

在这个 scene 中：

- approach-to-descend 因 cube 对 gripper_link 碰撞被 Pilz ValidateSolution 拒绝；
- closed-gripper descend-to-lift control 通过 plan-only；
- Candidate004 没有发布轨迹、没有 ExecuteTrajectory/FJT goal，也没有执行。

这证明 selective ACM 与 plan-only rejection/acceptance 边界，不证明物理抓取。

## 踩坑记录

### AABB 预检误报

早期只用 AABB 判断指垫与 cube 的相对关系，会把旋转后的盒子近似得过于粗糙。Candidate004 改用 exact OBB SAT，并把 Gazebo 指垫碰撞放到两个专用 child links，才能对 ACM 做精确 pair 控制。

### 只看一个 ready 布尔值

ready=true 还不够。必须核对 reason、frame、digest、对象名称和几何。跨节点 epoch 目前仍是本地管理，因此 reset 后应整组重启或协调同一 epoch。

### revalidation 期间发出假 false pulse

曾经 loader 已确认 scene，却在自己发起的周期查询尚未完成时短暂发布 false，trajectory gate 随即取消并锁存 stop-unconfirmed。修复后，已确认状态会保留到 timeout、service loss、mismatch 或 clock fault，而不是在正常 in-flight query 时抖动。

### ACM service 返回 false 仍继续规划

Candidate014 的第一轮表面上像几何失败，实际是 harness 在 base scene 未确认前调用 selective ACM，服务返回 success=false，而旧脚本忽略了它。修复后必须依次看到 base confirmation、SetBool success=true、后续 confirmed ACM status。

### 生成 SDF 与 scene JSON 不一致

测试要求 checked-in SDF 与生成器输出 byte-equal。仅在运行时“看起来差不多”不够；第 9 章会解释 LF/raw-byte hash。

## 排查过程

~~~mermaid
flowchart TD
    A[规划被 scene 拒绝] --> B{scene JSON 可解析且 digest 匹配?}
    B -->|否| C[修合同/重新生成 SDF]
    B -->|是| D{首次 get_planning_scene 已 confirmed?}
    D -->|否| E[等待精确 frame/object/geometry]
    D -->|是| F{ACM service success=true?}
    F -->|否| G[停止；不把它归因于候选几何]
    F -->|是| H{后续 status 确认 exact pairs?}
    H -->|否| I[零规划/零执行]
    H -->|是| J[运行 plan-only 并记录 collision pair]
~~~

图源：[planning_scene_loader.py](../../ros_ws/src/edgegrasp_ros/edgegrasp_ros/planning_scene_loader.py)、[validation report](../validation-report.md) 的 revalidation 回归和 [Candidate014 observation](../observations/2026-08-28-candidate014-moving-pad-runtime.json) 的 harness race。

替代文本：先检查 JSON/digest，再等首次 PlanningScene 确认，接着要求 ACM 服务成功并由后续状态确认 exact pairs，最后才允许 plan-only；任一步失败都保持零执行。

排查时保存 MoveIt 明确报告的 collision pair 和 trajectory index。不要把 INVALID_MOTION_PLAN、NO_IK_SOLUTION、scene not ready 混成同一个“规划失败”。

## 最终证据

- 一个 scene contract 同时生成 Gazebo 的 table/cube SDF 与 MoveIt objects。
- runtime 确认 table+cube、table-only，以及仅两个 distal-pad child links 的 selective ACM。
- Candidate004 在 retained cube 下显示 forbidden parent collision 能阻止 plan-only，而允许指垫 pair 没有放开父连杆。
- revalidation false pulse 与 Candidate014 ACM ordering race 都有回归修复记录。

证据层：static scene contract + scoped PlanningScene/ACM runtime + plan-only。

## 仍不能宣称的边界

- 不能宣称所有机器人 link 都有高保真碰撞几何。
- 不能从 ACM allowed 推导接触力、稳定夹持或 force closure。
- 不能从日志未出现 DART geometry error 推导“碰撞一定正确”。
- 不能把删除 optional cube 当作通用避碰方案。
- 没有测量全路径最小 clearance。

## 小练习

1. 为什么只允许 pad child links 比允许 gripper_link 更安全？
2. 设计一个检查 scene status 的字段清单。
3. 如果 ACM service 返回 false，但随后 MoveIt 给出 NO_IK_SOLUTION，实验应归因于哪个更早的失败层？
4. AABB 与 OBB SAT 在旋转 cube 上会出现什么差别？
