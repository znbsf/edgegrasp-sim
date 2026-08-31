# 第 2 章：ROS 2 类型化合同——身份、时间与 Action

## 学习目标

- 用白话区分 ROS 2 的 topic、service 和 action；
- 理解 TrackedTarget 为什么比 PointStamped 多身份、时钟域和 epoch；
- 理解 PlanTarget、ExecuteTrajectory、GraspSequence、GraspPhysicsEvidence 的分工；
- 知道“收到 cancel”与“下游已经终止”是两件事。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| node | ROS graph 中一个有明确职责的逻辑组件；一个进程可以承载一个或多个 node |
| topic | 持续广播，发送者通常不等待单条回复 |
| service | 一问一答，适合短请求 |
| action | 可持续一段时间、能反馈和取消，并最终返回 terminal result 的任务 |
| PointStamped | 一个三维点，加上它属于哪个坐标 frame 和何时观测到的时间戳 |
| frame | 坐标的参照系；同一组三个数字放在不同 frame 中含义可能完全不同 |
| clock domain | 时间戳来自哪套时钟，例如 ROS simulation time 或 system time |
| FJT | FollowJointTrajectory；controller 接收关节时间轨迹并返回终态的 Action |
| goal handle | 客户端拿到的“这一个 Action goal”的句柄 |
| future | 将来才会完成的异步结果盒子 |
| request_id | 应用层一次命令的身份 |
| attempt_generation | adapter 本地递增的尝试代数，用来隔离旧 callback |
| UUID | ROS Action goal 的 16-byte 身份。本项目当前 adapter 通常在拿到 ClientGoalHandle 后记录；若 client 显式预生成 goal_uuid，也可更早存在，但本项目未采用该路径 |
| epoch | 时间线的一代；clock rollback 后必须进入新一代，旧消息不能混进来 |

## 前置知识

- 已完成第 1 章，知道 fail closed 与状态机；
- 知道异步函数不是立刻返回最终结果；
- 知道 position 和 timestamp 都必须绑定到一个明确 frame/clock。

## 系统图/流程图

~~~mermaid
flowchart TB
    T[TrackedTarget<br/>target_id + PointStamped + domain + epoch]
    P[PlanTarget.action<br/>不可变规划请求]
    E[ExecuteTrajectory.action<br/>不可变轨迹命令]
    S[GraspSequence.action<br/>四阶段协议]
    O[GraspPhysicsEvidence.action<br/>只读物理观察]
    T --> S
    S -->|arm stages| P
    P -->|规划并验证 arm trajectory| E
    S -->|gripper close| E
    S --> O
    E --> F[下游 FollowJointTrajectory terminal]
    O --> R[接触 + lift + retention]
~~~

图源：[四个 Action schema](../../ros_ws/src/edgegrasp_interfaces/action/) 与 [TrackedTarget.msg](../../ros_ws/src/edgegrasp_interfaces/msg/TrackedTarget.msg)。

替代文本：TrackedTarget 进入四阶段序列；arm 阶段调用 PlanTarget，adapter 规划并验证后交给 typed ExecuteTrajectory gate，gripper close 则由序列直接调用 ExecuteTrajectory；执行 Action 关联下游 FJT 终态，独立物理 Action 只观察接触、抬升和保持。

~~~mermaid
sequenceDiagram
    participant C as Client
    participant A as EdgeGrasp adapter
    participant M as MoveGroup / fake server
    C->>A: PlanTarget(request_id)
    A->>M: send_goal_async
    Note over A,M: 此时只有 goal-response future；本项目尚未记录 UUID
    M-->>A: ClientGoalHandle accepted；记录 UUID
    A->>M: get_result_async
    alt result 正常终止
        M-->>A: terminal result
        A-->>C: correlated PlanTarget terminal
    else timeout / permission loss
        A->>M: cancel_goal_async(exact handle)
        M-->>A: cancel acknowledged
        Note over A: acknowledged 仍不等于 terminal
        M-->>A: exact terminal confirmation
        Note over A,M: 常见 CANCELED/ABORTED；cancel 后 SUCCEEDED 仍 fail closed
        A-->>C: fail-closed terminal
    end
~~~

图源：[P12d observation](../observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json)、adapter integration tests 和 [runbook D9](../ubuntu-jazzy-runbook.md#d9-injected-movegroup-cancelgoal-response-timeout-contract)。

替代文本：客户端发 PlanTarget，adapter 先等待 goal response；当前实现接受后才从 goal handle 记录 UUID，再等待 result。超时需要取消精确句柄，并在看到该 result future 的精确终态前保持 fail closed；终态不保证一定是 CANCELED。

## 实际操作

以下是接口检查命令，不启动运动；本轮没有执行：

~~~bash
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 interface show edgegrasp_interfaces/msg/TrackedTarget
ros2 interface show edgegrasp_interfaces/action/PlanTarget
ros2 interface show edgegrasp_interfaces/action/ExecuteTrajectory
ros2 interface show edgegrasp_interfaces/action/GraspSequence
ros2 interface show edgegrasp_interfaces/action/GraspPhysicsEvidence
~~~

包级构建/测试命令来自 ros_ws/README：

~~~bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test \
  --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --packages-select edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
    edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --return-code-on-test-failure
colcon test-result \
  --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --all --verbose
~~~

这些命令只应在隔离 Ubuntu 24.04 / ROS 2 Jazzy workspace 中运行。复制整个 edgegrasp-sim 工程，不要只复制 ros_ws/src/edgegrasp_core；它是有意依赖 monorepo 布局的 adapter。

## 期望输出

当前 P12d 包级 artifact 的选定测试结果：

| package | passed |
| --- | ---: |
| edgegrasp_core | 1/1 |
| edgegrasp_interfaces | 0 tests |
| edgegrasp_ros | 49/49 |
| edgegrasp_moveit_adapter | 34/34 |
| edgegrasp_grasp_sequence | 45/45 |
| 合计 | 129/129 |

P12d 保存的包级 artifact 来自当时的 uncommitted worktree；observation 同时保存 baseline/current source hash，可与现在的 main 文件追溯核对，但它不是“本轮在当前 main clean checkout 上 fresh rerun”的结果。

XML 为零 failure/error/skip。这个结果说明选定包级测试在该 Jazzy 环境通过，其中包含 static、fake 和 injected 分支；它不说明真实 MoveGroup cancel 健康、controller 物理停止或硬件安全，也不计入 focused 12-test/15-facet artifact。

接口字段中应该看到：

- TrackedTarget：target_id、observation、clock_domain、clock_epoch；
- PlanTarget result：request 所属 task/stage/sequence、trajectory_dispatched、digest、gate terminal、downstream terminal；
- ExecuteTrajectory result：accepted、terminal、downstream_terminal_observed、cancel_requested、FJT status；
- GraspSequence result：sequence_completed 与 physics_grasp_verified 分列；
- GraspPhysicsEvidence result：双指垫、lift、retention 和 evidence_digest。

## 踩坑记录

### 用 PointStamped 承担它没有的字段

PointStamped 只有 header 和 point，没有 target_id、clock_domain 或 epoch。旧 safety wrapper 因此只能自己维护本地 epoch。TrackedTarget 原子地携带这些字段，但 legacy safety/gate 节点尚未全部改用它；跨节点 reset coordination 仍未验证。

### 在 goal response 前强行写 UUID

send_goal_async 返回的是 goal-response future。ClientGoalHandle 尚未出现时，UUID 可以是 null。不能从 adapter 私有状态编造一个 UUID。若未来合同要求发送前就有 UUID，必须显式给 send_goal_async 预生成 goal_uuid，并新增可审计 stage；当前项目没有这样做。

### cancel accepted 就释放下游 slot

取消请求被接受只说明 server 收到了取消意图。若 terminal 永远不来，机械臂状态仍未知。EdgeGrasp 会继续等待有界 terminal，失败后锁存错误并要求更低层 stop escalation。

### 旧 generation 的 callback 污染新尝试

第一次 attempt 超时后，晚到的 CANCELED 或 SUCCEEDED 可能在第二次 attempt 运行时触发。如果 callback 没有捕获 generation，它可能清空新状态或错误地 dispatch。P12d 直接测试了 late-CANCELED 与 late-SUCCEEDED isolation。

## 排查过程

~~~mermaid
flowchart TD
    A[某个 Action 卡住] --> B{goal response 到了吗?}
    B -->|否| C[仅检查 goal-response future<br/>result future 应为 null]
    B -->|是| D[记录 exact handle / UUID / generation]
    D --> E{result 到了吗?}
    E -->|是| F[核对 request + generation + UUID + terminal]
    E -->|否| G[cancel exact handle]
    G --> H{exact terminal 到了吗?}
    H -->|是| I[fail-closed 完成]
    H -->|否| J[锁存错误 / stop escalation]
~~~

图源：[PlanTarget.action](../../ros_ws/src/edgegrasp_interfaces/action/PlanTarget.action)、[ExecuteTrajectory.action](../../ros_ws/src/edgegrasp_interfaces/action/ExecuteTrajectory.action) 与 [P12d timeout/cancel facets](../observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json)。

替代文本：排查 Action 时先分辨 goal response 和 result 两个 future；接受后记录精确句柄与 UUID，result 超时才取消，未看到精确 terminal 就锁存错误。

实际日志核对应保持同一个三元组：

~~~text
(request_id, attempt_generation, move_group_goal_id)
~~~

request_id 与 attempt_generation 从尝试开始就固定。planning 或 goal-response timeout 发生在 handle 之前，move_group_goal_id 可以为空；accepted 或 late-accepted 后 UUID 才在当前实现中落盘，一旦获得，result-timeout、cancel 和 terminal 阶段必须沿用同一个 UUID。

## 最终证据

- interfaces 文件明确把协议完成与物理成功分开。
- edgegrasp_ros 的 49 个 Jazzy 测试覆盖 typed target、fake FJT、clock/watchdog、PlanningScene/ACM 和 physics observer。
- 保存的 edgegrasp_moveit_adapter P12d 包级 artifact 为 34/34；focused injected runner 为 12/12、12 JSONL、15/15 facets。
- P12d 的强 request/generation/UUID join 只在 injected-scoped 字段为 true。

证据层：接口定义属于 static contract，129/129 属于 scoped Jazzy package validation，focused P12d 才属于 injected fake runtime。

## 仍不能宣称的边界

- 不能说 PointStamped 的跨节点 epoch 已解决。
- 不能说 cancel acknowledgement 等于 controller 或物理停止。
- 不能用 injected UUID 关联证明真实 MoveGroup 的全部生命周期健康。
- 不能把 sequence_completed 写成 physics_grasp_verified。
- 不能从 Action server 存在推导 controller、Gazebo 或硬件已运行。

## 小练习

1. 画出 goal-response future 与 result future 的先后关系。
2. 为什么 goal-response timeout 时 result_future_pending_at_timeout 应为 null？
3. 设计一个旧 generation 的 late-SUCCEEDED 测试：新 attempt 不应受到哪些影响？
4. 列出 PointStamped 相比 TrackedTarget 缺少的三个安全字段。
