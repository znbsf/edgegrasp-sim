# 第 6 章：四阶段抓取——协议完成与物理抓取分成两张成绩单

## 学习目标

- 理解 approach、descend、close_gripper、lift 的顺序和责任；
- 理解每个 stage 为什么必须等精确 terminal 才能前进；
- 理解新鲜 TrackedTarget、immutable orientation 和 trajectory digest；
- 区分 sequence_completed 与 physics_grasp_verified。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| approach | 先到目标附近的安全接近位姿 |
| descend | 从接近位姿走到夹取位姿 |
| close_gripper | 单独给夹爪 controller 发闭合轨迹 |
| lift | 夹闭后抬升 |
| immutable task | 一次任务开始后，目标身份、阶段序号和两组姿态不能偷偷变化 |
| correlated terminal | task、command、target、stage、sequence、digest、时钟都匹配的终态 |
| protocol result | 四个命令按合同走完；不回答物体是否被抓住 |
| physics result | 独立观察接触、抬升和保持后得到的结果 |

## 前置知识

- 已理解 PlanTarget 与 ExecuteTrajectory；
- 知道 arm 与 gripper 是两个不同 controller；
- 知道目标数据在 200 ms 后会过期，长任务不能复用最初时间戳。

## 系统图/流程图

~~~mermaid
stateDiagram-v2
    [*] --> APPROACH_PLAN
    APPROACH_PLAN --> APPROACH_EXEC: PlanTarget terminal matched
    APPROACH_EXEC --> DESCEND_PLAN: ExecuteTrajectory/FJT terminal matched
    DESCEND_PLAN --> DESCEND_EXEC: PlanTarget terminal matched
    DESCEND_EXEC --> CLOSE_GRIPPER_EXEC: arm terminal matched
    CLOSE_GRIPPER_EXEC --> LIFT_PLAN: gripper terminal matched
    LIFT_PLAN --> LIFT_EXEC: PlanTarget terminal matched
    LIFT_EXEC --> COMPLETE: arm terminal matched
    APPROACH_PLAN --> SAFE_STOP: any mismatch/timeout/stale
    APPROACH_EXEC --> SAFE_STOP: any mismatch/timeout/stale
    DESCEND_PLAN --> SAFE_STOP: any mismatch/timeout/stale
    DESCEND_EXEC --> SAFE_STOP: any mismatch/timeout/stale
    CLOSE_GRIPPER_EXEC --> SAFE_STOP: any mismatch/timeout/stale
    LIFT_PLAN --> SAFE_STOP: any mismatch/timeout/stale
    LIFT_EXEC --> SAFE_STOP: any mismatch/timeout/stale
~~~

图源：src/edgegrasp/grasp_sequence.py、GraspSequence.action 与 [runbook D1](../ubuntu-jazzy-runbook.md#d1-four-stage-correlated-sequence)。

替代文本：四阶段被拆成规划与执行子状态；每一步只有在匹配的 PlanTarget、ExecuteTrajectory 和 FJT 终态后前进，任意失配、超时或 stale 都进入 SAFE_STOP。

~~~mermaid
sequenceDiagram
    participant T as fresh TrackedTarget
    participant S as GraspSequence
    participant P as PlanTarget
    participant E as ExecuteTrajectory
    participant O as Physics observer
    T-->>S: same target_id, fresh stamp, domain, epoch
    S->>P: approach
    P-->>S: matched terminal + digest A
    S->>P: descend
    P-->>S: matched terminal + digest D
    S->>E: close_gripper
    E-->>S: matched gripper FJT terminal
    S->>P: lift
    P-->>S: matched terminal + digest L
    S-->>O: sequence terminal, exact lift command
    O-->>S: independent physics result
~~~

图源：[四个 Action schema](../../ros_ws/src/edgegrasp_interfaces/action/)、[GraspSequenceTerminal.msg](../../ros_ws/src/edgegrasp_interfaces/msg/GraspSequenceTerminal.msg) 和 [ros_ws README](../../ros_ws/README.md)。

替代文本：序列为每个阶段取得同一目标的新鲜观察，逐段等待规划与执行终态；完成后把 exact lift command 交给只读物理观察器，后者独立返回 physics result。

## 实际操作

以下是现有 runbook 的接口与 launch 命令，本轮没有启动：

~~~bash
ros2 interface show edgegrasp_interfaces/action/GraspSequence
ros2 launch edgegrasp_grasp_sequence grasp_sequence.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link
~~~

发送任何任务前应依次确认：

1. typed target publisher、safety monitor、interface probe、PlanningScene、MoveIt adapter 和 trajectory gate 健康；
2. 只有一个 /clock publisher；
3. 三个 controller active；
4. approach quaternion 与 grasp quaternion 均有限、归一化、非零；
5. approach 使用第一组 orientation，descend/lift 共用第二组；
6. 三个 position 和 gripper start state 经过当前 graph 重新验证，不能复制旧绝对值。

协议重复性脚本使用直接安装后的 entrypoint，而不是后台 ros2 run wrapper：

~~~bash
bash scripts/run_grasp_sequence_repetitions.sh 10 \
  /home/edgegrasp/ros2_ws/test_results/grasp_repeat_UNIQUE \
  grasp-direct-accept \
  '[0.391231968, -0.001571668, 0.256520737]' \
  '[0.391231968, -0.001571668, 0.251520737]' \
  '[0.391231968, -0.001571668, 0.261520737]' \
  '[0.017007859, 0.706463960, 0.013976791, 0.707406570]' \
  0.2
~~~

这些坐标只属于历史 integration smoke 的已记录起始状态，不可在另一 graph 直接复用。

## 期望输出（历史 artifact 示例）

上面的 launch/interface 命令和历史坐标并不足以在另一 graph 产生同样结果。下面是 corrected repeat artifact 的已记录输出，用来教读者读证据，不是本轮新运行：

~~~text
10/10 sequence_completed=true
wrapper terminal status = 4
每次 sequence 进程干净退出
physics_grasp_verified=false
10 个 final trajectory digest 不同
~~~

因此它证明“协议结果重复性”，不证明规划字节确定性或物体抓取。

Candidate024 的 physics success 还需另一个结果：

~~~text
sequence_completed=true
exact lift command terminal matched
both configured pad tokens in same-sample contact
peak/retained lift >= 0.020 m
retention duration >= 0.5 s
physics_grasp_verified=true
~~~

## 踩坑记录

### 四阶段复用一个旧 target timestamp

完整任务持续数秒，最初 target stamp 必然超过 200 ms。wrapper 现在每段绑定同一 immutable target 的新鲜 TrackedTarget，检查位置漂移，并保留 target ID/domain/epoch。简单增大 timeout 不是解决方案。

### 零四元数或阶段中途改 orientation

approach 可用 collision-routing orientation；descend 与 lift 必须共享 grasp orientation。两组都在任务开始时冻结，零/无效 quaternion 直接拒绝。

### topic 顺序猜测命令属于哪一段

旧 diagnostic topic 缺少完整身份。新路径只使用 typed actions，核对 task_id、command_id、target_id、stage、sequence_no 和 trajectory_digest。

### client cancel 与晚到 success 竞争

如果 cancel 已经决定 fail closed，晚到 success 不能把任务升级为成功。callback 要按 generation 过滤，并让 cancel decision 在 terminal race 中保持优先。

### rclpy teardown 过早销毁

早期测试把“sequence slot 已空”误当 ActionServer coroutine 已退出，随后 destroy node 产生 InvalidHandle/Destroyable。修复是等待 pending action work、join executor worker pool，并把 retained callback exception 变成测试失败。

### 后台 ros2 run 的 PID 不是实际 server PID

repeat harness 最初管理的是 CLI wrapper，导致 invalid batch 和 lingering process。修复后直接解析并启动 installed console-script entrypoint，每次使用独立 action endpoint。

## 排查过程

| terminal 现象 | 首先核对 | 处理 |
| --- | --- | --- |
| task_source_stale / target_source_stale | 当前 stage 的 TrackedTarget stamp | 取同 target 的新鲜观察，不放宽 200 ms |
| joint_state_stale | exact joint state receive time | 不发下一段；请求取消并等 terminal |
| command/digest mismatch | task/stage/sequence/digest | 保留下游 slot，拒绝 reset |
| FJT status 非法 | wrapper 与 downstream terminal | 不能只看 outer action success |
| sequence completed，physics false | observer reason、contact/lift/retention | 记录为协议成功 + 物理负证据 |
| shutdown warning | executor pending tasks 与进程所有权 | 等待 quiescence，使用 direct entrypoint |

Candidate024 r02 是典型例子：五个 baseline sample 把同一 target 从新鲜拖到 planning boundary 208 ms。安全门正确零运动拒绝。修复在 client 添加 100 ms pre-send admission recheck，并且只在 correlated CANCELED/observer_cancelled terminal 后重建 observer，没有修改 core 的 200 ms deadline。

## 最终证据

- dependency-free sequence core 有 60 个测试；Jazzy action/client 当前 45/45。
- corrected direct-entrypoint repeat artifact 10/10 protocol complete，并保留早期 4/10、invalid PID 与 shutdown-race 失败。
- Candidate024 r04 实际走过 stale observer cancel、identity matching、52 ms replacement 的恢复路径。
- GraspSequence.action 的注释和字段明确 sequence_completed 不等于 physics_grasp_verified。

证据层：core/static + scoped ROS/controller protocol；物理结论来自独立 observer。

## 仍不能宣称的边界

- 10/10 protocol 不是物理抓取、规划 determinism 或 hardware success。
- cancel accepted 不是确认停止。
- wrapper 仍没有自动化所有 target-contact scene transition。
- 跨节点 epoch reset 尚未成为一个原子分布式操作。
- rclpy 最终测试干净不等于所有 ROS 进程都有普适 clean-shutdown 保证。

## 小练习

1. 为什么 descend 与 lift 共享 grasp orientation，而 approach 可以不同？
2. 列出一个 stage 前进必须关联的六个身份字段。
3. sequence_completed=true、physics_grasp_verified=false 是否矛盾？举一个真实候选例子。
4. 解释 r02 为什么应修 client scheduling，而不能把 200 ms 改成 300 ms。
