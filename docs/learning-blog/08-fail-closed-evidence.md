# 第 8 章：fail-closed——P2 负证据与 P12d injected PASS 不能相加

## 学习目标

- 理解 fail-closed 是“证据不足就不放行”，不是“系统永不失败”；
- 区分 real normal plan-only、injected fake 和 historical real-process negative；
- 读懂 goal-response timeout、accepted-result timeout、cancel 和 late terminal；
- 知道哪些 injected-scoped 布尔值可以为 true，哪些 unqualified 字段必须保持 false。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| fail-closed | 依赖缺失、超时、身份不明或终态不可信时，默认不继续运动 |
| injected fake | 在进程内用可控 ActionServer 制造延迟、异常和晚到回调 |
| historical negative | 已经发生且保留的失败事实；不是可重复的成功基线 |
| goal-response timeout | 还没拿到 ClientGoalHandle 就超时 |
| accepted-result timeout | goal 已接受、有 handle/UUID，但最终 result 没按时来 |
| exact cancel | 对该次 accepted goal 的精确 handle 调 cancel |
| terminal confirmation | exact result future 到达 CANCELED/ABORTED 等终态；cancel 后若晚到 SUCCEEDED，仍按 race fail closed |
| strong correlation | request、attempt generation、client/server UUID 和 ledger 能直接 join |
| P2 | 仓库给一次历史 real-process 负实验的阶段标签；它不是 MoveIt 版本、测试等级或 PASS |
| P12d | 仓库给当前 in-process fake 合同验证快照的阶段标签；正结论必须保留 injected 范围 |

## 前置知识

- 已读第 2 章的 ROS Action 生命周期；
- 理解 plan-only 与 execution 分离；
- 接受一个组件 fail closed 的同时，上游进程仍可能 crash。

## 系统图/流程图

~~~mermaid
flowchart TB
    Q[要回答什么问题?]
    Q --> N[真实 MoveGroup 正常规划是否响应?]
    Q --> I[超时/取消/晚回调合同是否正确?]
    Q --> H[历史真实进程失败时 EdgeGrasp 做了什么?]
    N --> A[REAL_MOVEGROUP_NORMAL<br/>20/20 matrix matched<br/>30 trajectory validated/discarded<br/>zero execution]
    I --> B[INJECTED_CANCEL_TIMEOUT P12d<br/>12 tests / 12 JSONL / 15 facets PASS]
    H --> C[REAL_MOVEGROUP_CANCEL_RACE_NEGATIVE_EXISTING_ARTIFACT<br/>UPSTREAM_PROCESS_EXITED<br/>overall_pass=false]
    A -. 不能证明 cancel health .-> I
    B -. 不能证明 real MoveGroup health .-> N
    C -. 不能升级成 PASS .-> N
~~~

图源：[MoveIt evidence matrix](../observations/2026-08-29-moveit-evidence-matrix.json)、[P12d observation](../observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json) 和 [historical real negative](../observations/2026-08-29-real-moveit-fail-closed-runtime.json)。

替代文本：三个问题由三类不可交换证据回答：真实正常规划、进程内 injected 合同、历史真实进程失败；任一类都不能替代另两类。

~~~mermaid
sequenceDiagram
    participant W as PlanTarget wrapper
    participant F as send_goal future
    participant H as exact ClientGoalHandle
    participant R as exact result future
    W->>F: send_goal_async
    alt goal-response timeout
        Note over W,F: goal_response_future_pending=true
        Note over W: result_future_pending=null<br/>UUID 可为 null
        F-->>W: late accepted handle
        W->>R: bind exact result future
        W->>H: cancel_goal_async
        H-->>W: cancel acknowledged
        R-->>W: exact terminal confirmation
    else accepted
        F-->>W: handle + canonical UUID
        W->>R: get_result_async
        alt accepted-result timeout
            Note over W,R: goal_response pending=false<br/>result pending=true
            W->>H: cancel exact handle
            H-->>W: cancel acknowledged
            R-->>W: exact terminal required
        else result normal
            R-->>W: correlate and finish
        end
    end
~~~

图源：[runbook D9](../ubuntu-jazzy-runbook.md#d9-injected-movegroup-cancelgoal-response-timeout-contract) 和 [P12d JSONL facet contract](../observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json)。

替代文本：goal-response 超时时还没有 result future，UUID 可以为空；late accepted 后绑定精确 result future 并取消精确 handle，cancel acknowledgement 后仍要等待终态。终态可能是 CANCELED/ABORTED；若 cancel 后出现 SUCCEEDED，系统仍保持 fail closed。

## 实际操作

### 当前允许的安全复现路径

AGENTS.md 只允许在 Ubuntu/Jazzy 用进程内 fake runner，并把 artifact 写到全新仓库外目录。以下命令来自 runbook，本轮没有执行：

~~~bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
bash scripts/run_injected_moveit_fail_closed.sh \
  /home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_UNIQUE
~~~

运行前要求：

- 使用 disposable、已 source 的 Jazzy workspace；
- artifact 目录不存在；
- 确认脚本调用 in-process fake MoveGroup/typed-gate servers；
- 不启动真实 MoveGroup、controller 或 Gazebo；
- 只清理该 runner 自己的 exact process/domain。

### 历史 P2 只读

历史 real MoveGroup observation 只用于读取 JSON 状态、hash、计数和 claim boundary。仓库故意不包含可执行 OS-process interruption harness，不得重建、不得重跑，也不提供复现步骤。

## 期望输出

### P12d injected artifact

~~~text
evidence_class = INJECTED_CANCEL_TIMEOUT
summary.status = PASS
overall_pass = true
tests = 12/12
JSONL records = 12
required facets = 15/15
failures/errors/skips = 0
validation_errors = []
~~~

P12d 直接覆盖：

1. explicit cancel；
2. MoveGroup goal-response delay/timeout 与 late accepted cancel；
3. typed-gate goal-response delay/timeout；
4. accepted MoveGroup result-future timeout；
5. cancel terminal confirmed/unconfirmed；
6. success-after-cancel 仍 fail closed；
7. adapter 调用 typed gate 时，客户端侧 get_result_async 返回 None；
8. gate result request exception；
9. MoveGroup result-future request exception；
10. MoveGroup unavailable future adapter-seam injection；
11. old-generation late-CANCELED isolation；
12. old-generation late-SUCCEEDED isolation。

可以为 true 的是带 injected 后缀的字段，例如：

~~~text
accepted_goal_result_timeout_verified_injected
accepted_goal_result_future_timeout_verified_injected
strong_move_group_request_id_correlation_verified_injected
gate_delayed_goal_response_cancel_verified_injected
gate_result_future_unavailable_verified_injected
gate_result_exception_verified_injected
move_group_result_future_unavailable_verified_injected
old_generation_late_success_isolation_verified_injected
~~~

必须继续为 false：

~~~text
accepted_goal_result_timeout_verified
accepted_goal_result_future_timeout_verified
strong_move_group_request_id_correlation
move_group_result_future_unavailable_verified
old_generation_late_success_isolation_verified
real_move_group / controller / simulation_physics / hardware fields
~~~

### 历史 P2 real-process negative

两条历史记录共同是：

~~~text
summary.status = UPSTREAM_PROCESS_EXITED
overall_pass = false
edgegrasp_fail_closed = true
actual ExecuteTrajectory / arm FJT / gripper FJT goals = 0
move_group_survived = false
MoveGroup exit = -11 / SIGSEGV in PlanExecution::stop()
~~~

EdgeGrasp 零运动 fail closed 是正面的安全响应；MoveGroup 进程 crash 使整条记录必须是 negative，不得写 PASS。fresh graph 的晚状态不是 strong request-ID join，所以 unqualified correlation 保持 false。

## 踩坑记录

### 把 Track A 与 Track B 相加

P2 回答“上游进程失败时 EdgeGrasp 是否继续发 motion”，P12d 回答“可控 fake 下 Action 合同是否正确”。两个事实互补，但不能合成“真实 MoveGroup cancel 已健康”。

### 为了好看的总表删除 injected 后缀

一旦把 strong_move_group_request_id_correlation_verified_injected 改写成 strong_move_group_request_id_correlation，就把 fake 证据升级成真实证据。字段命名本身就是安全边界。

### late accepted goal 用全局 active handle 取消

goal response 超时后，全局 _active_moveit_goal 可能尚未设置。晚到 future 返回的 handle 才是要取消的 exact handle；调用通用 _cancel_move_group 可能找不到它。

### get_result_async 返回 None 后仍继续

不论是 gate 还是 adapter seam，没有 result future 就无法确认 terminal。正确行为是 fail closed，而不是假设 server 稍后自行结束。

这里必须分清两个同名但不同层的 seam：P12d 直接注入的是 adapter 作为 typed-gate 客户端时拿不到 result future；它没有直接覆盖 trajectory_gate.py 作为下游 controller 客户端时的 None 返回。当前静态审计看到后者已有 exception 分支，但没有独立的显式 None 分支；仓库里也没有一条可把它升级为 runtime failure 或 runtime PASS 的记录。因此本书把它记为 UNVERIFIED_STATIC_GAP，而不是把 P12d 的结果横向套用过去。

同理，plan_target_client.py 与 grasp_sequence/client.py 是一次性示例客户端，生命周期处理比生产 adapter 合同简化。它们可用于有界演示，但不能自动继承 adapter 对 late accepted、exact cancel 和 terminal join 的全部保证。

### cancel 后重复 emission 或 double cancel

P11 修过 duplicate gate terminal/unconfirmed emission 和 result timeout 后 double cancel。P12d 保留这些 source hardening，但没有把它们冒充新的独立 facet。

### late SUCCEEDED 升级旧 attempt

旧 generation 即使晚到 SUCCEEDED，也不能 dispatch 到新 attempt。P12d 增加了直接 stale-success isolation，弥补 P11 只直接覆盖 late-CANCELED 的边界。

## 排查过程

| 阶段 | handle/UUID | 应记录的 pending 字段 | 允许动作 |
| --- | --- | --- | --- |
| planning | 无 | 两个 result/goal future 尚无或不适用 | 只记录 request/generation |
| goal-response timeout | 可能无 | goal_response=true；result=null | 等 late handle；若接受则 exact cancel |
| accepted | 有 | goal_response=false | 建立 exact result future |
| accepted-result timeout | 有 | result=true | exact cancel；不能只看 ack |
| cancel acknowledged | 有 | result 仍 pending | 等 exact terminal |
| terminal | 有 | futures complete | join request/generation/UUID 与 ledger |

~~~mermaid
flowchart TD
    A[测试失败] --> B{证据 class 正确吗?}
    B -->|否| C[先修分类，不重跑真实进程]
    B -->|是| D{JSONL 条数/测试数/facet 数一致?}
    D -->|否| E[检查漏 emission、duplicate、validator]
    D -->|是| F{request + generation + UUID 能 join?}
    F -->|否| G[检查 nullable timing 与 exact handle]
    F -->|是| H{terminal confirmed?}
    H -->|否| I[保持 fail closed / escalation]
    H -->|是| J[只更新该证据层字段]
~~~

图源：[P12d safe runner](../../scripts/run_injected_moveit_fail_closed.sh) 与 [validation report](../validation-report.md) 的 MoveIt delivery addendum。

替代文本：失败排查先确认 evidence class，再核对测试、JSONL 和 facet 数；随后 join 身份并确认 exact terminal，最终只更新对应证据层字段。

## 最终证据

- REAL_MOVEGROUP_NORMAL：Candidate024 matrix 的 20/20 outcome、30 条 validated/discarded trajectory 和零 execution counters。
- P12d INJECTED_CANCEL_TIMEOUT：12/12 tests、12 JSONL、15/15 facets，validator error 为零。
- 历史 P2 REAL_MOVEGROUP_CANCEL_RACE_NEGATIVE_EXISTING_ARTIFACT：EdgeGrasp fail closed 且零 actual motion，但 MoveGroup 两次均进程退出，overall_pass=false。
- 当前仓库只保留安全 fake runner；历史 real OS-process harness 故意缺席。

证据层：三个并列且不可交换的 MoveIt evidence class。

## 仍不能宣称的边界

- 没有健康 real MoveGroup cancel、goal-response timeout 或 accepted-result timeout PASS。
- 没有真实 controller stop、motor power cut、torque stop 或 physical-stop guarantee。
- P12d unavailable MoveGroup future 是 adapter seam injection，不是真实 ClientGoalHandle 观察。
- P12d 的 typed-gate None seam 不是 trajectory_gate 下游 controller seam；后者当前仍是静态未验证缺口。
- 一次性示例客户端不等同于生产 adapter 的完整 late-goal 清理合同。
- P2 的晚状态不是 strong request-ID correlation。
- fail-closed 不表示上游不会 crash；它只限制未知状态下继续发运动。

## 小练习

1. goal-response timeout 与 accepted-result timeout 的 pending 字段为何相反？
2. 为什么 P2 不能因为 edgegrasp_fail_closed=true 就把 overall_pass 改成 true？
3. 设计一个 late accepted goal 的 exact-handle cancel 断言。
4. 写出把 injected claim 安全升级为 real claim 还缺的三类直接证据。
