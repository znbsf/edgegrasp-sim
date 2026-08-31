# 第 1 章：先不用 ROS——纯 Python 确定性核心

## 学习目标

读完本章，你应该能：

- 解释为什么机器人项目先写一个不依赖 ROS 的核心；
- 说清 Target3D、预测器、安全门、状态机和 backend 的职责；
- 区分 source freshness 与本地消息流 liveness；
- 运行或阅读 deterministic replay，并知道它不证明仿真物理确定性。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| deterministic | 相同输入与相同时间序列，多次运行得到相同结果与摘要 |
| Target3D | 一个带身份、三维位置、来源时间、坐标系、置信度、时钟域和 epoch 的目标快照 |
| predictor | 根据最近目标估计短时间之后的位置；当前合成场景用常速度模型 |
| gate | 门卫；字段、时间或范围不合规则拒绝继续 |
| backend | 最后执行计划的接口；core 中只有 mock 真正执行，Gazebo/real 选择默认 fail closed |
| SAFE_STOP | 已知不应继续运动，停止成功后锁存的安全状态 |
| ERROR | 连停止结果都不可信，或内部合同被破坏时的错误状态 |

## 前置知识

只需知道：

- Python 类可以把数据和行为放在一起；
- 单元测试用输入检查输出；
- 纳秒是十亿分之一秒；
- 状态机只允许预先声明的状态跳转。

本章不要求 ROS、Linux、机械臂运动学或 Gazebo。

## 系统图/流程图

~~~mermaid
flowchart LR
    A[Target3D] --> B[字段/时间/坐标系检查]
    B --> C[常速度预测]
    C --> D[EndpointWorkspaceGate]
    D --> E[MotionPlan 合同检查]
    E --> F{backend}
    F -->|mock| G[同步返回执行结果]
    F -->|gazebo / real 未配置| H[fail closed]
    B -->|无效、未来、过期、回退| I[SAFE_STOP / ERROR]
    D -->|端点越界或被挡| I
~~~

图源：[README Architecture](../../README.md#architecture)、[simulation plan 架构](../simulation-plan.md#2-architecture-and-contracts)、src/edgegrasp/controller.py。

替代文本：Target3D 先经过字段与时间检查、预测和端点门，再检查 MotionPlan；只有 mock backend 在纯核心中执行，未配置的 Gazebo 或 real backend 会 fail closed。

~~~mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TRACKING: 收到目标
    TRACKING --> PLANNING: 预测完成
    PLANNING --> EXECUTING: 计划与执行边界均有效
    EXECUTING --> TRACKING: 执行完成且继续跟踪
    EXECUTING --> IDLE: 执行完成
    IDLE --> SAFE_STOP: 输入不安全
    TRACKING --> SAFE_STOP: 过期/回退/门拒绝
    PLANNING --> SAFE_STOP: 计划矛盾或目标变旧
    EXECUTING --> SAFE_STOP: watchdog/手动停止
    IDLE --> ERROR: 内部异常
    TRACKING --> ERROR: 内部异常
    PLANNING --> ERROR: 停止或 backend 异常
    EXECUTING --> ERROR: 停止或 backend 异常
    SAFE_STOP --> IDLE: 合法 reset
    ERROR --> IDLE: 合法 reset；时钟故障需新 epoch
~~~

图源：[fsm.py](../../src/edgegrasp/fsm.py) 与 [controller.py](../../src/edgegrasp/controller.py)。

替代文本：正常路径是 IDLE、TRACKING、PLANNING、EXECUTING；任何阶段都可能进入 SAFE_STOP 或 ERROR，只有受约束的 reset 才回到 IDLE。

## 实际操作

以下命令来自当前 README；本轮写作没有重新执行。Windows 安全静态路径是：

~~~powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
powershell -NoProfile -File scripts\check.ps1 -ReplayRuns 100 -PythonExecutable .\.venv\Scripts\python.exe
~~~

想把动作拆开看，可以运行：

~~~powershell
.\.venv\Scripts\python.exe -m pytest --collect-only -q
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m edgegrasp replay --runs 100
.\.venv\Scripts\python.exe scripts\validate_project.py
~~~

scripts/check.ps1 会临时把项目 src 放入 PYTHONPATH，并在 finally 中恢复原值。标准 packaging 测试还会清除 PYTHONPATH/PYTHONHOME，安装到临时目录后用隔离解释器导入，从而防止“只因当前目录碰巧可见所以 import 成功”。

来源：scripts/check.ps1、tests/test_packaging.py 和 [README Quick start](../../README.md#quick-start-deterministic-core)。

## 期望输出

当前提交记录的 Windows P12d 边界是：

~~~text
collected: 332
passed:    330
skipped:   2  # 仅因可选固定 SO-101 checkout 在当前 worktree 中缺席
0 / 20 / 40 mm/s 三种场景：各 replay 100 次，摘要稳定
structural validation: PASS
~~~

更早的完整固定上游布局是 332/332。两者不能拼成一个“更大的成功数”。

三种 replay 的历史摘要前缀分别是 0a6ba6f7、854cf2da、da7fcd92；每种场景接受 20/20 cycles。摘要相同只说明纯核心输入/输出确定，不说明 Gazebo 的接触、求解器或 controller 可重复。

来源：[validation report Windows snapshot](../validation-report.md#windows-core-structure-replay-and-syntax)。

## 踩坑记录

### 把端点 AABB 当成碰撞规划

EndpointWorkspaceGate 只检查最终点是否在保守盒子里、是否落入 blocked AABB、frame 是否正确。它不知道机械臂连杆、扫掠路径、IK 或障碍物几何。项目曾删除误导性的 planning_collision 命名，统一为 endpoint_blocked。

### 只在接收时检查目标新鲜度

如果规划花了 150 ms，接收时只有 80 ms 的目标到执行边界就变成 230 ms。旧调用点没有传 execute_now_ns，可能把接收时间误当执行时间。现在 execute_now_ns 是必填，并在 backend 前重新检查。

### 把来源时间当作消息仍在流动

一条旧消息可以带一个看似合法的时间，但不能证明订阅端持续收到新消息。因此：

- source freshness：当前时间减目标来源时间；
- receive liveness：当前时间减本地最后一次收到新鲜消息的时间。

二者都是 200 ms 边界，但回答不同问题。恰好 200 ms 允许，200 ms + 1 ns 拒绝。

### 选择 gazebo/real backend 就认为会运行

纯 core 中这两个 backend 是 disabled placeholder，会返回 adapter_not_configured。它们的存在是接口设计，不是 runtime 实现证据。

## 排查过程

| 现象 | 证据先看哪里 | 根因判断 | 最小处理 |
| --- | --- | --- | --- |
| import edgegrasp 失败 | 使用的 Python、pip target、PYTHONPATH | 解释器与安装位置不一致 | 用同一个显式 Python 做 venv、pip 和测试 |
| stale_target | target timestamp 与 execute_now_ns | 规划期间目标变旧 | 获取新目标；不能放宽 200 ms 掩盖延迟 |
| clock_rollback | 当前 now_ns 与上次 now_ns | 时间回退或 replay rewind | fail closed；协调新 epoch 后 reset |
| endpoint_blocked | 终点和 blocked AABB | 只是端点门拒绝 | 不要写成 MoveIt 碰撞；后续用 plan-only |
| stop_failed / stop_exception | backend stop result | 无法确认停止 | 进入 ERROR，不允许直接回 IDLE |

推荐用“证据 → 根因 → 最小修复 → fresh rerun”的顺序，而不是同时改时间阈值、空间范围和 backend。

## 最终证据

- pyproject.toml 声明 Python >=3.10、项目版本 0.1.0、runtime dependencies 为空。
- tests/test_packaging.py 验证标准 distribution 在没有 source PYTHONPATH 时可导入。
- src/edgegrasp/fsm.py 明确六个状态与合法转移。
- tests/test_safety.py 和 tests/test_controller.py 覆盖 200 ms 边界、slow planning、watchdog、clock rollback、stop failure。
- 当前 Windows checkout 330 passed + 2 可解释 skip；三种 100-run replay 保持确定。

证据层：core/static。

## 仍不能宣称的边界

- 不能说 endpoint gate 做了 IK、扫掠碰撞或 MoveIt 规划。
- 不能说 replay 证明 Gazebo physics、controller 或抓取确定性。
- 不能说 disabled Gazebo/real backend 已接入。
- 不能从 mock 同步 stop 推导真实电机已经停住。
- 不能把单元测试通过写成硬件安全认证。

## 小练习

1. 为什么 exactly 200 ms 可以通过，而 200 ms + 1 ns 必须拒绝？写出一个边界测试。
2. 给出一个“source freshness 合格但 receive liveness 已超时”的例子。
3. 如果 backend.stop 抛异常，状态应是 SAFE_STOP 还是 ERROR？说明证据差别。
4. 找出 EndpointWorkspaceGate 能检查和不能检查的项目，各列三项。
