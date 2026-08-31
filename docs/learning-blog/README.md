# EdgeGrasp 零基础图文学习博客

> 证据快照：仓库 main 提交 211bbced68c3b2f4819822388ec888fb20821f9d，2026-08-30（Asia/Shanghai）。
>
> 阅读边界：这套文章记录的是纯 Python、ROS 2、MoveIt plan-only、controller、Gazebo 代理物理和 injected fake 的分层实践。仓库没有真实 SO-101 硬件抓取、真实电机停止、相机外参或力闭环证据。

这不是一篇“让机械臂动起来就算成功”的教程。它从一个不依赖 ROS 的确定性 Python 核心开始，逐步接上 ROS 2 的类型化消息、MoveIt 的只规划接口、PlanningScene、Gazebo controller、四阶段抓取和独立物理观察器，最后解释为什么 fail-closed 系统必须把“停止请求”“终态确认”“仿真接触”“硬件事实”分开记录。

## 适合谁

- 会使用 Windows 文件系统和 PowerShell，但可以从零开始学习 Python、Linux 与 ROS 2。
- 想理解机器人软件中的时间戳、坐标系、Action、规划、controller 和证据边界。
- 希望看到失败候选怎样推动下一轮单变量实验，而不是只看最终成功截图。
- 愿意把“暂无可靠数据”当作正确结论，而不是用推测填空。

## 先记住六层证据

| 层级 | 本书中的含义 | 绝不能自动升级成 |
| --- | --- | --- |
| core/static | Python 单元测试、解析、结构、可重复 replay | ROS、规划器或物理运行 |
| plan-only | MoveIt 返回并验证轨迹，随后丢弃；执行计数为零 | controller 执行或抓取 |
| injected fake | 进程内假的 ActionServer 验证超时、取消和终态合同 | 健康的真实 MoveGroup |
| controller | FJT 接收命令并返回终态 | 碰撞安全或抓取成功 |
| simulation physics | Gazebo 中满足指定接触、抬升、保持合同 | 真机抓取、力闭环或泛化 |
| hardware | 真实机械臂、标定、停止、温度、接触与保持 | 当前仓库尚无此层证据 |

来源：[simulation plan](../simulation-plan.md#1-evidence-vocabulary-and-current-state)、[Ubuntu/Jazzy runbook](../ubuntu-jazzy-runbook.md#evidence-levels) 和 [MoveIt evidence matrix](../observations/2026-08-29-moveit-evidence-matrix.json)。

## 学习路线

| 顺序 | 章节 | 读完能回答的问题 |
| ---: | --- | --- |
| 0 | [阅读与安全环境准备](00-setup.md) | Windows、WSL、venv、仓库根目录与证据层怎样分工？ |
| 审计附录 | [材料审计与写作方案](00-material-audit.md) | 哪些来源是当前事实，哪些段落已过时？初学者可先跳过。 |
| 1 | [纯 Python 确定性核心](01-python-core.md) | 不装 ROS，怎样先把安全合同写清楚？ |
| 2 | [ROS 2 类型化合同](02-ros2-contracts.md) | topic、service、action、时间与 epoch 怎样协作？ |
| 3 | [MoveIt plan-only](03-moveit-plan-only.md) | “规划成功”为什么不等于“已经执行”？ |
| 4 | [PlanningScene 与 ACM](04-planning-scene-acm.md) | 怎样允许指垫接触目标，又不放开掌部碰撞？ |
| 5 | [SO-101、Gazebo 与 controller](05-gazebo-so101.md) | 模型、控制器、接触观察各自证明什么？ |
| 6 | [四阶段抓取](06-four-stage-grasp.md) | approach、descend、close、lift 怎样逐段关联？ |
| 7 | [Candidate002–024 实验日志](07-candidate-lab.md) | 失败候选怎样一步步定位到目标姿态问题？ |
| 8 | [fail-closed 与证据体系](08-fail-closed-evidence.md) | P2 历史负证据与 P12d injected PASS 有何不同？ |
| 9 | [Windows/WSL 与复现坑](09-windows-wsl-repro.md) | PATH、EOL、raw-byte hash、rsync、teardown 怎样排查？ |
| 10 | [证据维护手册](10-evidence-maintenance.md) | 新实验怎样加入博客而不夸大结论？ |
| 11 | [从踩坑到开源贡献](11-open-source-contribution.md) | 怎样判断本地 bug、上游配置缺陷、文档错配和待确认问题？ |

第一次阅读建议从第 0 章开始；那里有 `node`、workspace、commit、artifact、evidence class 等共享术语。材料审计是作者维护用附录，不要求初学者先理解 detached HEAD 或 SHA-256。

## 全书总览图

~~~mermaid
flowchart LR
    A[纯 Python core] --> B[ROS 2 typed contracts]
    B --> C[MoveIt plan-only]
    C --> D[PlanningScene / ACM]
    D --> E[ExecuteTrajectory gate]
    E --> F[arm / gripper FJT]
    F --> G[四阶段协议完成]
    G --> H[独立物理观察器]
    H -->|接触 + 至少 20 mm 抬升 + 0.5 s 保持| I[窄范围 simulation physics]
    H -->|任一条件不满足| J[负证据]
    I -. 仍需标定与真机试验 .-> K[hardware 未验证]
~~~

图源：根据 [README Architecture](../../README.md#architecture)、四个 Action 定义和 [Candidate024 observation](../observations/2026-08-29-candidate024-face-aligned-runtime.json) 整理。

替代文本：系统从纯 Python 核心经 ROS 2、MoveIt、轨迹门和 controller 到四阶段协议；只有独立观察器满足接触、抬升和保持后才形成窄范围仿真物理证据，硬件仍未验证。

## 当前事实速览

- 当前 Windows checkout：332 个测试被收集，330 passed，2 个只因可选固定上游 checkout 不在本工作树而 skipped；早期依赖齐全快照是 332/332。
- P12d 五个 Jazzy 包的选定测试：129/129；focused injected runner：12/12 tests、12 条 JSONL、15/15 facets。
- Candidate024 固定姿态：11 次 runtime attempt 中 10 次满足仿真物理合同，另 1 次在运动前因 stale target 正确 SAFE_STOP；修复后 r03–r11 为 9/9。
- Candidate024 distinct-target：20 个 plan-only 假设结果全部与声明一致；其中 3 个预先选定姿态各执行一次并通过独立观察器。
- 这些结果都不是每姿态重复性、工作空间成功率、完整碰撞保真或真机抓取证据。

精确来源见 [evidence-map.json](evidence-map.json) 与 [材料审计](00-material-audit.md)。

## 图和命令怎样读

- Mermaid 图是对仓库合同与机器记录的重绘，不是实拍图。每幅图都附来源与替代文本。
- 文中的 ROS/Gazebo 命令来自现有 runbook，写作本轮没有执行；只有在隔离 Ubuntu 24.04/Jazzy 环境、全新 artifact 目录和明确证据层下才可复现。
- Windows 静态命令与 Ubuntu runtime 命令不能互相替代。
- 文中不提供历史 OS 进程信号注入 harness 的复现命令；相关 JSON 只作为只读负证据。

## 维护入口

新增实验时先更新 [evidence-map.json](evidence-map.json)，再按照 [证据维护手册](10-evidence-maintenance.md) 的清单改章节。发现可能的上游问题时，先按 [开源贡献章节](11-open-source-contribution.md) 做归属和当前状态审计。所有数字都应回链到 machine-readable observation；只有解释性材料时，应明确标记 documentation-only。
