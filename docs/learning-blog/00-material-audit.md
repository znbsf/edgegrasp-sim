# 材料审计、读者画像与写作方案

本文件是全书的作者审计记录，不计入第 0–11 章的固定学习栏目检查。零基础读者可以先读 [第 0 章](00-setup.md)，再在需要核对来源时回到这里。

## 审计结论

只读审计在任何博客文件写入前完成。审计时工作树无本地变更，处于 detached HEAD，但 HEAD、main 都指向：

~~~text
211bbced68c3b2f4819822388ec888fb20821f9d
fix(moveit): harden fail-closed timeout correlation
~~~

这意味着文章基线是当前本地 main 的精确内容，而不是旧工作区、旧分支或记忆中的中间状态。

## 已核对的材料

| 材料 | 用途 | 使用规则 |
| --- | --- | --- |
| [AGENTS.md](../../AGENTS.md) | 自动化边界、禁止重现实验、证据分类 | 最高优先级；历史 real MoveIt JSON 只读 |
| [README.md](../../README.md) | 当前总览、架构、证据入口 | 优先采用 2026-08-30 当前表格 |
| [CHANGELOG.md](../../CHANGELOG.md) | 真实演进、失败与修复顺序 | 不把历史计数当成当前计数 |
| [simulation-plan.md](../simulation-plan.md) | 证据词汇、系统合同、候选主线 | 用来解释“为什么这样设计” |
| [validation-report.md](../validation-report.md) | 命令、时间、计数、回归与限制 | 同一文件冲突时优先当前 final/后加入的机器记录 |
| [ubuntu-jazzy-runbook.md](../ubuntu-jazzy-runbook.md) | 可复现操作和期望输出 | 命令标记证据层；本轮不执行 runtime |
| [environment-audit.md](../environment-audit.md) | Windows/WSL/版本边界 | 保留审计日期，不冒充当前实时盘点 |
| [upstream-manifest.json](../upstream-manifest.json) | 上游 SHA、许可证、用途 | 版本与用途的 machine-readable 来源 |
| [observations/](../observations/) | 候选、规划、仿真与 fail-closed 事实 | 数字和布尔 claim 的首要来源 |
| [robot_ai_frontier_review_2026.md](../robot_ai_frontier_review_2026.md) | 技术谱系和后续阅读 | documentation-only，不能证明运行时或硬件 |

机器可读 observations 在审计时均可被 JSON 解析。当前两个 SDF 的实际 SHA-256 也与 2026-08-30 source contract 相符：

- table_cube.sdf：8a4b436cd4863a0801602d15bfafebecaadfbe104d4b7db93e4a5f33415bb4c4
- table_cube_candidate024_face_aligned.sdf：058e3237b430c56dbcfcde1091573bf8ce6827e6322114c094011a16095af0ba

## 来源优先级

同一个结论出现多个版本时，按以下顺序判断：

1. 当前提交中的 machine-readable observation 与直接 source contract。
2. validation report 的 current final snapshot 或明确晚于旧段落的新增章节。
3. README 当前证据表与 simulation plan 当前快照。
4. runbook 中的历史复现段落、CHANGELOG 的历史过程。
5. documentation-only 综述和解释性文字。

这个顺序不代表 JSON 永远正确；它意味着 JSON 提供了可校验字段、hash、计数和 claim boundary，发现冲突时能指出具体字段，而不是凭印象选数字。

## 审计发现的版本漂移

### Windows 与 Jazzy 测试数

- 早期完整固定上游布局：Windows 332/332。
- 当前 P12d 本地 checkout：收集 332，330 passed，2 skipped；skip 原因只是可选固定上游 checkout 缺席。
- 当前 P12d 五包选定 Jazzy 测试：129/129。
- runbook 中仍有 113-test 的旧段落；博客只把它作为历史快照，不作为当前总数。

来源：[validation report 当前 Windows 段](../validation-report.md#windows-core-structure-replay-and-syntax) 与 [P12d package 段](../validation-report.md#ubuntu-2404--ros-2-jazzy-packages)。

### distinct-target 限制段落

validation report 的旧 remaining-limitations 段仍说没有十个 distinct target 研究；但同一文件已经新增：

- 20-case plan-only matrix：10 个可达假设、4 个场景合同拒绝、6 个 MoveIt 远目标拒绝，全部 matched，运动副作用为零；
- 3 个预选代表姿态各一次 typed runtime，通过 12/12 correlated terminals 和 3/3 独立观察器。

因此当前准确边界是：已有规划覆盖和三个单次代表执行；仍没有每姿态重复性、工作空间成功率或全碰撞保真。

来源：[target matrix](../observations/2026-08-29-candidate024-target-matrix-plan-only.json) 与 [distinct-target typed runtime](../observations/2026-08-29-candidate024-distinct-target-typed-runtime.json)。

### Candidate 编号不是连续运行证据

| 编号 | 仓库内材料状态 | 写作处理 |
| --- | --- | --- |
| 002–014 | 有对应 observation，013 记录在 014 observation 内 | 可写计划/运行事实，但逐项区分 static、plan-only、runtime |
| 015–019 | 只有候选配置与 claim_boundary，没有 checked-in runtime observation | 写“设计候选”，不写成已运行 |
| 020–022 | 共用一份 machine-readable progression observation | 可写 plan-only、controller/contact 与负物理证据 |
| 023 | 只有 Candidate024 profile 指向 guest artifact 的来源字段 | 写“仓库内暂无可独立复核结果” |
| 024 | 固定姿态、translation diagnostic、target matrix、3-pose runtime 都有 observation | 分别写，不合并成一个成功率 |

这也是为什么第 7 章使用“实验日志”而不是“24 次成功实验”。

## 读者画像

主要读者是零 ROS 基础的软件学习者：

- 已知：文件、命令行、函数和测试的基本概念。
- 未假设：机器人学、四元数、tf2、ROS graph、MoveIt、Gazebo、Action 生命周期。
- 学习目标：能读懂系统图，能分辨证据层，能从一次失败定位到下一次单变量实验。
- 非目标：仅凭文章控制真实机械臂，或把仿真数字映射成真机成功率。

## 全书目录

0. 阅读准备：Windows/WSL 分工、venv、仓库根目录与安全边界。
1. Python core：数据、预测、状态机、安全门和 deterministic replay。
2. ROS 2：typed target、时钟、epoch、topic/service/action。
3. MoveIt plan-only：IK、规划、轨迹验证、零执行计数。
4. PlanningScene/ACM：共享场景、碰撞白名单与选择性接触。
5. SO-101/Gazebo：上游模型、controller、FJT、contact observer。
6. 四阶段抓取：逐段关联和独立物理判定。
7. Candidate002–024：单变量实验、负结果、空缺和 Candidate024。
8. fail-closed：Action future 生命周期、P2 负证据和 P12d injected 合同。
9. Windows/WSL：PATH、monorepo packaging、EOL/hash、rsync、rclpy teardown。
10. 维护：证据模板、claim review 和新实验接入。
11. 开源贡献：本地 bug、环境问题、上游配置缺陷、文档错配与 Issue/PR 边界。

## 代表性图表清单

下表只列关键概念图，不是对每个 Mermaid block 的逐项枚举。

| 图表 | 形式 | 主要来源 |
| --- | --- | --- |
| 六层证据阶梯 | Mermaid flowchart | simulation plan、MoveIt matrix |
| Python 核心状态机 | Mermaid stateDiagram | src/edgegrasp/fsm.py、controller.py |
| ROS motion path | Mermaid flowchart | README Architecture、runbook D |
| Action 生命周期 | Mermaid sequenceDiagram | Action 定义、adapter 状态事件、P12d |
| 坐标/时钟合同 | Mermaid flowchart | Target3D、TrackedTarget、simulation plan |
| PlanningScene/ACM | Mermaid graph | scene loader、Candidate004 observation |
| 四阶段抓取 | Mermaid stateDiagram | grasp_sequence.py、GraspSequence.action |
| 物理成功判定 | Mermaid flowchart | GraspPhysicsEvidence.action、Candidate024 |
| Candidate 对比 | 表格 | Candidate observations/config |
| shoulder_pan 目标生成 | Mermaid schematic | Candidate024 target matrix observation |
| P2/P12d 分流 | Mermaid flowchart | real negative JSON、P12d JSON |
| Windows→WSL 复现 | Mermaid flowchart | environment audit、runbook |
| 问题归属与上游贡献 | Mermaid flowchart | upstream manifest、固定源码、Issue/PR 状态 |

所有图都用仓库事实重绘，附来源说明和替代文本；没有把论文插图或第三方截图当作项目运行证据。

## 写作与文件方案

目录固定为 docs/learning-blog：

~~~text
README.md                 学习入口与证据图例
00-setup.md               零基础阅读与安全环境准备
00-material-audit.md      本次只读审计、冲突和内容方案
01-... 到 11-...          按学习顺序拆分的章节
evidence-map.json         claim ID、证据层、来源路径和边界
~~~

维护规则：

- 一章一个主题，避免把 runtime 计数复制到多个无来源段落。
- 每章固定保留“学习目标、术语、前置知识、系统图、实际操作、期望输出、踩坑、排查、最终证据、仍不能宣称、小练习”。
- 命令复制自现有 README/runbook，并标注本轮没有执行。
- 数字尽量由 evidence-map 指向 observation；若文档冲突，记录冲突而不是静默选择。
- robot AI frontier review 只放在“延伸阅读”，不能出现在运行证据链中。

## 本轮执行边界

本轮仅进行了文件读取、Git 状态/commit 核对、JSON 解析、raw-byte SHA-256 核对与后续 Windows 静态验证。没有启动 ROS、真实 MoveGroup、Gazebo、远程主机或硬件；没有运行 injected harness；没有创建或重建历史 OS-signal harness。

## 成稿后的安全校验

博客完成后，在同一个 checkout 只执行了 AGENTS.md 允许的静态/core 检查：

- scripts/check.ps1 -ReplayRuns 100：当前系统 Python 3.14.5 收集 332 项，330 passed、2 skipped；project structure PASS；0/20/40 mm/s 三个场景各 100 次 replay 均 deterministic。
- ruff check .：通过。
- 68 个仓库 JSON（含 evidence-map.json）逐个解析：通过。
- 12 个 shell 脚本执行 bash -n、4 个 PowerShell 脚本执行 Parser.ParseFile：语法通过；这不是 Ubuntu/ROS runtime 证据。
- 第 0–11 章固定栏目齐全；27 幅 Mermaid 均同时有图源和替代文本；相对链接和 evidence-map 源路径均存在。

这次 fresh rerun 只刷新 core/static 与文稿结构信心，没有刷新 Jazzy package、MoveGroup、controller、simulation physics 或 hardware 证据。Ruff 使用当前 PATH 上的独立可执行文件；python -m ruff 在当前系统 Python 中不可用，所以两种入口不能混为同一环境事实。
