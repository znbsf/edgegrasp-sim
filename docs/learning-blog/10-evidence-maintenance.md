# 第 10 章：证据维护手册——让博客能跟着实验安全更新

## 学习目标

- 建立“源码 → 原始 artifact → machine observation → claim → 博客”的追踪链；
- 为新实验选择正确 evidence class；
- 在更新数字前发现文档漂移和 claim 越界；
- 用 Windows 静态检查验证博客和 JSON，不启动 ROS runtime。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| provenance | 一条结论来自哪个 commit、命令、artifact 和 hash |
| source of truth | 冲突时优先核对的机器记录或当前 source contract |
| claim boundary | 这条证据能说到哪里、不能说到哪里 |
| superseded | 被更新 artifact 取代，但历史没有删除 |
| negative artifact | 失败、拒绝或 crash 的原始记录 |
| drift | README、report、runbook 或博客复制的数字随时间不一致 |
| evidence map | 本书的 claim ID、证据层、来源和边界索引 |

## 前置知识

- 完成前九章；
- 能解析 JSON、看 Git diff；
- 愿意保留失败，不用新成功覆盖旧 artifact。

## 系统图/流程图

~~~mermaid
flowchart LR
    C[git commit / source hashes] --> R[raw logs + JSONL + JUnit + MCAP]
    R --> O[machine-readable observation]
    O --> E[evidence-map claim ID]
    E --> B[博客数字与图表]
    B --> Q[static QA / link / boundary review]
    Q -->|发现冲突| O
    Q -->|通过| P[可维护发布稿]
~~~

图源：[P12d observation](../observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json)、[validation report](../validation-report.md) 的 observation log template 和本书 [evidence-map.json](evidence-map.json)。

替代文本：commit 和原始日志先形成 machine observation，再进入 evidence map 和博客；静态 QA 发现冲突时回到 observation，不直接在文章里改数字。

## 实际操作

### 1. 新建而不是覆盖 artifact

~~~text
command:
started_at (timezone):
ended_at (timezone):
exit_code:
source commit:
source file SHA-256:
artifact directory:
stdout/stderr:
expected:
observed:
status: PASS | FAIL | PARTIAL | UNVERIFIED
evidence_class:
claim_boundary:
cleanup:
~~~

如果目录已存在就停止。历史 failure/partial 仍保留，新的 fresh rerun 使用新 ID。

### 2. 先更新 machine observation

观察文件应至少包含：

- schema_version 与 observation_id；
- environment 和上游 pin；
- 输入配置/hash；
- exact command/artifact；
- request/task/generation/UUID 或 stage identities；
- 结果计数与 validator；
- evidence_status；
- 布尔 claim boundary；
- cleanup/clock 结果。

缺少直接数据时用 null、false 或 UNVERIFIED，不要推算。

### 3. 更新 evidence-map

在 evidence-map.json 添加/修改一个 claim ID，写清：

- class；
- value；
- sources；
- boundary。

只有 map 先完成，章节才更新。documentation-only 的 robot AI review 可以进入延伸阅读，不能成为 runtime claim source。

### 4. Windows 静态 QA

以下命令不会启动 ROS/MoveGroup/Gazebo：

~~~powershell
powershell -NoProfile -File scripts\check.ps1 -ReplayRuns 100
ruff check .
Get-ChildItem -Recurse -Filter *.json | ForEach-Object {
  Get-Content -LiteralPath $_.FullName -Raw -ErrorAction Stop |
    ConvertFrom-Json -Depth 100 -ErrorAction Stop | Out-Null
}
git diff --check
~~~

Bash 脚本只做语法检查：

~~~bash
bash -n scripts/run_injected_moveit_fail_closed.sh
~~~

这不是 Ubuntu runtime evidence。PowerShell 文件用 Parser.ParseFile 做语法检查，不执行脚本。

### 5. 做 claim review

对每个新增句子问：

1. 是 core/static、plan-only、injected、controller、simulation physics 还是 hardware？
2. source 是当前还是 superseded？
3. 数字能否回链到 JSON 字段？
4. 是否把 cancel ack 当 terminal？
5. 是否把 sequence complete 当 grasp？
6. 是否把仿真/论文升级成硬件？

## 期望输出

当前基线 QA 应看到：

~~~text
evidence-map.json parses
all learning-blog Markdown files exist
每章 11 个固定栏目齐全
Mermaid 图有“图源”和“替代文本”
git diff --check: no whitespace errors
scripts/check.ps1: 330 passed + 2 optional skips in this worktree
three 100-run core replays deterministic
no ROS/MoveGroup/Gazebo process started by documentation QA
~~~

若具备可选固定上游 checkout，另一个历史/完整布局可以是 332/332；报告必须写依赖条件，不能偷偷替换当前结果。

## 踩坑记录

### 在 README、report、runbook、博客复制同一数字

复制越多越容易 drift。本仓库已经出现 runbook 的 113-test 旧段与 P12d 129/129 并存，也出现 validation report 旧 limitation 未吸收 distinct-target 新结果。博客因此用 evidence-map 作为更新入口，并在 audit 中公开冲突。

### 新成功覆盖旧失败

Candidate024 的旧 translation generator、r02 stale、P2 crash 都是重要证据。删除它们会让读者无法理解修复是否针对真实问题。

### 只保存 summary，不保存身份 ledger

一个 PASS 如果没有 request/generation/UUID、stage/digest、source hash 和 artifact 路径，很难判断是否串了旧 callback 或别的 graph。

### 把 source presence 当 runtime

launch file、package prefix、config 或模型可加载只属于 static/inventory。Candidate015–019 正是必须保持 config-only 的例子。

### 对缺失字段做“合理推算”

Candidate023 没有 checked-in observation；正确值不是猜一个结果，而是 UNVERIFIED。P2 fresh graph 晚状态也不能推成 strong correlation。

### 文档任务顺手跑 runtime

写博客不需要启动 MoveGroup/Gazebo。只有明确的新验证任务、隔离环境、允许的 runner 与全新 artifact 才能增加 runtime 证据。

## 排查过程

### 文档冲突决策树

~~~mermaid
flowchart TD
    A[两个文档数字冲突] --> B{存在 machine-readable observation?}
    B -->|是| C[核对 commit、时间、artifact、字段]
    B -->|否| D[标记 UNVERIFIED；不选更好看的数字]
    C --> E{同一 source snapshot?}
    E -->|否| F[分别保留 historical/current]
    E -->|是| G{定义/证据层相同?}
    G -->|否| H[拆成两个指标或 evidence class]
    G -->|是| I[修正文档漂移并记录 superseded]
~~~

图源：[材料审计的来源优先级](00-material-audit.md#来源优先级) 与 validation report 的“Earlier success count is never reused”规则。

替代文本：文档数字冲突时先找 machine observation，再比较 commit/时间和指标定义；不同 snapshot 分开保留，不同证据层拆开，缺数据标 UNVERIFIED。

### 证据层审查表

| 句子 | 最低所需证据 | 常见误写 |
| --- | --- | --- |
| “Python replay 可重复” | core tests + replay digest | “仿真确定” |
| “MoveIt 可规划” | real plan-only response + validator | “机械臂执行成功” |
| “取消合同通过” | exact fake/real class + terminal ledger | “机器人已停” |
| “controller 完成命令” | correlated FJT terminal | “抓住物体” |
| “仿真物理抓取” | contact + lift + retention observer | “真机抓取” |
| “硬件抓取” | 真实硬件、标定、停止与保持 artifact | 当前仓库不具备 |

## 最终证据

- 本书已有独立 audit、入口、十章正文和 machine-readable evidence-map。
- 每章沿用相同栏目与 claim boundary。
- 关键数字回链到 observations，而不是 robot AI 综述。
- 当前仓库的 AGENTS.md 明确禁止重跑历史 real process interruption，并限定 Windows 安全验证。
- observation log template 与现有 P12d/Candidate024 artifacts 展示了可维护证据链。

证据层：documentation + static traceability；本章不产生 runtime 证据。

## 仍不能宣称的边界

- 文档完整不等于系统已通过新 runtime。
- hash 一致不等于物理行为一致。
- 测试全绿不等于没有未覆盖分支。
- evidence map 是索引，不替代原始 artifact。
- robot_ai_frontier_review_2026.md 只能作为 documentation-only 背景。
- 当前硬件层仍是 false/未运行。

## 小练习

1. 为一个新的 Candidate025 写实验卡，但不要填写未运行结果。
2. 把“controller status 4，所以抓取成功”改写成两条证据边界正确的句子。
3. 如果 README 写 129/129、旧 runbook 写 113/113，你会核对哪些字段？
4. 设计一个 evidence-map claim，来源是 plan-only JSON，boundary 应包含什么？
5. 找一个本书中的 Mermaid 图，检查其图源和替代文本是否足以独立理解。
