# 第 7 章：Candidate002–024——把失败写成下一轮实验的输入

## 学习目标

- 学会用单变量、分层 gate 和 machine-readable observation 记录候选；
- 逐项理解 Candidate002–024 的真实证据，不把编号空缺补成故事；
- 看懂接触样本、接触时长、峰值抬升与 retention 的区别；
- 理解 Candidate024 为什么来自几何诊断，而不是继续盲扫参数。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| Candidate | 一组明确冻结的目标、姿态、几何、材料或控制变量假设 |
| control | 与新候选比较时保持不变的基线 |
| single variable | 一轮只改变一个可解释因素，避免结果无法归因 |
| admission | 在 runtime 前用 static/scene/plan-only gate 判断是否准入 |
| negative evidence | 实验没过目标门槛，但能排除假设或暴露系统问题 |
| retention | 物体抬起后持续满足高度/接触窗口，不是只看瞬时峰值 |
| bounded inference | 有数据支持的候选解释，但还没有直接因果证明 |

## 前置知识

- 已读完第 3–6 章；
- 能区分 NO_IK_SOLUTION、FJT failure、sequence complete 和 physics false；
- 接受“配置存在但没有 observation”必须写成“未验证”。

## 系统图/流程图

~~~mermaid
flowchart LR
    H[提出一个候选假设] --> S[static geometry / scene / hash]
    S -->|FAIL| R1[拒绝；零 ROS / 零运动]
    S -->|PASS| P[isolated plan-only]
    P -->|FAIL| R2[记录 IK/scene/planner 层负证据]
    P -->|PASS + zero execution| T{是否预注册一次 runtime?}
    T -->|否| R3[停在 plan-only]
    T -->|是| C[typed controller runtime]
    C --> O[独立 physics observer]
    O -->|未过 20 mm / 0.5 s| N[simulation physics negative]
    O -->|全部满足| V[scoped simulation physics positive]
    V -. 不能自动升级 .-> W[hardware 未验证]
~~~

图源：[simulation plan completed ladder](../simulation-plan.md#6-completed-ladder-and-next-runtime-gates)、runbook D6–D8 与 Candidate observations。

替代文本：候选先过静态和 plan-only，失败就以零运动结束；通过后也只有预注册的 bounded runtime 才进入 controller 和独立物理观察，仿真 positive 仍不能升级为 hardware。

~~~mermaid
flowchart TB
    A[002–004<br/>碰撞与选择性 ACM] --> B[005–008<br/>从接触存在到接触质量/时间]
    B --> C[009–012<br/>可达性、闭合角、有限摩擦]
    C --> D[013–014<br/>对齐与 pad 长度]
    D --> E[015–019<br/>配置中的路径/对齐假设<br/>无独立 runtime observation]
    E --> F[020–022<br/>reachable pre-close、preload、tolerance]
    F --> G[023<br/>只留外部 artifact 引用]
    G --> H[024<br/>按 C022 接触轴重做目标姿态]
~~~

图源：[docs/observations](../observations/)、[候选配置](../../ros_ws/src/edgegrasp_ros/config/) 和 [CHANGELOG](../../CHANGELOG.md) 的候选演进。

替代文本：候选从碰撞、接触、闭合角和摩擦逐步转向接触几何；015–019 只有配置，023 只有来源路径，024 使用 022 的接触轴诊断重做目标姿态。

## 实际操作

### 先写实验卡，再运行

每个候选至少填写：

~~~text
candidate_id:
baseline:
changed_variable:
held_constant:
evidence_class_requested:
static_gate:
plan_only_gate:
runtime_pre_registration:
success_contract:
abort_contract:
artifact_directory: NEW
ROS_DOMAIN_ID: NEW
claim_boundary:
~~~

### 先 plan-only

使用第 3 章的 isolated runner。必须同时满足：

- graph 没有 motion node/client；
- 每段返回轨迹校验并丢弃；
- trajectory publication、ExecuteTrajectory、FJT、execution 全部为零；
- failure layer 保留为 scene、IK、planner 或 validator 的精确原因。

### runtime 只改变已声明变量

以 Candidate012 为例，先分别跑 implicit/control/treatment 的 plan-only；三行都 10/10 且零运动后，才对 mu=1.0 与 mu=1.5 各进行一条明确的 paired runtime。不能同时改 close angle、pad 长度和 preload。

### 保存 machine-readable observation

保存输入 hash、task/command identity、trajectory digest、controller terminal、contact/cube timeline、observer result、cleanup 和 claim_boundary。命令必须指向新 artifact 目录；历史 observation 不可被新跑覆盖。

以上命令与流程来自 runbook，本轮写作没有启动任何 runtime。

## 期望输出

下表是当前 main 可追溯的 Candidate 账本。“无 observation”不是失败，而是明确的证据边界。

| Candidate | 唯一变化/目的 | 最高证据层 | 关键结果 | 当前结论 |
| --- | --- | --- | --- | --- |
| 002 | 初始抓取姿态与 Pilz/OMPL 基线 | plan-only | Pilz 在 57/68 state 处遇 cube–gripper_link；OMPL/Pilz 都拒绝；零执行 | fail-closed 生效，未进入接触 |
| 003 | 改 approach | controller + sim negative | approach plan-only 通过；四阶段/FJT 完成；零 distal-pad contact，峰值 lift 2.43 mm，约 52.3 mm 横移，无 retention | 协议完成，不是抓取 |
| 004 | selective ACM，只允许两 pad child links | static + plan-only | approach→descend 被 forbidden parent collision 拒绝；closed-gripper lift control 通过；零执行 | ACM 合同通过，候选未执行 |
| 005 | routed approach 与 same-sample bilateral contact | plan-only + sim negative | routed 三段各 10/10；fresh proxy runtime 35 个双 pad sample，lift 0.332 mm，无 retention | 接触存在但没有抬升 |
| 006 | 增加 penetration/wrench/effort 诊断 | sim negative | fixed/moving pad 最大深度约 8.55/3.43 µm，法向诊断约 54.42/16.01 N；lift 0.333 mm | 接触不对称；数值未标定 |
| 007 | 增加接触 source-time 关联 | sim negative | 39 个双 pad sample、0.099 s；在 sequence terminal 前 1.747 s 丢失；lift 0.359 mm | 瞬时接触丢失；具体机制未证 |
| 008 | close q：0.60 → 0.50 rad，匹配 moving-pad geometry | sim negative | 98 sample、0.252 s，lift 2.667 mm，最终 XY drift 16.57 mm，无 retention | 指标改善但仍失败 |
| 009 | 静态 tool-X 对称偏移，选择 -0.9 mm | static + plan-only | 静态 overlap 差降至约 0.0166 mm；所有已抽样负 offset（-0.1、-0.2、-0.3、-0.5、-0.7、-0.8、-0.9 mm）均在 descend IK 失败；零偏移 control 10/10、30 段通过 | 静态最优不等于运动学可达；不执行 |
| 010 | close q：0.50 → 0.45 rad | plan-only + sim negative | plan-only 10/10；144 sample、0.368 s，lift 3.323 mm，drift 14.12 mm，无 retention | 继续改善瞬时指标 |
| 011 | close q：0.45 → 0.40 rad | plan-only + sim negative | plan-only 10/10；171 sample、0.463 s，lift 3.711 mm，drift 13.56 mm，无 retention | 停止 close-angle sweep |
| 012 | pad mu=mu2：1.0 → 1.5 | static + plan-only + sim negative | implicit/context、explicit control 与 explicit treatment 三个 plan-only preflight 各 10/10；后两项的 lift 3.392 → 3.468 mm，仅 +0.076 mm，均无 retention | 该有限范围不支持继续盲调摩擦 |
| 013 | tool-frame Y +5 mm / +2 mm | plan-only | 两者均 approach→descend NO_IK_SOLUTION，零运动 | 候选丢弃，无 physics runtime |
| 014 | moving distal-pad local-Y +12 mm | plan-only + sim negative | corrected plan-only 3/3；172 sample、0.462 s，lift 3.680 mm，无 retention | 未显示 pad 长度是已证瓶颈 |
| 015 | grasp-frame +Z 约 30 mm lateral pregrasp | config only | 只有配置和 claim_boundary | 未运行，不能写 plan-only PASS |
| 016 | grasp-frame -Z 约 63 mm reverse pregrasp | config/static only | 60 mm 版本静态 clearance 2.45 mm，低于 5 mm 合同；当前仅配置 | 未见 observation |
| 017 | cube 朝 moving pad 的 grasp-frame X 平移 1 mm | static only | world delta 约 [0.565,-0.826,0] mm；有 scene contract tests | 未见独立 runtime observation |
| 018 | lateral approach 提高 80 mm，分阶段 contact policy | config only | 只有配置与 policy 声明 | 未见 plan-only/runtime |
| 019 | fixed-pad pre-close clearance 约 1 mm | static only | 派生 descend/lift delta 与 scene tests | 未见独立 runtime |
| 020 | reachable pre-close pose | plan-only + controller negative | 三次、每次三段，共 9/9 plan-only；低 preload close FJT 超时，lift 未 dispatch | 低 preload 只触达 moving pad |
| 021 | preload -0.004 → -0.02 N·m | controller + sim negative | fixed/moving contacts 345/739，同 sample 345；close status 6/error -5，lift 未 dispatch | 双侧接触不等于 action 成功 |
| 022 | contact tolerance 0.42 + timing/admission 修复 | controller + sim negative | 首次尝试无 motion；valid retry 四阶段/FJT 完成，lift 12.199 mm，无 20 mm/0.5 s retention，最终 drift 43.061 mm | 接触轴/倾覆解释是有据推断，非直接因果 |
| 023 | pan tradeoff FK scan | provenance pointer only | 只有 Candidate024 geometry 中一个 guest artifact 路径，没有 checked-in config/observation/result | 当前仓库无法独立证明已运行 |
| 024 | cube face 对齐 C022 closing axis，旋转约 37.243°、平移 9.7 mm | plan-only + scoped sim positive | fixed plan-only 10/10、30/30；11 次 runtime attempt 中 10 次 physics success，另 1 次在运动前因 stale 正确安全拒绝；r03–r11 9/9，retained lift 28.858–29.128 mm | 一个固定 proxy scene 的正证据 |

表源：Candidate JSON observations；015–019 来自对应 config 的 changed_variable/claim_boundary；023 仅来自 Candidate024 geometry 的 source_fk_artifact。精确路径汇总见 [材料审计](00-material-audit.md#candidate-编号不是连续运行证据)。

### Candidate024 的 distinct-target 补充

| 实验 | 结果 | 证据边界 |
| --- | --- | --- |
| 旧 rigid translation generator | 20 case 只 matched 10；十个所谓正例全部第一段 IK 失败 | 保留为错误生成假设 |
| corrected shoulder-pan matrix | 20/20 matched；10 reachable、4 invalid scene、6 distant MoveIt rejects；30 accepted segments；零 side effect | distinct-target plan-only |
| 3 selected typed runtime | near-control 与两个 range endpoints 各一次；3/3 observer，12/12 correlated terminal，28.965–29.095 mm retained lift | 每个 pose 一次，不是重复率 |

## 踩坑记录

### 把指标上升写成因果

Candidate008–011 的 contact sample、duration 和 peak lift 逐步上升，但没有实验能单独证明“close angle 导致抓取失败/成功”。这些值只能描述在固定 proxy setup 下的伴随变化。

### 看到 friction treatment 差异小，就说摩擦无关

Candidate012 只测试一个姿态、一个仿真材料映射和 1.0→1.5 的有限范围。正确结论是“这个 bounded sweep 没有显示值得继续扩大”，不是“摩擦永远不是根因”。

### 把 Candidate014 的第一轮 rejection 归因于几何

第一轮实际暴露 ACM harness ordering race。只有修复 base scene confirmation、SetBool success 和 confirmed status 后的 fresh plan-only/runtime 才能评价 +12 mm pad。

### 把 015–019 的 JSON 配置当成已运行

配置包含 changed_variable 和 claim_boundary，说明设计认真；但没有 checked-in observation 就不能写 plan-only、controller 或 physics 结果。

### 从 Candidate023 的目录字符串猜结果

一个 source_fk_artifact 路径只说明 Candidate024 的生成记录引用了外部 guest artifact。缺少内容、hash 和 observation，正确写法是“暂无仓库内可验证数据”。

### 用错误 MoveGroup 的 contact histogram 支撑规划

Candidate005 早期 histogram 误启动固定上游 MoveGroup。Gazebo contact counts 可保留为物理历史，但不能归到 EdgeGrasp proxy planning evidence。

### 把共同发生的 crash 当成物理失败原因

Candidate003 记录同时含 MoveGroup/rclpy teardown 异常和 physics failure，没有直接关联证明前者造成后者。必须分别记录。

## 排查过程

### 从 Candidate022 到 Candidate024 的证据链

Candidate022 valid retry 已完成四阶段，但：

- peak lift 12.199 mm，小于 20 mm；
- final XY drift 43.061 mm；
- cube 最终接近 90° roll；
- bilateral contact 点 world-Y 相距约 50.022 mm；
- fixed/moving contact normal 没有形成清晰共同对向作用线。

这支持“存在 torque/tipping mechanism”的 bounded inference，但不是直接因果证明。下一步没有继续加摩擦或 preload，而是测量 closing-axis 相对 cube face 的约 37.243° 偏差。

~~~mermaid
flowchart LR
    C22[C022<br/>双 pad + 12.199 mm transient lift<br/>43.061 mm drift] --> M[测量接触点/normal/closing axis]
    M --> A[发现 closing axis 与最近 cube face normal<br/>相差约 37.243°]
    A --> C24[旋转 cube face 对齐该轴<br/>再平移 9.7 mm 恢复约 1 mm gap]
    C24 --> P[isolated plan-only 10/10<br/>30/30，零执行]
    P --> R[r01 success]
    R --> S[r02 at planning boundary 208 ms stale<br/>正确零运动 SAFE_STOP]
    S --> F[100 ms pre-send recheck<br/>correlated cancel + generation filter]
    F --> Z[r03–r11 9/9 scoped sim physics]
~~~

图源：[Candidate020–022 observation](../observations/2026-08-29-candidate020-022-preclose-control-runtime.json) 与 [Candidate024 fixed observation](../observations/2026-08-29-candidate024-face-aligned-runtime.json)。

替代文本：Candidate022 的低抬升和大漂移促使测量接触轴；Candidate024 对齐 cube face 并恢复预闭合间隙，先通过零执行 plan-only；r02 暴露 stale scheduling，修复后九次通过。

### shoulder_pan matrix 为什么修正

~~~mermaid
flowchart TB
    W[错误：以 base 原点做刚体 XYZ 平移] --> X[10 个正例均 first-stage NO_IK_SOLUTION]
    U[读取 pinned URDF] --> O[shoulder_pan origin<br/>约 0.0388353, -8.98e-9, 0.0624 m]
    U --> Z[local +Z 映射到 base -Z]
    O --> T[绕 exact joint origin/axis<br/>同时变换 cube 与全部 stage poses/orientations]
    Z --> T
    T --> Y[10/10 distinct reachable plan-only]
~~~

图源：[translation diagnostic](../observations/2026-08-29-candidate024-translation-matrix-plan-only.json) 与 [corrected matrix](../observations/2026-08-29-candidate024-target-matrix-plan-only.json)。

替代文本：错误 generator 围绕 base 原点平移导致所有正例 IK 失败；读取固定 URDF 后，围绕 shoulder_pan 精确原点和轴共同变换场景与姿态，十个 distinct target 才通过 plan-only。

## 最终证据

- Candidate002–014、020–022、024 各自证据层的关键实验事实有 checked-in machine-readable observation。
- Candidate015–019 只有 design/config；Candidate023 只有来源路径，均未被升级。
- Candidate024 fixed control 的 11 次 attempt 包含 10 次 scoped simulation-physics success 与 1 次 pre-motion stale safety rejection；post-fix r03–r11 为 9/9。
- distinct-target matrix 与三个单次 runtime 分开记录，不计算伪造的 workspace success rate。
- 负结果没有被覆盖：旧 translation generator、Candidate005 错误 MoveGroup histogram、Candidate014 harness race、r02 stale 都保留。

证据层：从 static 到 scoped simulation physics 的混合账本；每行单独标记。

## 仍不能宣称的边界

- 不能把 10/11 或 9/9 称为硬件成功率、物理确定性或全局泛化。
- 不能说 015–019、023 已通过 plan-only 或 runtime。
- 不能把 transient contact/lift 称为 grasp。
- 不能把有限 friction/pad/angle 实验推广到所有材料和几何。
- 不能把 C022 的 tipping inference 写成已证实因果。
- 不能从三个 pose 各一次计算工作空间成功概率。

## 小练习

1. 选择 Candidate008–012 中任意一对，列出改变变量与保持变量。
2. 为什么 Candidate009 的静态对称性最优不能直接执行？
3. 给 Candidate015 写一份“准入前仍缺哪些证据”的清单。
4. 把 Candidate022 写成三句互不混淆的结论：controller、contact、physics。
5. Candidate024 r02 是实验失败还是安全功能正确工作？说明两种视角。
