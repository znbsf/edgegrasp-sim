# 第 11 章：从踩坑到开源贡献——先证明问题属于谁

## 学习目标

- 把“运行失败”拆成项目本地 bug、环境问题、上游配置缺陷、文档错配和待确认的上游问题；
- 学会用固定 commit、源码、官方文档和最小回归证明一个结论；
- 知道什么适合先做 docs-only/static patch，什么必须补 runtime 才能报告；
- 区分“本地已经 PR-ready”和“已经获得向上游提交 Issue/PR 的授权”。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| upstream | EdgeGrasp 依赖但不由本仓库维护的开源项目 |
| deterministic bug | 输入、版本和步骤固定后，能够稳定得到同一错误，并能写回归测试的问题 |
| source-confirmed mismatch | 不启动程序，仅比较同一 commit 的源码/配置就能证明两处合同互相矛盾 |
| runtime bug | 必须真正运行后才能证明的行为错误；仅看源码通常还不够 |
| docs-only patch | 只修说明、示例或注释，不改变程序行为的补丁 |
| minimal reproduction | 去掉 EdgeGrasp 后仍能触发问题的最小上游示例 |
| PR-ready | 本地补丁、测试和说明已准备好；不等于已经发布或被维护者接受 |
| dated status | Issue、PR、分支和权限会变化，所以状态必须附核对日期 |

## 前置知识

- 已读第 5 章，知道 EdgeGrasp 固定了哪些 SO-101 上游版本；
- 会查看 Git commit、README、YAML 和 XML；
- 接受“线索很强”仍可能只能写成待确认，而不是确定 bug。

## 系统图/流程图

~~~mermaid
flowchart TD
    A[发现 warning / crash / 文档不通] --> B{去掉 EdgeGrasp 后仍存在?}
    B -->|否| C[EdgeGrasp 本地问题<br/>先修本仓库并加回归]
    B -->|未知| D[证据不足<br/>缩小步骤并保留 UNVERIFIED]
    B -->|是| E{同一 upstream commit<br/>仅看源码即可证明矛盾?}
    E -->|是| F[上游配置或文档错配<br/>优先 static/docs patch]
    E -->|否| G{有独立、可重复 runtime?}
    G -->|否| H[疑似上游问题<br/>不声称 deterministic]
    G -->|是| I[上游 runtime bug candidate<br/>准备最小复现和版本矩阵]
    F --> J[核对 CONTRIBUTING / Issue / PR 当前状态]
    I --> J
    J --> K[先在本地完成 patch + tests]
    K --> L{用户明确授权公开提交?}
    L -->|否| M[仅保存 PR-ready 材料]
    L -->|是| N[创建 Issue/PR 并接受维护者反馈]
~~~

图源：根据本仓库的 [upstream manifest](../upstream-manifest.json)、[validation report](../validation-report.md) 和 2026-08-31 对直接上游源码/Issue/PR 的只读核对整理。

替代文本：发现问题后先判断是否能脱离 EdgeGrasp 重现，再区分源码可证的配置/文档错配与需要 runtime 的行为 bug；完成本地补丁和测试后，只有得到明确授权才公开提交。

## 实际操作

### 1. 我们先做了一张“问题归属表”

实践中最危险的不是程序报错，而是太快给错误贴上“上游 bug”标签。下面是当前能站得住脚的分类：

| 现象 | 当前分类 | 为什么 | 适合的贡献形式 |
| --- | --- | --- | --- |
| PlanningScene 周期 revalidation 短暂发出 false | EdgeGrasp 本地确定性 bug，已修 | 本地状态机把 in-flight query 当成未确认 | 本仓库回归测试和说明 |
| rclpy teardown 出现 InvalidHandle/Destroyable | EdgeGrasp harness/application teardown 顺序问题，已修 | task 尚未退出就销毁 ActionServer/Client | 本地 lifecycle regression；若报上游需另写 rclpy-only 最小复现 |
| cancel 已确认但 terminal 未确认 | EdgeGrasp 安全合同，不是已证实上游 bug | cancel acknowledgement 本来就不能替代 exact result terminal | 本地 fake regression 与合同文档 |
| SDF SHA-256 因 CRLF/LF 漂移 | Windows/Git 环境问题 | 内容相同，raw bytes 不同；仓库已固定 `*.sdf eol=lf` | `.gitattributes` 与排查文档，不开上游代码 Issue |
| Python 命令命中错误解释器 | 操作/环境问题 | PATH 与 venv 选择错误，不是依赖库缺陷 | 明确 `py -3.10`/venv 的使用说明 |
| 历史 MoveIt `PlanExecution::stop()` 附近 SIGSEGV | 疑似上游 runtime 问题，证据不足 | 只有冻结的历史负 artifact；没有当前独立最小复现、core/backtrace 或跨版本对照 | 暂不创建确定性 bug；保留未来复现计划 |

这张表的核心规则是：**本地修复可以确定，不代表根因已经被证明在上游。**

### 2. 候选 A：`use_camera=false` 的 URDF/SRDF 源码错配

截至 2026-08-31（Asia/Shanghai），只读核对的 `adoodevv/so101_ros2` main 仍指向 EdgeGrasp 固定的 `0305e03ab54e64aae9263fcbf339622e654012f3`。这个 commit 中：

1. [SO-101 robot xacro](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_description/urdf/robots/so101.urdf.xacro) 声明 `use_camera`；
2. [D435 sensor xacro](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_description/urdf/sensors/intel_rgbd_cam_d435.urdf.xacro) 只在 `use_camera=true` 且 `use_gazebo=true` 时创建 `torso_link` 和 `camera_head_link`；
3. [固定 SRDF](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/config/so101/so101.srdf) 却无条件声明这些 camera-only link 的 collision pairs；
4. [move_group launch](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/launch/move_group.launch.py) 只把 mapping 传给 URDF，semantic description 仍加载固定 `.srdf`；
5. [srdfdom](https://raw.githubusercontent.com/moveit/srdfdom/ros2/src/model.cpp) 对 URDF 中不存在的 link 发 warning 并跳过 pair，这是 parser 的预期防御行为。

因此可以静态确定的是“机器人包的 URDF/SRDF 配置在 `use_camera=false` 时不一致”，不是“MoveIt parser 有 bug”。

最小贡献方案：

- 把 SRDF 改为可接收 `use_camera` mapping 的 xacro；
- 只在 camera 启用时生成 camera-only collision pairs；
- 加一个静态测试：分别展开 camera true/false，确认每个 SRDF pair 的 link 都存在于对应 URDF；
- 再在 fresh Ubuntu/Jazzy 中启动两种模式，确认 warning 消失且 camera=true 没有回归。

前 3 项可以做成 PR-ready 静态补丁；第 4 项尚未在本轮执行，所以不能写成“runtime 已修复”。

### 3. 候选 B：gripper controller 的文档与 YAML 合同冲突

同一固定 commit 的 [controller YAML](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/config/so101/ros2_controllers.yaml) 把 `gripper_controller` 配成 `joint_trajectory_controller/JointTrajectoryController`；但 [package README](https://raw.githubusercontent.com/adoodevv/so101_ros2/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/README.md) 把它写成 `ForwardCommandController`，并给出 `Float64MultiArray` 的 `/commands` 示例。

官方 Jazzy 文档说明：[JointTrajectoryController](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html) 使用 `trajectory_msgs/msg/JointTrajectory` topic 或 `FollowJointTrajectory` Action；[`forward_command_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/forward_command_controller/doc/userdoc.html) 的 `~/commands` 才使用 `std_msgs/msg/Float64MultiArray`。

这里能确定的是“README 与 YAML/interface 不一致”，还不能仅凭源码断言当前 Gazebo launch 一定以某种方式失败。安全的第一笔贡献应是 docs-only：

- 若维持 JTC，README 示例改成 `JointTrajectory` 或 `FollowJointTrajectory`；
- 删除 package README 中已经过时的“MoveIt 尚未配置”段落；
- 用静态测试解析 YAML，检查 README 声明的 controller 类型与示例消息是否一致。

如果上游真正想保留 `Float64MultiArray`，就必须同时改 controller 类型、参数、MoveIt controller mapping 和 runtime 测试；这已经不是小型文档修复。

### 4. 候选 C：开放 Issue 不等于仍未处理

[TheRobotStudio/SO-ARM100 Issue #54](https://github.com/TheRobotStudio/SO-ARM100/issues/54) 截至 2026-08-31 仍为 Open、无 assignee、无 linked branch/PR。它最初报告 joint limits、joint axes 和 decomposed collision meshes 等 URDF 问题。

但状态栏不是全部事实：[PR #106](https://github.com/TheRobotStudio/SO-ARM100/pull/106) 和 [PR #117](https://github.com/TheRobotStudio/SO-ARM100/pull/117) 已合并，修过轴向、frame、link suffix 和 origin 等一部分范围。当前 main 也已经有 revolute joint limits；[SO-101 simulation README](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md) 把部分 base collision mesh 的缺失记录为仿真/规划稳定性取舍。

所以 #54 目前更准确的分类是“历史开放 enhancement/模型质量议题”。剩余 collision decomposition 缺少绑定当前版本的可重复失败、CAD ground truth 和维护者认可的 fidelity target，不能因为 Issue 仍 Open 就宣称“确定 bug 没人处理”。

### 5. 上游互动前的最小清单

~~~text
[ ] 核对当前默认分支 commit，而不是只看 EdgeGrasp 的旧 pin
[ ] 搜索 Issue、PR、linked branch、评论和最近提交
[ ] 读取 LICENSE、CONTRIBUTING、模板与 issue creation 权限
[ ] 写清 minimal reproduction 与 expected/actual
[ ] 分开 source-confirmed、runtime-confirmed 和尚未验证
[ ] 最小 patch 不混入 EdgeGrasp 私有设计
[ ] 本地静态/单元测试通过
[ ] 获得用户对公开 Issue/PR/comment 的明确授权
~~~

截至 2026-08-31（Asia/Shanghai），`adoodevv/so101_ros2` 的 GitHub 页面显示 0 个 Issue、0 个 PR，并限制创建 Issue。因此即使候选 A/B 已经具备源码证据，也应先准备小而清楚的 patch/说明，再通过维护者允许的渠道沟通；本轮没有创建 Issue、PR 或评论。

## 期望输出

完成一次合格的上游审计后，输出应像下面这样，而不是一句“发现 bug”：

~~~text
upstream: adoodevv/so101_ros2
audited_ref: 0305e03ab54e64aae9263fcbf339622e654012f3
checked_at: 2026-08-31
classification: source-confirmed configuration mismatch
reproduction: compare camera-disabled URDF link set with fixed SRDF pairs
runtime_status: not run in this documentation task
minimal_patch: conditional SRDF xacro + static true/false test
public_issue_or_pr: none created
claim_boundary: not a MoveIt core bug; runtime fix not yet verified
~~~

这个格式把“已经知道什么”“还缺什么”和“是否做了公开动作”分开了。

## 踩坑记录

### warning 来自 MoveIt，就把责任归给 MoveIt

日志组件只说明谁发现问题，不说明谁制造问题。srdfdom 在这里正确地拒绝未知 link；矛盾来自机器人包固定 SRDF 与条件 URDF 的组合。

### README 能跑通一次，就认为它永远是 source of truth

README 很容易落后于 YAML 和 launch。相反，也不能盲信 YAML：应先判断维护者想保留哪一种 controller interface，再修文档或行为。

### Issue Open、无人认领，就直接开始大改

一个开放 Issue 可能已被别的 PR 部分解决，也可能缺少维护者认可的目标。先阅读历史和当前源码，通常比写代码更省时间。

### 本地修复成功，就写成上游确定性 bug

PlanningScene false pulse、teardown ordering 和 late callback generation 都是值得记录的本地工程成果；它们没有独立上游最小复现时，就不应包装成 MoveIt/rclpy 缺陷。

### 为了“贡献记录”过早公开评论

公开 Issue/PR 会占用维护者时间。应先做到复现短、范围小、测试清楚，再取得用户授权；PR-ready 与已发布是两种状态。

## 排查过程

| 你手里的证据 | 可以写什么 | 还不能写什么 | 下一步 |
| --- | --- | --- | --- |
| 同 commit 两份配置互相矛盾 | source-confirmed mismatch | runtime 一定失败 | static validator，再做 isolated runtime |
| 本地 EdgeGrasp regression 能稳定重现 | local deterministic bug | upstream deterministic bug | 去掉 EdgeGrasp 写最小复现 |
| 一份历史 crash artifact | observed historical negative | 当前版本仍必现、根因已知 | core/backtrace、当前版本、跨版本对照 |
| README 过时 | docs defect | controller 实现有 bug | docs-only patch |
| Open Issue + no assignee | 当前页面状态 | 完全无人处理、范围仍原样 | 阅读 linked/merged PR 和 current main |
| fake ActionServer 测试通过 | injected contract PASS | real MoveGroup health | 保留 `_injected` 后缀；需要独立 real evidence |

遇到拿不准的问题，正确输出是 `UNVERIFIED` 或“暂无可靠数据”，而不是选择更像成果的解释。

## 最终证据

- `use_camera=false` 下的 URDF/SRDF link-set 矛盾是固定上游 commit 可直接核对的配置缺陷；srdfdom warning 是预期防御，不是 MoveIt 核心 bug。
- gripper controller 的 README、YAML 与官方接口文档存在确定的文档/接口错配；是否导致指定 runtime 失败尚未在本轮验证。
- SO-ARM100 #54 仍 Open，但已有相关修复合并，因此不能把原 Issue 全部范围当作当前无人处理的 bug。
- EdgeGrasp 自己已经修复并回归了 PlanningScene false pulse、teardown ordering、cancel/terminal 与 generation/UUID 隔离；这些仍按本地成果分类。
- 本轮只读核对上游和编写文档，没有向任何上游创建 Issue、PR、评论或认领。

证据层：documentation-only + source/static audit；不新增 MoveGroup、controller、simulation physics 或 hardware runtime 证据。

## 仍不能宣称的边界

- 没有 fresh Ubuntu/Jazzy camera true/false 启动结果；不能说候选 A 已 runtime 修复。
- 没有 isolated ros2_control introspection；不能说候选 B 是已确认运行 bug。
- 没有当前 MoveIt crash 的独立最小复现、backtrace 和跨版本矩阵。
- 没有修改上游仓库，也没有公开提交、维护者确认或合并记录。
- 论文、作者 benchmark、产品宣传和路线建议仍是 documentation-only，不能进入 EdgeGrasp runtime 证据链。

## 小练习

1. 一个 warning 同时提到 MoveIt 和不存在的 robot link，你会先检查哪两份文件？
2. YAML 写 JTC、README 给 Float64MultiArray 示例时，为什么先做 docs patch 比直接换 controller 更安全？
3. 为历史 MoveIt crash 列出升级为 deterministic upstream bug 至少还缺的四项证据。
4. 找一个 Open Issue，核对 merged PR 后重写它的“当前剩余范围”。
5. 用本章模板写一张 PR-ready 审计卡，但不要真的发布。
