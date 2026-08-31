# 第 9 章：Windows/WSL 复现——PATH、EOL、raw bytes 与 teardown

## 学习目标

- 理解 Windows host、WSL guest 和 ROS workspace 是三层不同环境；
- 避免 Python/PYTHONPATH、ROS setup 与 monorepo layout 混淆；
- 理解 LF/CRLF 为什么能改变 raw SHA-256 而不改变 XML 语义；
- 理解 rclpy teardown warning 为什么也属于证据。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| host | Windows 本机；负责代码、PowerShell 静态检查和 WSL 管理 |
| guest | WSL 中的 Ubuntu；ROS 2/Gazebo runtime 所在处 |
| PATH | shell 查找 python、ros2、gz 等可执行文件的目录列表 |
| PYTHONPATH | Python 额外查找模块的目录；临时便利不能替代正确安装 |
| source setup.bash | 把 ROS/colcon 安装路径加入当前 Linux shell 环境 |
| EOL | 行尾；Windows 常见 CRLF，Linux 常见 LF |
| raw-byte hash | 直接对文件每一个 byte 做 SHA-256，换行不同就会不同 |
| teardown | 测试或节点结束时停止 executor、销毁 node、shutdown context 的顺序 |

## 前置知识

- 会在 PowerShell 与 Bash 中辨认当前目录；
- 知道 Windows 的 python 和 WSL 的 python3 不是同一程序；
- 知道 Git checkout 可能按 .gitattributes 规范化行尾。

## 系统图/流程图

~~~mermaid
flowchart TB
    W[Windows 11 host<br/>PowerShell + Git + Python core] --> A[scripts/check.ps1<br/>Ruff / JSON / syntax]
    W --> X[WSL 2 exact distro selection]
    X --> U[Ubuntu 24.04 guest<br/>ROS 2 Jazzy + Gazebo Harmonic]
    U --> R[/home/edgegrasp/ros2_ws<br/>isolated colcon workspace]
    R --> P[pinned upstream copy]
    R --> E[complete edgegrasp-sim monorepo copy]
    P --> B[colcon build/test]
    E --> B
    B --> O[new test_results artifact]
~~~

图源：[environment audit](../environment-audit.md)、[Ubuntu/Jazzy runbook isolated workspace](../ubuntu-jazzy-runbook.md#isolated-workspace-and-exact-sources) 和 validation report。

替代文本：Windows 负责静态/core 检查并精确选择 WSL distro；Ubuntu 24.04 guest 中使用隔离 ros2_ws，将固定上游和完整 EdgeGrasp monorepo 分开复制后 build/test，结果写入新 artifact。

~~~mermaid
flowchart LR
    H[历史 Candidate024 SDF<br/>CRLF bytes] -->|raw SHA-256| A[历史 world hash]
    C[当前 Candidate024 SDF<br/>canonical LF bytes] -->|raw SHA-256| B[当前 world hash]
    H -->|同一 SDF 内容<br/>CRLF 转 LF 可解释差异| C
    A -. 不能替代 .-> B
    J[历史 Candidate012 contract JSON<br/>旧 SDF pin] -->|raw SHA-256| D[历史 contract hash]
    K[当前 Candidate012 contract JSON<br/>新 canonical-LF SDF pin] -->|raw SHA-256| E[当前 contract hash]
    J -->|不仅是 EOL<br/>嵌入的 source pin 也改变| K
    D -. 不能替代 .-> E
    B --> V[raw-byte validator]
    E --> V
~~~

图源：[.gitattributes](../../.gitattributes)、[validation report](../validation-report.md) 的 source-contract 段、[Candidate012](../observations/2026-08-28-candidate012-pad-friction-runtime.json) 与 [Candidate024](../observations/2026-08-29-candidate024-face-aligned-runtime.json) observations。

替代文本：Candidate024 world SDF 的历史 CRLF 与当前 LF 能解释其 raw hash 差异；Candidate012 contract JSON 还刷新了嵌入的 SDF source pin，不能只归因于换行。两类历史 hash 都保留 provenance，不能覆盖当前 source contract。

## 实际操作

### Windows 安全只读环境盘点

以下命令来自 runbook/environment audit，本轮博客写作没有改变 WSL 状态：

~~~powershell
cmd.exe /c ver
wsl.exe --status
wsl.exe --version
wsl.exe --list --verbose
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\audit_environment.ps1 -IncludeWsl -WslDistribution Ubuntu-24.04 -WslTimeoutSeconds 30
~~~

不要让 audit script 自动执行 wsl --update、wsl --shutdown、安装/注销 distro、Windows feature、SFC/DISM、driver 或 reboot；它们都是需要另行授权的 host mutation。

### 核对 Python 身份

~~~powershell
Get-Command python
python --version
.\.venv\Scripts\python.exe --version
.\.venv\Scripts\python.exe -m pip --version
~~~

使用同一个显式解释器执行 venv 内的 pip、pytest 和 edgegrasp。scripts/check.ps1 临时设置项目 src 到 PYTHONPATH，并在 finally 中恢复；tests/test_packaging.py 另行清除 PYTHONPATH/PYTHONHOME，验证真实安装可导入。

### WSL guest 只读盘点

~~~bash
source /opt/ros/jazzy/setup.bash
bash scripts/audit_environment.sh
python3 --version
ros2 doctor --report
gz sim --versions
~~~

先 source ROS，再启用严格 nounset 或运行依赖 AMENT 变量的 shell。一次失败就是因为先 set -u，ROS setup 读取未定义 AMENT_TRACE_SETUP_FILES，build/test 根本没开始。

### 核对 EOL 与 raw SHA-256

~~~powershell
git check-attr text eol -- ros_ws/src/edgegrasp_ros/worlds/table_cube.sdf
git check-attr text eol -- ros_ws/src/edgegrasp_ros/worlds/table_cube_candidate024_face_aligned.sdf
Get-FileHash -Algorithm SHA256 ros_ws\src\edgegrasp_ros\worlds\table_cube.sdf
Get-FileHash -Algorithm SHA256 ros_ws\src\edgegrasp_ros\worlds\table_cube_candidate024_face_aligned.sdf
~~~

不要在 validator 内临时把 CRLF 变 LF 再算 hash；那会掩盖文件传输/checkout 漂移。当前合同要求严格 raw bytes。

## 期望输出

### 带日期的环境事实

environment-audit 的 2026-08-26 快照记录：

- Windows 11 Pro 10.0.26200 x64；
- Windows project Python 3.10.11；Windows 原生 CMake/colcon/ROS 2/gz/RViz2 absent；
- WSL 2.7.12.0；
- 精确选择 Ubuntu-24.04，guest 为 Ubuntu 24.04.4 LTS；
- guest Python 3.12.3、Gazebo Sim 8.11.0，ROS 2 Jazzy/MoveIt/ros2_control 包可见；
- 旧命令 gazebo 缺失是 Harmonic 使用 gz 的预期现象。

这些是有日期的 inventory，不应当作永远不变的实时状态。

### 当前 canonical raw hashes

~~~text
table_cube.sdf
8a4b436cd4863a0801602d15bfafebecaadfbe104d4b7db93e4a5f33415bb4c4

table_cube_candidate024_face_aligned.sdf
058e3237b430c56dbcfcde1091573bf8ce6827e6322114c094011a16095af0ba
~~~

.gitattributes 要求 *.sdf、*.py、*.md、*.json、*.yaml 等为 LF，*.ps1 为 CRLF。

Candidate024 historical observation 的 world raw hash 前缀 `14312067…` 来自 CRLF working-copy snapshot；把当前 LF 内容换成 CRLF 可解释该差异，但不能说两个 raw snapshot 完全相同。完整历史值以对应 observation 字段为准。

Candidate012 historical observation 记录的 experiment contract hash 前缀是 `6cb370…`；当前 contract JSON raw hash 前缀是 `2ad8a1…`，因为当前提交刷新了 canonical-LF source pins。当前 table_cube.sdf hash 与当前 contract 内部的 `8a4b…` 前缀相符。省略号表示这里只引用 SHA-256 前缀，完整值以对应 source/observation 为准。历史运行仍是有效历史 observation，但精确 source snapshot 应回到当时 commit。

## 踩坑记录

### 常见风险：Windows python、py launcher 与 venv 混用

如果 py -3.10 创建 venv，却用 PATH 上另一个 python 安装/运行，就可能出现“pip 成功、import 失败”。当前 checked-in observations 没有记录一次这样的实际事故；仓库通过显式 .venv 解释器和清除 PYTHONPATH/PYTHONHOME 的 packaging test 防范这个常见复现风险。

### 只复制 edgegrasp_core ROS 子包

edgegrasp_core 的 setup.py 通过 parents[3] 找项目根 src/edgegrasp。这是单一源码树设计；孤立复制会故意报 expects ros_ws/src/edgegrasp_core inside the EdgeGrasp project。

### Windows→WSL 变量式 rsync 变成 root self-scan

一次 native-command boundary 错误展开了变量，造成根路径自扫描。它没有 --delete，并被及时中断。修复流程是：

1. 使用 literal absolute source/destination；
2. 先 dry-run；
3. 检查输出只包含预期 project files；
4. 不做 broad delete；
5. 再执行 exact sync。

### 把 Git blob SHA-1 和 source-contract SHA-256 混在一起

Git 对 object 有自己的 hash；实验合同使用文件 raw SHA-256。算法和输入 bytes 都不同。报告里必须写明算法、路径、commit 和 EOL。

### 在工作副本自动改 EOL

为了“让测试过”直接转 CRLF/LF 会改变 raw provenance。正确做法是由 .gitattributes 固定 canonical checkout，并在历史 observation 中保留原 hash。

### rclpy context 已 shutdown 才取 future

Jazzy fake/action test harness 的历史 teardown 曾出现 InvalidHandle、Destroyable 和 invalid-context wait-set。该 harness 的根因是 ActionServer coroutine 尚在返回、executor worker 未 join，就销毁 node/context；修复顺序是停止接收新工作、等待 pending action、join workers、取出 retained exceptions、destroy nodes，最后 shutdown context。这不证明真实 MoveGroup 的 SIGINT -11 或 PlanningScene runtime teardown 已被同一修复解决。

## 排查过程

| 问题 | 先检查 | 正确证据 |
| --- | --- | --- |
| import 失败 | Get-Command python、python -m pip 路径、PYTHONPATH | 同一解释器的安装与隔离 import |
| ros2/gz 找不到 | 当前 shell 是否 source Jazzy；是否在 WSL guest | package prefix/inventory；不是 runtime |
| colcon 看不到 core | 是否复制完整 monorepo | ros_ws 与根 src 同时存在 |
| hash drift | raw bytes、EOL attr、commit、算法 | 当前 LF hash与历史 hash分列 |
| rsync 范围异常 | literal absolute path dry-run | OUTSIDE_ROOT_COUNT=0/预期文件清单 |
| teardown warning | executor pending tasks、context 顺序、实际 child PID | JUnit + stderr + clean process exit |

~~~mermaid
flowchart TD
    A[hash 不一致] --> B{算法都是 SHA-256 raw bytes?}
    B -->|否| C[统一算法与文件路径]
    B -->|是| D{commit 相同?}
    D -->|否| E[保留两个 source snapshot]
    D -->|是| F{EOL 相同?}
    F -->|否| G[解释 CRLF/LF provenance<br/>不在 validator 中归一化]
    F -->|是| H[检查内容漂移或传输损坏]
~~~

图源：[Candidate012 validator](../../scripts/validate_candidate012_experiment.py)、[Candidate012 observation](../observations/2026-08-28-candidate012-pad-friction-runtime.json) 与 [Candidate024 observation](../observations/2026-08-29-candidate024-face-aligned-runtime.json) 的 source-contract 审计。

替代文本：hash 排查依次确认算法/路径、commit 和 EOL；若只是 CRLF/LF，也要保留两个 raw snapshot，不能在 validator 内归一化掩盖。

## 最终证据

- Windows 与 WSL inventory 有明确日期、distro 和 guest metadata。
- 当前 static/core 与历史 ROS runtime 分开报告。
- packaging tests 覆盖无 PYTHONPATH 的标准安装和 monorepo adapter 的故意孤立失败。
- .gitattributes 与当前两个 SDF raw SHA-256 已直接核对。
- 最新 package result 没复现早期 teardown warning，但历史 warning 与某次 MoveGroup Ctrl-C exit -11 仍保留。

证据层：environment/static、packaging 和历史 scoped runtime；不是硬件。

## 仍不能宣称的边界

- WSLg、DISPLAY、WAYLAND_DISPLAY 不证明 GUI/OpenGL 性能。
- package prefix 不证明 SO-101 runtime。
- Git EOL 可解释 hash 差异，不证明历史与当前 raw bytes 相同。
- 当前 clean teardown 不能推广到所有 ROS shutdown 路径。
- Windows 没有 ROS 工具不代表项目无法在 WSL 运行；反之，WSL runtime 也不是 Windows-native runtime。

## 小练习

1. 写一条命令同时显示正在使用的 Python 与 pip 路径。
2. 为什么 tests/test_packaging.py 要清除 PYTHONPATH？
3. 同一 XML 只改 LF 为 CRLF，语义和 raw hash 分别会怎样？
4. 为一次 Windows→WSL sync 写出“不使用 --delete”的 dry-run 检查表。
5. 排出正确 teardown 顺序：destroy node、join worker、shutdown context、等待 pending action。
