# 第 0 章：阅读与安全环境准备

本章是第 1 章之前的先导。它先回答三个问题：代码应该从哪里读、Windows 和 WSL 分别负责什么、哪些命令可以安全跟做。它不启动 ROS、MoveIt、Gazebo、controller 或真实机械臂。

## 学习目标

读完本章，你应该能：

- 找到 EdgeGrasp 的仓库根目录，并知道为什么命令要从那里运行；
- 用白话解释 Windows host、WSL guest、Python venv、commit、branch 和 detached HEAD；
- 使用同一个显式 Python 做 venv、pip 和 Windows 静态检查；
- 判断一段文字是当前可跟做的静态/core 操作、需要已有隔离 workspace 的运行步骤，还是只能阅读的历史 artifact；
- 在没有 Ubuntu/Jazzy workspace 时，安全地停在 core/static 层，而不是用推测补上 runtime 证据。

## 术语白话解释

| 术语 | 白话解释 |
| --- | --- |
| 仓库根目录 | 项目的最外层目录；本项目根目录同时能看到 `README.md`、`pyproject.toml`、`src`、`ros_ws`、`scripts` 和 `docs` |
| host | Windows 本机；本章只在这里做文件、Git、Python 和静态检查 |
| guest | WSL 里的 Ubuntu；后续 ROS 2/Gazebo runtime 只能在已有的隔离 guest workspace 中讨论 |
| Python venv | 项目自己的 Python 小环境；它把解释器和依赖与系统 Python 分开 |
| `PYTHONPATH` | Python 额外寻找模块的目录；临时加入源码目录不能代替正确安装 |
| commit | 一份带唯一指纹的代码快照 |
| branch | 指向一串 commit 的可移动名称，例如 `main` |
| `HEAD` | Git 当前正在查看的那份快照；它可以指向 branch，也可以直接指向 commit |
| detached HEAD | `HEAD` 直接指向某个 commit、没有停在 branch 上；适合只读检查，贡献前应先按项目流程进入工作分支 |
| hash / SHA-256 | 根据内容计算出的指纹；内容或字节不同，指纹就可能不同 |
| artifact | 一次实验留下的原始输出，例如 JSON、JSONL、JUnit、日志或 SDF hash；它记录过去发生过什么，不自动变成可重跑命令 |
| evidence class | 证据层级，例如 `core/static`、`plan-only`、`injected fake`、`controller`、`simulation physics` 和 `hardware` |
| isolated workspace | 专门给某次实验使用的 workspace、domain 和新 artifact 目录；避免把别的 graph 或旧输出混进来 |

## 前置知识

只需要：

- 会打开 PowerShell、复制路径和阅读 Markdown；
- 知道“命令在终端里运行、文件有路径”；
- 不需要预先懂 Python、Linux、ROS 2、机器人学或 Git 分支操作。

如果你还没有 Ubuntu 24.04/Jazzy，仍然可以完整阅读本章和第 1 章的 core/static 部分；没有 workspace 不等于项目失败，只表示暂时不能进入 ROS runtime 章节。

## 系统图/流程图

~~~mermaid
flowchart LR
    A[README 学习入口] --> B[第 0 章<br/>确认路径与边界]
    B --> C[Windows venv<br/>core/static 检查]
    C --> D{已有隔离<br/>Ubuntu 24.04/Jazzy workspace?}
    D -->|否| E[阅读历史 artifact<br/>停在 core/static]
    D -->|是| F[按证据类型进入第 2–8 章]
    F --> G[第 9 章<br/>排查 host/guest 差异]
    E --> H[第 10 章<br/>学习证据维护]
    G --> H
    H --> I[新实验前重新确认<br/>source、artifact、claim boundary]
~~~

图源：[博客入口](README.md)、[项目 AGENTS.md](../../AGENTS.md)、[环境审计](../environment-audit.md) 和 [Ubuntu/Jazzy runbook](../ubuntu-jazzy-runbook.md)。

替代文本：先从 README 和本章确认路径与边界，再在 Windows 做 core/static 检查；只有已有隔离 Ubuntu/Jazzy workspace 时才进入后续 runtime 章节，否则只阅读历史 artifact，最后统一回到证据维护。

## 实际操作

以下操作都属于 Windows 的安全准备或静态检查。本章没有 ROS launch、MoveIt plan、Gazebo stepping、controller goal 或硬件命令；本章也不创建 WSL distro、不安装 ROS、不关闭 WSL。

### 1. 先确认仓库根目录

把下面的示例路径替换为你自己的 checkout 路径，然后在 PowerShell 中执行：

~~~powershell
Set-Location -LiteralPath 'C:\path\to\edgegrasp-sim-private'
git rev-parse --show-toplevel
Get-Item -LiteralPath README.md,pyproject.toml,src,ros_ws,scripts,docs | Select-Object -ExpandProperty Name
~~~

第二条命令应打印仓库根目录；第三条命令应只列出六个顶层入口。它们用于确认你没有停在 `docs/learning-blog`、`src` 或某个临时 artifact 子目录。若 `git rev-parse` 失败，先回到正确 checkout，不要修改路径变量来掩盖错误。

### 2. 只读查看 Git 快照

~~~powershell
git branch --show-current
git rev-parse --short HEAD
git status --short --branch
~~~

`git branch --show-current` 输出为空通常表示 detached HEAD。它不是代码损坏：你仍在读取一个明确的 commit，只是没有处在可移动的 branch 名称上。本章只查看，不在 detached HEAD 上提交，也不使用 `git reset --hard`、`git checkout --` 或删除命令覆盖已有工作。

### 3. 创建并核对 Windows Python venv

本项目的纯 Python core 要求 Python 3.10 或更高版本；为了避免系统 Python、`py` launcher 和 venv 混用，后续命令都显式写出 venv 解释器：

~~~powershell
py -3.10 --version
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe --version
.\.venv\Scripts\python.exe -m pip --version
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
~~~

如果 `py -3.10` 不存在，先停在这里并记录“本机没有可用的 Python 3.10”；不要用另一个未知版本创建 venv 后仍把输出写成 3.10 结果。`.venv` 是 checkout 内的本地环境，不是 ROS workspace，也不证明 controller 或 Gazebo 已安装。

### 4. 运行允许的 Windows core/static 检查

~~~powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\check.ps1 `
  -ReplayRuns 100 `
  -PythonExecutable .\.venv\Scripts\python.exe
~~~

这个检查路径只验证项目结构、Python 测试、确定性 replay 和静态合同；它不会启动 ROS、MoveGroup、Gazebo、controller 或硬件。`scripts/check.ps1` 可能临时设置项目 `src` 到 `PYTHONPATH`，但会在结束时恢复；这不应被误写成“安装已经通过”或“仿真已经运行”。

### 5. 可选：只读查看 WSL 是否存在

这两条命令只查看 Windows 侧的 WSL 状态，不进入 distro，不启动 ROS：

~~~powershell
wsl.exe --status
wsl.exe --list --verbose
~~~

如果没有 `Ubuntu-24.04`，本章仍然完成；这只表示后续 Ubuntu/Jazzy runtime 章节暂时不能跟做。不要在本章执行 `wsl.exe --update`、`wsl.exe --shutdown`、安装/注销 distro、Windows feature、SFC/DISM、driver 或 reboot。

### 哪些内容可以跟做

| 内容 | 现在能否跟做 | 最低条件 | 读者应怎样写结论 |
| --- | --- | --- | --- |
| 本章路径、Git、venv 检查 | 可以 | Windows checkout 和 PowerShell | 这是环境/static 事实 |
| 第 1 章 Python tests/replay | 可以 | 显式 venv；依赖安装成功 | 这是 `core/static` 证据 |
| 第 2–8 章 ROS/MoveIt/controller/Gazebo 命令 | 有条件 | 已有隔离 Ubuntu 24.04/Jazzy workspace、新 artifact 目录和对应 evidence class | 只报告本次明确运行的层级 |
| README、JSON observation、历史报告阅读 | 可以 | 文件路径存在 | 这是历史或静态来源，不是 fresh runtime |
| 历史 real-process interruption 或硬件操作 | 不可以 | 本仓库和本章都不提供安全复现路径 | 保留为只读负证据/未验证硬件边界 |

后续章节中的 Ubuntu/Jazzy 命令只在它们各自的证据类型和 runbook 前提下才有意义；本章不把“有命令”当成“已经运行”。

## 期望输出

完成本章后，至少应能得到下面这组可审计结果：

~~~text
repository root: 能由 git rev-parse --show-toplevel 确认
required top-level paths: README.md / pyproject.toml / src / ros_ws / scripts / docs 可见
python: venv 的 python.exe 与 venv 的 pip 路径一致
git: HEAD commit 可记录；branch 为空时标记 detached HEAD
static check: 只记录实际输出；不把历史数字自动复制为本次结果
runtime side effect: ROS/MoveGroup/Gazebo/controller/hardware = not started by this chapter
~~~

当前博客快照中，第 1 章引用的 Windows 结果是 `332 collected`、`330 passed`、`2 optional skips`；这是某个明确 checkout 和依赖条件下的记录，不是你在另一台机器上必须得到的数字。若重新运行，应保存自己的时间、commit、解释器和 artifact，而不是覆盖历史记录。

## 踩坑记录

### 在子目录运行命令

在 `docs/learning-blog` 里直接运行 `scripts\check.ps1` 会找不到脚本或读错相对路径。先用 `git rev-parse --show-toplevel` 确认根目录，再执行项目命令。

### venv 与系统 Python 混用

`py -3.10` 创建 venv 后，若用 PATH 上另一个 `python`、`pip` 或 `pytest`，可能出现“安装成功但 import 失败”。检查时同时记录：

~~~powershell
Get-Command python
.\.venv\Scripts\python.exe --version
.\.venv\Scripts\python.exe -m pip --version
~~~

测试和脚本优先使用 `.\.venv\Scripts\python.exe`，不要依赖当前目录碰巧让源码可见。

### 把 host、guest 和 workspace 当成同一层

Windows 的 `C:\...` 路径、WSL 的 `/home/...` 路径和 ROS 的 `ros2_ws` 不是同一个环境。Windows 静态检查通过，只能说明 core/static 层可用；它不能代替 Ubuntu/Jazzy package、MoveIt、controller 或 Gazebo 运行。

### 把 detached HEAD 当成损坏

detached HEAD 只是“当前快照没有 branch 名称”。先记录 commit 和工作树状态；不要用 reset、checkout 或删除操作“修复”它。准备贡献时再按仓库协作流程创建或切换工作分支。

### 把历史 artifact 当成复现脚本

Candidate024、P12d 和历史 real MoveGroup JSON 是带时间、commit、hash 和边界的观察记录。看到 `PASS`、`10/10` 或 `12/12` 时，先看 `evidence_class`、source snapshot 和 artifact 路径；没有相同前提就写“历史 observation”，不要写“我刚刚复现”。

### 用 WSL 的存在推导 runtime 健康

`wsl.exe --list --verbose` 只能说明 distro 是否存在、是否运行；它不证明 ROS 2、MoveIt、Gazebo、controller 或模型已经启动。运行证据只能来自对应章节允许的隔离命令和新 artifact。

## 排查过程

| 现象 | 先看什么 | 最小处理 | 结论边界 |
| --- | --- | --- | --- |
| `git rev-parse` 失败 | 当前目录和 checkout 是否完整 | 回到仓库根目录；不改 Git 历史 | 尚未开始项目检查 |
| `py -3.10` 找不到 | `Get-Command py`、Python 版本 | 记录缺少 Python 3.10；不要换解释器冒充 | venv 未准备好 |
| import 失败 | venv 的 `python`/`pip` 路径、`PYTHONPATH` | 用同一个显式解释器重新核对安装 | 不能把 import 失败归因于 ROS |
| static check 有 skip | skip 的完整原因、commit、依赖 checkout | 保留可解释 skip；不要改数字 | 当前 core/static 结果是 partial/带条件 |
| 没有 Ubuntu-24.04 | `wsl.exe --list --verbose` | 停在 core/static 或阅读 artifact | ROS runtime 未验证 |
| 看到历史 `SIGSEGV` 或 `UPSTREAM_PROCESS_EXITED` | observation 的 evidence class 和 claim boundary | 只读记录，不重建 process harness | 这是历史负证据，不是本章新实验 |

推荐顺序仍是：**路径 → 解释器 → Git snapshot → 静态检查 → evidence class → 结论边界**。一次只改变一个准备条件，避免把路径、Python 版本和 runtime 环境混成一个失败原因。

## 最终证据

- 本章只建立阅读路径、Windows venv 和 core/static 检查的准备条件。
- `git rev-parse`、显式 venv Python、`scripts/check.ps1` 输出和工作树状态可以组成可复核的静态记录。
- WSL inventory 只能作为环境存在性信息；Ubuntu/Jazzy runtime 仍由后续章节按证据类型单独验证。
- 历史 artifact 保留原有 commit、hash、时间和 claim boundary，不因本章检查而升级成 fresh runtime 或 hardware evidence。

证据层：environment/static + core/static preparation；本章不产生 ROS、MoveIt、controller、simulation physics 或 hardware 证据。

## 仍不能宣称的边界

- 不能因为 venv 安装成功就宣称 ROS 2、MoveIt、Gazebo 或 controller 已安装。
- 不能因为 Windows tests/replay 通过就宣称仿真物理、机械臂运动或抓取成功。
- 不能因为 WSL distro 存在就宣称 Ubuntu/Jazzy graph 健康。
- 不能把 detached HEAD 的 commit hash 写成 branch 或最新 upstream 状态。
- 不能把历史 JSON、日志或截图写成本轮 fresh rerun。
- 本章不提供真实 MoveGroup 进程中断、真实电机停止或硬件抓取的复现步骤。

## 小练习

1. 你在 `docs/learning-blog` 中运行脚本失败时，为什么先检查 `git rev-parse --show-toplevel`？
2. 用自己的话解释 host、guest 和 ROS workspace 的区别。
3. 为什么 venv 的 `python.exe -m pip` 比直接输入 `pip` 更容易审计？
4. `git branch --show-current` 为空时，HEAD 一定损坏吗？你会保留哪些只读信息？
5. 给下面四项分别标记“可直接跟做 / 需要隔离 workspace / 只能阅读”：第 1 章 replay、Candidate024 历史 JSON、MoveIt launch、硬件抓取。
6. 如果本机没有 Ubuntu-24.04，正确的下一步是扩大 Windows 命令、修改证据数字，还是停在 core/static？说明理由。
