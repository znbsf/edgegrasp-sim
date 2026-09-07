# AM 基线：配置、只读检查与已有证据

版本 `candidate024-am-20260906-v1` 固定在 `e58586f8d7dfa294697c2f7cb5525cf995e73613`
加已核验的未提交交接修复。AM 在固定 control 场景、同一仿真内连续 3/3 完整循环通过；
AL 是同源码的单轮复测。两个 X±1 mm 位置只有早期抓取成功，完整循环仍未准备好。
本次整理没有运行新 ROS / MoveGroup / Gazebo / 控制器实验。

入口是 [scripts/baseline.py](../scripts/baseline.py)、
[统一配置](../config/release_cycle_baseline.json) 和 [来源锁](../config/release_cycle_baseline.lock.json)。
默认执行只读检查，不导入 ROS、不启动进程、不生成文件。
配置中的判据是现有源码的固定契约说明，不是放宽评分器的参数；更改配置后锁校验会拒绝，
需要单独建立新的评测版本。旧运行脚本及 candidate024 几何边界保持原样。

## 1. 检查仓库与本机依赖

在新 worktree 根目录执行：

```powershell
python scripts/baseline.py

# 加上原始证据：只读核对 AJ 五段规划及其原始哈希。
python scripts/baseline.py check --evidence-root '//wsl.localhost/Ubuntu-24.04/home/edgegrasp/ros2_ws/test_results'

# 明确得到 NOT_READY/退出 2；不会改用 control 规划。
python scripts/baseline.py check --case x_minus_1mm
python scripts/baseline.py check --case x_plus_1mm
```

在 WSL 的当前 worktree 根目录检查已安装文件。这里只读取文件和 package.xml：

```bash
python3 scripts/baseline.py check --case control --require-local-runtime \
  --evidence-root /home/edgegrasp/ros2_ws/test_results --runtime-root /
```

输出分别列出仓库源码、规划、安装文件与缺失项；`command_preparation_ready`
只表示命令准备所需的文件匹配，不能替代实时图、规划、控制器、物理验证或当前执行授权。
Windows UNC 可读取原日志，但无法可靠解析安装工作区的 Linux 符号链接；
安装环境检查应在 WSL 内执行，不能把 Windows 的符号链接不可读诊断为依赖确实不存在。

本机依赖为 Ubuntu 24.04、Python 3.12.3、ROS Jazzy、Gazebo Harmonic、
现有 `/home/edgegrasp/ros2_ws/install` 及其固定 SO-101 模型。2026-09-07 只读核对的版本为：

| 包 | 本机版本 |
|---|---|
| rclpy | 7.1.11 |
| moveit_ros_move_group | 2.12.4 |
| ros_gz_sim | 1.0.22 |
| gz_ros2_control | 1.2.19 |
| joint_trajectory_controller | 4.40.1 |
| rosbag2_storage_mcap | 0.26.11 |
| NumPy | 1.26.4 |

来源锁保存 AM 的 20 个运行源码原始哈希，以及 5 个仓库资产与相应安装资产。
安装 SDF 为 128 行 CRLF，仓库为 LF；只读比较确认规范化文本完全相同，两份原始哈希分别保存。
没有重写任一 SDF。其他所需资产包括固定 xacro、生成接口、模型网格、控制配置和现有 overlay；
更完整依赖说明见 [Ubuntu runbook](ubuntu-jazzy-runbook.md) 与 [上游版本清单](upstream-manifest.json)。
本入口不安装、不下载、不重建，也没有完成干净机器复现。仅克隆仓库缺少 AJ 原规划、
AM/历史日志和 MCAP、安装 overlay 与模型；缺少任一所需文件必须保留 NOT_READY。

## 2. 从原始日志重现汇总

以下只读取已有文件，输出目录必须是新的仓库外目录：

```powershell
python scripts/baseline.py summarize `
  --evidence-root '//wsl.localhost/Ubuntu-24.04/home/edgegrasp/ros2_ws/test_results' `
  --output 'C:/Users/huang/Documents/edgegrasp-results/am-evidence-new'
```

输出 `report.json`、`report.md`、`rounds.csv`。JSON 保留源账本 SHA-256、原始日志哈希核对、
逐轮原账本条目、所有历史 41 目录索引以及汇总分母。
输入范围明确限定为 2026-09-06 的 release-cycle 初次/迭代账本和 RGB-D 最终验证/目录索引；
不是扫描全部 `test_results` 并自动发现任何新实验的通用评分器。

独立抓取必须在原始 `grasp_trial_result` 中同时有 VERIFIED、双指同帧接触、
至少 20 mm 保持抬升及 retention 成功。完整循环额外要求三个支撑阶段成功记录、
同一 task 的 place / release / retreat 成功终态、实际张开及撤离判据、原 1 mm ready 门、
再抓取几何、成功 handoff 且 epoch 不变、release 退出 0。
这只是核对已记录的独立观察器/门控结果，不重新从 MCAP 计算物理，也不提升为硬件结论。

分母规则：

- 已进入的每轮都属于尝试分母，动作前准入失败也保留。一次组失败后没有进入的后续轮次记为未执行。
- 每组保留计划数、尝试数、成功数和未执行数；AM 仅使用自身三轮，不和 AL 或旧组拼接。
- 初始 A 与后续修复后复测分别标注。不同源码/路径的历史总数不能解释成 AM 成功率。
- K、L 第 1 轮和 M 第 1 轮的旧 COMPLETE 原样保留，单列“未记录当前再抓取几何”，不伪装成当前完整成功，也不改写历史终态。
- 规划拒绝、启动失败、仅运动学候选与早期抓取协议各自统计；不混入完整循环的物理尝试分母。
- 缺日志、哈希漂移、账本矛盾或损坏 JSONL 均拒绝确认成功。不会将缺失值当 0 或静默跳过损坏 JSONL。

原始失败和离线修正见 [首次记录](release-cycle-result.md)、[迭代记录](release-cycle-iteration.md)。
本次实际检查与结果入口见 [整理交接](tasks/baseline-handoff-20260907.md)。

## 3. 三位置评测清单与新实验准备

| 位置 | 本版本计划次数 | 场景/规划 | 已有完整循环证据 | 准备状态 |
|---|---:|---|---|---|
| control | 同一仿真连续 3 轮 | candidate024 face_aligned；AJ 五段 PLAN_ONLY_PASS；有限肩转 ±0.5° | AM 3/3，零整轮重试、零轮间控制器重启 | 固定文件可检查；新运行仍需当前授权和运行时准入 |
| X−1 mm | 同一仿真连续 3 轮 | 完整循环场景、放回配置及相符五段规划缺失 | 未执行；早期抓取成功另记 | NOT_READY |
| X+1 mm | 同一仿真连续 3 轮 | 完整循环场景、放回配置及相符五段规划缺失 | 未执行；早期抓取成功另记 | NOT_READY |

三位置计划数是下一组预声明，不是本轮已运行的次数。偏移位置必须先提供各自场景/世界、
完整抓放配置、版本绑定和原几何门内可用的五段无执行规划；还需明确该位置的支撑/ready 判据。
当前 `StablePlacement` 固定在 control 中心，不能简单平移名称或复用 control 规划当作完成准备。
准备工作涉及新规划/仿真时，以当时授权为准；本轮未执行这些步骤。

生成 control 的新实验命令（在 WSL 当前 worktree；此命令本身不运行仿真）：

```bash
python3 scripts/baseline.py command --case control \
  --evidence-root /home/edgegrasp/ros2_ws/test_results --runtime-root / \
  --domain 227 --task-id baseline-control-new \
  --artifact-dir /home/edgegrasp/ros2_ws/test_results/baseline_control_new \
  --linux-repo "$PWD" --output /home/edgegrasp/baseline_commands_new

# 生成脚本的默认行为仍是 dry-run，仅打印提示：
bash /home/edgegrasp/baseline_commands_new/reviewed-command.sh
```

`227` 只是待选择和核对的专用域示例，本轮未探测其运行状态。
生成脚本保留明确的 `--execute-simulation` 执行开关；只有当前任务明确允许新仿真后才选择它。
执行前脚本再次核对源/配置/规划/依赖，拒绝已有输出目录，清除继承的 `EDGEGRASP_*` 实验覆盖，
再调用既有 `run_candidate005_contact_quality.sh`。不会自动尝试别的位置或自动重复整轮。
原 runner 继续执行专用域检查、类型化门控和按本次进程归属清理。
每轮 sequence 90 s、physics 100 s，trial 墙钟 260 s，释放循环墙钟 160 s，
每个支撑等待墙钟 20 s；失败立即结束当前组，保留回退结果。脚本不会证明下一次运行必然成功。

## 4. 零运动回归

```powershell
powershell -NoProfile -File scripts/check.ps1
ruff check scripts/baseline.py src/edgegrasp/baseline.py src/edgegrasp/baseline_results.py tests/test_baseline.py
python -m pytest tests/test_baseline.py -q
```

WSL 可用现有 `bash scripts/check_rgbd.sh` 检查 RGB-D 子集；本次还接续上一会话的
171 项零运动回归并加入新入口测试，实际完整命令保存在整理交接所链接的检查脚本。
其中 ROS 服务测试使用进程内假依赖，不启动真实 MoveGroup、Gazebo 或控制器。
真实 MoveGroup 超时验证与强请求关联两个标志继续为 false。
