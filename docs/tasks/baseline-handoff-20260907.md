# 2026-09-07 基线整理交接

本轮验收已完成：接续未提交修复，固定 AM 基线，提供默认零运动的检查/命令生成/证据汇总入口，
定义三位置小型评测清单并完成必要回归。没有运行新仿真实验；不要求本轮证明两个偏移位置成功。
工作从 `2026-09-07 13:03:22 UTC` 开始，在两小时预算内提前完成，没有为了用满预算扩项。

## 首先打开

- [统一入口与使用说明](../release-cycle-baseline.md)
- [配置](../../config/release_cycle_baseline.json) / [来源锁](../../config/release_cycle_baseline.lock.json)
- [机器交接收据](../observations/2026-09-07-baseline-packaging.json)
- [逐轮与分组报告](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/baseline-evidence/report.md)
- [完整 JSON](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/baseline-evidence/report.json) / [逐项 CSV](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/baseline-evidence/rounds.csv)
- [已生成但未执行的仿真命令](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/command-review-01/reviewed-command.sh)

外部文件仅在本机，路径不代表已上传。机器收据保存这些文件的原始 SHA-256。

## 接续与改动

当前 worktree：`C:/Users/huang/.codex/worktrees/487a/edgegrasp-sim-private`。
整理验收时 HEAD 为 `e58586f8d7dfa294697c2f7cb5525cf995e73613`，处于 detached HEAD，尚未提交或推送。
随后用户授权提交推送，交付分支为 `codex/release-cycle-baseline`；提交与远端状态以 Git 记录为准。
开始确认工作区干净后，核验交接 manifest、tracked.patch SHA-256、24 个新增文件的相对路径和 SHA-256，
`git apply --check` 成功后恢复 18 个 tracked 文件修改及 24 个新增文件，没有覆盖冲突文件。
结束前 24 个恢复文件仍逐字节匹配快照，AM 的 20 个运行源码也全部匹配。
原 `961f` 工作区、交接快照、原始日志均保持只读。

本次在恢复内容上新增 9 个文件，修改 3 个文档：

| 文件 | 用途 |
|---|---|
| `config/release_cycle_baseline.json`、相邻 `.lock.json` | 版本、三位置、参数/判据、包版本、源码/资产/AJ 规划来源锁 |
| `scripts/baseline.py` | 统一 check / command / summarize 命令行，输出仅允许新外部目录 |
| `src/edgegrasp/baseline.py` | 只读检查和显式执行开关的命令生成；拒绝偏移位置复用 control |
| `src/edgegrasp/baseline_results.py` | 日志核对、独立判据、失败/未执行/历史 COMPLETE 分账与 JSON/CSV/Markdown 输出 |
| `tests/test_baseline.py` | 27 项配置漂移、来源锁、越界、假终态、缺失/损坏记录、分母回归 |
| `docs/release-cycle-baseline.md`、本文件、机器交接收据 | 复现说明、剩余限制与实际验证证据 |
| `README.md`、`docs/tasks/next-release-cycle.md`、`docs/ubuntu-jazzy-runbook.md` | 同步 AM 结果和新入口 |

此前恢复的机器人运行源码没有继续修改。AGENTS.md、Skills 和权限边界未改动。

## 证据结论

原始证据 222 项哈希核对全部通过，生成 113 条记录。完整循环历史跨多个修复版本，
不能把汇总解释成 AM 基线成功率。

| 记录范围 | 结果 |
|---|---|
| AM 同一仿真 | 独立抓取与完整循环均 3/3；整轮重试 0，轮间控制器重启 0 |
| release 历史已进入轮次 | 37 次；独立抓取通过 29 次，当前完整判据通过 11 次 |
| 其余已进入轮次 | 23 次失败；3 次旧 COMPLETE 未记录当前再抓取几何，单独保留 |
| 组内后续未进入轮次 | 10 轮未执行，计入各组计划数，不计入已尝试分母 |
| release 五段规划 | 20 次，4 次拒绝；不计作物理动作尝试 |
| 早期 RGB-D | 全部 41 目录保留；规划、抓取、启动/准入和其他记录单列 |
| X−1 mm / X+1 mm | 早期抓取成功保留；当前完整循环均 NOT_READY，未执行 |

汇总读取独立观察器与类型化门控日志，不重新计算 MCAP 物理，不以 COMPLETE 标签代替独立判定。
全部真实硬件结论仍未验证；真实 MoveGroup 超时和强请求关联标志继续为 false。

## 实际命令与测试

所有命令从当前 worktree 运行；下列外部结果目录已经存在，复跑必须换新名称。

```powershell
powershell -NoProfile -File scripts/check.ps1
ruff check scripts/baseline.py src/edgegrasp/baseline.py src/edgegrasp/baseline_results.py tests/test_baseline.py
python -m pytest tests/test_baseline.py -q
python scripts/baseline.py summarize --evidence-root '//wsl.localhost/Ubuntu-24.04/home/edgegrasp/ros2_ws/test_results' --output 'C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/baseline-evidence'
```

Windows 最终 **409 passed / 3 skipped**，结构检查 PASS，三场景各 100 次确定性回放通过。
跳过项为本机 Windows 缺少 perception extra，以及固定 SO-101 checkout 缺失的两项检查；没有安装依赖。
[完整日志](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/windows-check-final.log)。

WSL 实际执行以下仓库外检查脚本。它复用前次 171 项零运动回归，增加 27 项新测试，最终 **198 passed**：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/check_baseline_zero_motion.sh
```

[精确测试命令与文件列表](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/check_baseline_zero_motion.sh) ·
[完整日志](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/wsl-zero-motion-final.log)。
测试中的 ROS 服务均为进程内假依赖，没有运行真实 MoveGroup、Gazebo 或控制器。

已执行的 WSL 命令生成命令如下；它只写检查结果与待审核脚本：

```bash
python3 scripts/baseline.py command --case control \
  --evidence-root /home/edgegrasp/ros2_ws/test_results --runtime-root / \
  --domain 227 --task-id baseline-control-review \
  --artifact-dir /home/edgegrasp/ros2_ws/test_results/baseline_control_review_20260907 \
  --linux-repo /mnt/c/Users/huang/.codex/worktrees/487a/edgegrasp-sim-private \
  --output /mnt/c/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/command-review-01
```

[文件/版本检查结果](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/command-review-01/check.json)
为 `command_preparation_ready=true`，不是实时健康结论。生成脚本 `bash -n` 通过；
不带参数运行只打印 `Dry-run: no processes started`，未选择 `--execute-simulation`。
两个偏移位置检查均返回预期退出码 2。没有创建声明的实验目录，也未将域 227 用于新仿真实验。

首轮安装资产校验拒绝 SDF 原始哈希差异；随后只读证实为 CRLF/LF，分别锁定原始哈希后通过，
[首次拒绝记录](C:/Users/huang/.codex/visualizations/2026/09/07/01a07bf7-1a1e-7720-b8dc-c7e4c9ffca84/runtime-check-01/check.json)
保留。环境和 SDF 均未修改。

## 剩余工作

1. 为两个偏移位置分别准备完整循环场景、放回配置和同源五段规划；保持原 candidate024 几何限制。
   当前固定 control 评分中心也须明确处理，不能复制 control 规划或改个标签宣称已准备好。
2. 如果下一次任务授权仿真，再明确选择专用域和新输出目录，遵守组内首次失败即停止的协议。
   本轮的命令文件不代表未来任务已获授权，也不保证下一次复现成功。
3. 干净机器复现仍缺可分发原始日志/AJ 规划、安装 overlay、模型及完整依赖交付。
   当前文件/包版本检查不递归验证所有 xacro/网格依赖，不证明实时控制链健康。
4. 当前汇总覆盖指定历史账本；新实验若不在账本中，需要先建立独立的新记录来源，不能悄悄并入旧分母。

整理阶段没有远端访问、下载/安装、环境重建、硬件动作、发布、提交/推送或记录删除。
