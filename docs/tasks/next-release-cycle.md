# 下一会话：正常释放与完整操作循环

状态：交接已准备，尚未创建执行会话、尚未开始本阶段实验。
路线图：[阶段一](../roadmap.md#阶段一完成可重复操作循环下一阶段未开始)。
工作基线：`25050e6e64cf6e777c8d4442c99e28f881c22ab4`，后续先核对 main 的实际进度和未提交文件。

## 建议任务描述

> 在 EdgeGrasp 项目中推进固定方块的正常释放和可重复操作循环。
> 先阅读 AGENTS.md、docs/roadmap.md、docs/rgbd-static-grasp-result.md 和本任务说明，
> 从现有日志与源码区分失败的具体环节，给出证据与最小修复。
> 首先完成控制位置的抓取、保持、放回、张开和撤离；单次通过后再按预先声明的次数验证连续循环。
> 保留安全门控、原有独立抓取判定和全部失败记录。不要接入新算法或硬件，不要重跑历史进程冻结实验。
> 以本次会话实际授权界定执行范围；若当前只授权诊断，就完成日志分析、代码修复及零运动检查，
> 将具体有界仿真方案准备好后再申请尚缺的授权。

## 首个可完成的小任务

先回答“为什么正常释放没有成功”，不要直接添加整套放置状态机。

- 读取 `scripts/run_candidate005_contact_quality.sh` 的 `bounded_zero_effort_release` 段：
  当前调用 `prepare_so101_trial.py --gripper-only --gripper 1.5 --gripper-effort 0.0`，
  失败后才停用控制器并检查 effort 接口 unclaimed。
- 从 `prepare_so101_trial.py` 追踪类型化门控和下游 FJT，核对目标参数、反馈、终态和拒绝原因。
- 区分“夹爪张开”“零 effort 命令”“归还控制接口”“方块已受支撑”四件事。
  控制器 inactive/effort unclaimed 只证明回退状态，不证明正常放置/释放。
- 先复现最小相关软件故障，增加必要的代表性回归；无证据时写明未知，不把服务发现猜测当作根因。

## 已有证据入口

本机 WSL `/home/edgegrasp/ros2_ws/test_results/` 中：

- `rgbd_measured_pad_trial_20260906b`：control 抓取成功，正常释放失败。
- `rgbd_x_minus_trial_20260906b`、`rgbd_x_plus_trial_20260906a`：平移位置抓取成功，释放问题保留。
- `rgbd_x_minus_trial_20260906a`：闭合前服务不可用而停止；这是不同故障，不能混为释放根因。

重点文件：`gripper_release.log`、`gripper_release_exit_code.txt`、`sequence.log`、
`adapter.log`、`trial.log`、控制器状态及 hardware_interfaces 日志、MCAP。
代码退出码、正常动作结果与整个试验 wrapper 的退出码可能不同，要读实际文件。

仓库的 [验证摘要](../observations/2026-09-06-rgbd-final-validation.json)、
[录包索引](../observations/2026-09-06-rgbd-artifact-inventory.json) 和
[Blender 工作流](../blender-replay.md) 提供其他入口。
上述路径不随 Git 上传；换机器需明确取得源数据，不声称仅克隆即可读到日志。

## 交付与边界

交付：根因说明、最小补丁、必要测试、具体运行命令、正常释放/落放/撤离判定、
逐次结果，以及代表性的成功/失败回放。具体验收沿用路线图阶段一。

新路径需先无执行规划，再在获授权范围内有界运行；原始录包写在仓库外。
不得清除安全锁存、放宽时间阈值、强制停启进程或删除失败证据来获得成功。
真实 MoveGroup 的超时/请求关联和所有硬件标志保持原边界，仿真和回放不提升证据类别。

建议新会话执行，以免把前序展示、发布、清理事项混入本任务。
创建时确保能读到本交接文件：若采用新 worktree，需包含这些新增文档或先保存提交；
不要假定尚未提交的文件会自动出现在另一 worktree。
