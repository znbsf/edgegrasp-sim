# 正常释放与操作循环：阶段一工作记录

基线 `e58586f8d7dfa294697c2f7cb5525cf995e73613`；本地分支
`codex/release-cycle`。尚未提交或推送。仅本机 Gazebo 固定控制位置。

本页保留首次诊断和 A 的原始结果。后续已经实现正常放回/释放、携物场景及
再抓取几何检查；逐次结果和连续验证状态见[迭代记录](release-cycle-iteration.md)，
不要把本页的历史失败状态当作最新运行结论。

## 历史释放根因

读取了三个原始 `gripper_release.log`、MCAP 的安全/感知/门控状态、
夹爪状态与物体位姿。三次均接受了类型化夹爪目标，随后由安全信号取消，
不是服务发现或 FJT 目标拒绝。时间均为录包仿真时间（秒）：

| 原始运行 | 释放接受 | 首次感知拒绝 | stale_target | 门控开始取消 |
|---|---:|---:|---:|---:|
| rgbd_measured_pad_trial_20260906b | 37.850 | 38.129 | 38.262 | 38.265 |
| rgbd_x_minus_trial_20260906b | 32.825 | 33.117 | 33.239 | 33.243 |
| rgbd_x_plus_trial_20260906a | 42.934 | 43.245 | 43.382 | 43.382 |

控制位置的首个拒绝为 `orientation_outside_declared_bound`，之后为
`missing_orientation_face`。安全监视器对观测过期输出 false；门控取消成功，
下游终态 status=5，类型化外层 status=6、reason=`canceled:safety_false`。
下游 `fjt_error_code=0` 不表示正常完成。

脚本在抬升/保持后直接请求张开，缺少放回阶段。控制位置松开前夹爪约
0.796 rad、方块最低点约 0.198127 m，桌面为 0.180 m；方块没有落在桌上。
松动过程中方块进一步倾转，感知拒绝导致目标流中断。取消前夹爪仅约
0.812 rad，没有达到目标 1.5 rad。这个时间顺序和机械观测支持“悬空松动使
视觉姿态失效”的解释；不是安全监视器直接判定物体无支撑。

必须区分：

- `--gripper-effort 0.0` 是末端轨迹点的前馈值。effort-only JTC 仍计算位置 PID，
  也会插值已有轨迹状态；控制位置 37.888 s 输出约 −0.02261 N·m，
  38.278 s 取消保持阶段约 −0.02001 N·m。该参数不保证实际力矩为零。
- 夹爪关节未到 1.5±0.08 rad，正常张开未成立。
- 后续 inactive/unclaimed 是控制器停用回退，不是正常释放。
- 支撑需要桌面接触、物体位姿及稳定时间独立证据，不能由前三项推出。

离线提取保留在 WSL `/home/edgegrasp/ros2_ws/test_results/release_cycle_diagnosis_20260906a/`。
原始 MCAP 保持不变。提取脚本在 Windows
`C:/Users/huang/.codex/visualizations/2026/09/06/01a07557-51c4-7440-93e9-b2f625df97a6/`
下的 `release_audit.py`、`release_motion.py`、`bottom.py`。

## 最小候选与预先声明的判据

不改变感知姿态界限、新鲜度、安全锁存或取消契约。为类型化 arm 路径添加
`place`、`retreat`，沿用 MoveIt 规划、轨迹验证、目标身份和 FJT 关联。
放回与撤离期间保留已存在的指垫/目标接触许可；下一次 approach 恢复禁止。
新的可选 wrapper 路径只在独立抓取返回成功后调用
`scripts/complete_release_cycle.py`；失败不再发送另一个张开动作。

直接反向下降 40 mm 会忽略已经倾斜、滑移的夹持姿态。此次候选仅将原抬升
目标下降 18 mm，姿态不变；历史三位置最低点距桌面约 17.786–18.242 mm。
这个历史数值用于预声明固定路径，不作新试验支撑证据。
撤离目标为原 approach 位姿。无执行规划中的放回夹爪状态为实测约 0.796 rad，
撤离为 1.5 rad。规划仍是现有代理场景规划，不证明携物接触动力学。

单次运行前固定以下门槛，失败不改门槛重试：

- 保留同帧双指接触、至少 20 mm 抬升、至少 0.5 s 保持的原独立抓取判定。
- 放回区域：原中心 X/Y 各 ±15 mm；50 mm 方块 OBB 最低点与桌面相差 ≤1 mm。
- 连续 0.5 s 桌面接触，平移漂移 ≤1 mm，姿态四元数点积绝对值 ≥0.99995。
- 支撑观测年龄 ≤200 ms，相邻样本间隔 ≤100 ms，位姿/接触样本差 ≤50 ms。
- 释放后夹爪为 1.5±0.08 rad，无指垫接触，仍满足支撑稳定判定。
- 撤离后上述条件持续成立，另需新鲜 TF 证明末端原点距方块中心 ≥80 mm。

这些观察只判定支撑/释放，不能给规划提供目标位姿或覆盖安全信号。
代码到达 `PLACE_RELEASE_RETREAT_COMPLETE` 也不替代原独立抓取结果。
未单次通过前，不执行连续三次；三连循环尚待验收。

## 运行与验证

- `release_cycle_plan_20260906a`：域 171，一次五段规划；五段均 PASS，
  `execution_attempted=false`，轨迹发布/ExecuteTrajectory/FJT 计数全部 0。
  每段规划 2 s，探针上限 180 s；清理 `REMAINING_COUNT=0`。
- `release_cycle_trial_20260906a`：域 172，一次控制位置候选，**STOPPED**。
  速度缩放 0.15、加速度 0.1；原抓取序列 90 s、物理判定 100 s；
  新增阶段总上限 160 s，每项支撑等待墙钟 20 s。失败保留并停止，不自动重试。
- Windows `scripts/check.ps1`：355 passed、3 skipped，结构检查及三个场景各
  100 次确定性回放通过。首次检查因旧静态测试只允许 lift 接触而失败；
  更新其阶段集合预期后完整重跑通过。
- WSL 零运动相关检查：9 passed；新增脚本导入/`--help`、shell 语法检查通过。
- Windows Python 未安装 Ruff，未安装新组件；不能声称 Ruff 通过。

### 唯一有界执行的结果

抓取通过：28.73498 mm 抬升、1679 个同帧双指接触样本、保持判定 VERIFIED。
`place|4` 在 41.415 s 获得相关门控/FJT 成功终态；之后支撑检查在墙钟
20 s 上限停止，`supported_before_release:continuous_support_evidence_timeout`。
**没有发送正常张开或撤离命令，没有启动连续三次循环。**
夹爪控制器停用/effort unclaimed 是回退清理；wrapper 的清理日志记录
`original_exit=24`，外层 WSL 命令返回 1，二者分别保留。
域 172 清理 `REMAINING_COUNT=0`，与域 171 的规划清理分开记录。

此次失败来自新增客户端的输入选择错误：它等待 ROS `/edgegrasp/table_contacts`，
但 `scene_bridge.yaml` 仅桥接方块位姿和 `/edgegrasp/target_cube_contacts`。
Gazebo 的 table sensor 主题不是 ROS 主题；录包请求一个主题也不证明该主题有发布者。
方块接触流本来就包含 table/target_cube 接触对。

已将客户端改为从同一已桥接方块接触消息提取桌面与指垫接触，未新增桥接，
未降低时间/支撑门槛。新增零节点回归核对客户端所有场景订阅都存在于当前桥接配置；
修正后 WSL 相关检查为 **10 passed**。修正版本**尚未重新执行仿真**。
同一个零节点订阅回归对保留的原运行脚本实际失败（多出的 table_contacts），
对修正版通过；没有通过重启或新桥接掩盖输入缺失。
原运行脚本已在实验目录恢复为 `complete_release_cycle_attempt_a.py`，SHA-256
与运行前清单一致：`41ba4e7004615a656290924269662aa5c62ae84b46b2e2aa97c3ffa1f1d4b4ba`。
不要把当前修正版当成本次实际运行源码。

离线诊断 `support_failure_audit.json`：40 s 后有 9041 条方块接触消息，
独立桌面 ROS 主题为 0 条；41.660 s 方块最低点为 0.179999823 m。
`support_corrected_offline.json` 使用严格录包接收时间重新评分：632 个样本进入评分，
其中 624 个含桌面接触；130 个时间不合格/不同步样本打断连续性，连续支撑通过数为 0。
录包接收时钟不等于原客户端回调时钟；这一结果不能证明修正版在线会通过，
也不能用几何落桌替代连续支撑验收。后续需先解决并验证时间对齐，而不是放宽新鲜度。

阶段一仍未完成：正常张开、释放后稳定性、撤离、就绪及连续三次均未验证。
按失败后暂停的约定，本轮没有第二次执行。
机器可读账本：[逐次结果](observations/2026-09-06-release-cycle-attempt.json)。

### 代表性回放

`C:/Users/huang/Documents/edgegrasp-replays/release-cycle-stopped-20260906a/`
包含 `replay.blend`、`replay.mp4`、`overview.png` 和 `gripper.png`。
775 帧、无变换缺口，6,199 个烘焙变换核对通过，未重算物理成功。
视频中的 VERIFIED 指原抓取判定；类型化释放退出 2，完整循环失败。
视频保留了本次放回及停止过程；不将它标记为正常释放成功演示。

生成命令：

```powershell
powershell -NoProfile -File scripts/run_blender_replay.ps1 `
  -Run /home/edgegrasp/ros2_ws/test_results/release_cycle_trial_20260906a `
  -World /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds/table_cube_candidate024_face_aligned.sdf `
  -Output C:/Users/huang/Documents/edgegrasp-replays/release-cycle-stopped-20260906a `
  -RenderVideo
```

精确启动命令分别保存在上述 Windows 外部目录的 `plan_release.sh` 和
`run_release.sh`；用 `wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/huang/.codex/visualizations/2026/09/06/01a07557-51c4-7440-93e9-b2f625df97a6/run_release.sh`
启动了唯一有界试验。这些脚本的输出路径是一次性的，不能原地重跑。

`accepted_goal_result_timeout_verified=false`、
`strong_move_group_request_id_correlation=false` 和所有硬件标志保持不变。

## 实际使用的启动脚本全文

以下保留原目录和任务 ID，供审计；已存在目录会拒绝，不能直接重复运行。

plan_release.sh

```bash
#!/bin/bash
set -eo pipefail
cd /mnt/c/Users/huang/.codex/worktrees/961f/edgegrasp-sim-private
export EDGEGRASP_RELEASE_CYCLE_PLAN=1
export EDGEGRASP_RGBD_OBSERVATION=/home/edgegrasp/ros2_ws/test_results/rgbd_opposite_capture_20260906a/first_accepted_estimate.json
export EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge
export EDGEGRASP_RGBD_VELOCITY_SCALING=0.15
bash scripts/run_grasp_candidate_plan_only.sh 171 /home/edgegrasp/ros2_ws/test_results/release_cycle_plan_20260906a release-cycle-plan-a so101_grasp_geometry_candidate024_face_aligned_q0p40.json 1 0.0 candidate012_control_mu1p0 0.0 so101_side_grasp_candidate024_face_aligned.json scene_candidate024_face_aligned.json table_cube_candidate024_face_aligned.sdf
```

run_release.sh

```bash
#!/bin/bash
set -eo pipefail
cd /mnt/c/Users/huang/.codex/worktrees/961f/edgegrasp-sim-private
export EDGEGRASP_RGBD_TRIAL=1
export EDGEGRASP_RELEASE_CYCLE=1
export EDGEGRASP_RGBD_ROTATION_BOUND_DEG=40.0
export EDGEGRASP_RGBD_CAMERA_HZ=30
export EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge
export EDGEGRASP_RGBD_VELOCITY_SCALING=0.15
export EDGEGRASP_RGBD_MOTION_MODE=measured_pad
export EDGEGRASP_RGBD_PLAN_RESULT=/home/edgegrasp/ros2_ws/test_results/release_cycle_plan_20260906a/plan_only_result.json
bash scripts/run_candidate005_contact_quality.sh 172 /home/edgegrasp/ros2_ws/test_results/release_cycle_trial_20260906a release-cycle-control-a so101_grasp_geometry_candidate024_face_aligned_q0p40.json 0.40 true candidate012_control_mu1p0 0.0 effort_pid_preload -0.02 so101_side_grasp_candidate024_face_aligned.json scene_candidate024_face_aligned.json table_cube_candidate024_face_aligned.sdf
```
