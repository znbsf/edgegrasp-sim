# 固定场景 RGB-D 抓取验证结果

三个预先声明的位置均完成了 RGB-D 定位、类型化规划/执行与独立物理判定。
范围仅为本机 Gazebo 固定桌面、一个初始静止的 50 mm 红色方块，以及以下三个位置。

| 位置 | 相对控制位置 | 保持抬升 | 同帧双指接触样本 | 独立物理结果 |
|---|---:|---:|---:|---|
| control | 0 | 28.8468 mm | 1612 | contact_lift_retention_verified |
| x_minus_1mm | X −1 mm | 28.9127 mm | 1642 | contact_lift_retention_verified |
| x_plus_1mm | X +1 mm | 28.6782 mm | 1715 | contact_lift_retention_verified |

判定要求仍为同一样本中的两个远端指垫接触、至少 20 mm 抬升及至少 0.5 s 保持。
序列 COMPLETE 与独立物理 VERIFIED 分别记录，不相互替代。

## 实现与证据边界

感知只读注册 RGB、深度、相机标定及相机 TF。它保留三输入源时间戳，发布原子的
中心、姿态、目标身份和时钟元数据；仿真真值仅用于独立评价。过期、错误身份、
损坏姿态、输入冲突、深度异常和不充分可见面均有拒绝路径。

传感器使用固定对侧视角、30 Hz、原始 424×240 分辨率。像素中心采用已与传感器
点云交叉核对的 0.5 偏移；感知发布前要求源年龄不超过 100 ms。序列层使用原有
200 ms 源新鲜度上限。闭合前维持静态目标检查；抬升时，实测方块 OBB 与同源时间
TF 下的固定指垫进行几何一致性检查，允许的归一化分离轴间隙为 5 mm。
这个几何条件不证明实际接触；双指接触、抬升与保持仍由独立观察器判定。

两个平移位置的原始固定夹爪姿态均被 NO_IK_SOLUTION 拒绝，执行数为 0。
根据关节几何和实测中心推导的偏航调整分别为 +0.145323°、−0.144329°，
通过了各自三段无执行规划，再用于上述成功试验。目标位置与方块偏航未改变，
原失败候选和结果均保留。

343 项 Windows 测试通过、3 项明确跳过；WSL RGB-D/ROS 检查 26 项通过。
另有物理观察器 3 项、安全监视器 8 项回归通过，三个场景各 100 次确定性回放通过。
源基线 842c42ca5ee8fd849e1d6116b4983d44afcb15c3 是当前分支祖先。
代码、测试及结果文档已随 `cf1618e` 合入并推送至主分支。原始大体积录包保留在下述 WSL 实验目录，Git 中保存其清单与证据摘要。

## 保留的限制与失败

- 静态位置资格检查全部有效样本通过 0.25 mm 预算，启动期拒绝未删除。
  三个成功录包共 1821 条原子观测，其中 1817 条通过最近邻真值比较；4 条超限保留，
  包括控制试验闭合阶段的 1 条与释放后的 3 条。相邻时间插值仅是补充，不能冒充完全同源真值。
- 三个成功试验的类型化释放均未成功，随后通过已有的控制器停用、释放 effort 所有权路径清理。
  执行成功与释放回退分别报告；最终相关仿真域无剩余进程。
- 一次新增回调的锁顺序错误造成超时，已修正，并由零运动回归检查确认旧顺序失败、新顺序通过。
- X−1 mm 的首次执行在闭合前因执行服务不可用停止。其底层发现问题仍未证实。
  新增的最长 50 ms 就绪等待不发送或重试动作，等待后重新检查源年龄；成功复测未触发该分支。
- 仅有本地仿真证据。硬件、真实 MoveGroup 超时契约及强请求 ID 关联标志均保持 false。

## 复现

依赖现有 Ubuntu 24.04 / ROS Jazzy 工作区、固定 SO-101 源与 NumPy 感知依赖。
Windows 检查：`powershell -NoProfile -File scripts/check.ps1`。
WSL 在本仓库目录执行：`bash scripts/check_rgbd.sh`。

以下使用已保留且匹配的无执行规划；每次必须使用新的外部输出目录和任务 ID。
程序仍会读取新的实时 RGB-D 观测，并重新经过类型化规划/轨迹边界。

```bash
# 在 WSL 的本仓库目录执行；选择 control、x_minus_1mm 或 x_plus_1mm。
case_id=x_minus_1mm
case "$case_id" in
  control)
    candidate=so101_side_grasp_candidate024_face_aligned.json
    scene=scene_candidate024_face_aligned.json
    world=table_cube_candidate024_face_aligned.sdf
    plan=rgbd_v015_plan_20260906a ;;
  x_minus_1mm|x_plus_1mm)
    candidate="so101_rgbd_${case_id}_yaw.json"
    scene="scene_rgbd_${case_id}.json"
    world="table_cube_rgbd_${case_id}.sdf"
    if [ "$case_id" = x_minus_1mm ]; then
      plan=rgbd_x_minus_yaw_plan_20260906a
    else
      plan=rgbd_x_plus_yaw_plan_20260906a
    fi ;;
  *) exit 2 ;;
esac
export EDGEGRASP_RGBD_TRIAL=1
export EDGEGRASP_RGBD_ROTATION_BOUND_DEG=40.0
export EDGEGRASP_RGBD_CAMERA_HZ=30
export EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge
export EDGEGRASP_RGBD_VELOCITY_SCALING=0.15
export EDGEGRASP_RGBD_MOTION_MODE=measured_pad
export EDGEGRASP_RGBD_PLAN_RESULT="/home/edgegrasp/ros2_ws/test_results/$plan/plan_only_result.json"
bash scripts/run_candidate005_contact_quality.sh 170 \
  /home/edgegrasp/ros2_ws/test_results/NEW_UNIQUE_RUN NEW_UNIQUE_TASK \
  so101_grasp_geometry_candidate024_face_aligned_q0p40.json 0.40 true \
  candidate012_control_mu1p0 0.0 effort_pid_preload -0.02 \
  "$candidate" "$scene" "$world"
```

重新制作观测与规划的流程、相机选择、配置生成及完整开发历史见
[rgbd-static-grasp.md](rgbd-static-grasp.md)。

## 完整记录

- [最终逐位置验证](observations/2026-09-06-rgbd-final-validation.json)：完整物理结果、实时来源审计、精度失败和清理结果。
- [41 个保留实验目录及文件清单](observations/2026-09-06-rgbd-artifact-inventory.json)：包含失败、成功与初步分析文件，不把初步文件自动视为合格证据。
- [按全部运行元数据核对的最终进程状态](observations/2026-09-06-rgbd-final-process-check.json)。
- [两处平移的原始拒绝及调整后规划](observations/2026-09-06-rgbd-translations-yaw-plan-only.json)。

原始录包、图像和日志位于 `/home/edgegrasp/ros2_ws/test_results/`，清单列出精确路径。
