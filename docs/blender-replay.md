# Blender 自动回放工作流

将已有本地 Gazebo 录包转换为可暂停、旋转视角、慢放的 Blender 场景。
不重新运行 ROS 节点、控制器或物理仿真，不向任何动作接口发送指令。

项目内 [Skill 入口](../.agents/skills/edgegrasp-blender-replay/SKILL.md) 可用
`$edgegrasp-blender-replay` 调用；清理步骤见 [清理 runbook](blender-replay-cleanup.md)。
Skill 放在项目 `.agents/skills`，遵循 [官方项目级 Skill 目录约定](https://learn.chatgpt.com/docs/build-skills)。
如果当前任务尚未显示新入口，可直接指定该 SKILL.md 路径读取使用。

```text
实验目录：MCAP + generated_proxy.urdf + trial.log + grasp_timeline.csv
                         + 匹配场景的 world.sdf
                                  ↓
                 export_blender_replay.sh / .py（WSL）
                 导出带源时间戳的 replay.json，复制模型
                                  ↓
                  build_blender_replay.py（Blender）
                   烘焙动画、识别框、接触框、阶段与原因
                                  ↓
                 verify_blender_replay.py（重新打开文件）
                     核对每帧变换、可见性与源数据
                                  ↓
                 replay.blend + 预览 PNG + 可选 MP4
```

## 下载和一条命令转换

Windows PowerShell，在仓库目录执行：

```powershell
# 官方免安装版；验证官方 SHA-256，不修改 PATH、注册表或系统运行库。
powershell -NoProfile -File scripts/install_blender_portable.ps1

# Output 必须是新目录。Run 和 World 是 WSL 内的绝对路径。
powershell -NoProfile -File scripts/run_blender_replay.ps1 `
  -Run /home/edgegrasp/ros2_ws/test_results/rgbd_measured_pad_trial_20260906b `
  -World /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds/table_cube_candidate024_face_aligned.sdf `
  -Output C:/Users/huang/Documents/edgegrasp-replays/my-new-replay `
  -RenderVideo
```

省略 `-RenderVideo` 会生成 `.blend`、两张预览及校验结果，省去视频渲染。
可用 `-Blender` 指定其他可用的 Blender 4.5 可执行文件，`-Distro` 指定 WSL 发行版。
现有 WSL 中需已有 `/opt/ros/jazzy`、`/home/edgegrasp/ros2_ws/install` 及感知模块的依赖。
导出器支持本项目固定世界原点、已有 TF、立方体位姿及时间线的试验；不承诺转换任意 ROS 录包。
模型使用录包旁保存的生成 URDF，STL 从其引用的现有本机路径复制到输出目录。
不往 Git 仓库复制第三方网格或原始大体积录包。

Blender 默认下载至 `C:/Users/huang/Documents/edgegrasp-tools`（一般形式为用户 Documents 下）。
本机 Codex 的 AppData 写入存在 MSIX 重定向，因此避免在那里部署需要并行程序集的可执行文件。
实际验证使用官方 Blender **4.5.13 LTS**，下载入口：
[官方 4.5 发行目录](https://download.blender.org/release/Blender4.5/)。

## 打开和观察

```powershell
& "$env:USERPROFILE/Documents/edgegrasp-tools/blender-4.5.13-windows-x64/blender.exe" `
  "C:/Users/huang/Documents/edgegrasp-replays/success/replay.blend"
```

- 空格播放/暂停，拖动底部时间线定位；所有动画已烘焙，无须启用 Python 自动执行。
- 鼠标中键旋转视角，滚轮缩放，小键盘 0 返回相机视图。
- 在 Outliner 选择 `Gripper close-up` 相机后，鼠标置于 3D 视图，Ctrl+小键盘 0 切换特写。
- 时间线保留动作阶段、安全停止及最近邻真值精度超限标记。
- 红色实心方块是录包中的 Gazebo 位姿；青色线框是 RGB-D 测量。
- 绿色线框是出现接触样本的**仿真碰撞代理区域**，不等同于外观网格表面。
- 左上角是阶段和原因；底部源时间随帧变化。右下角结果是整次实验的历史结果，不是当前帧的新判定。
- 也可直接播放 MP4；不用打开 Blender，但不能自由旋转镜头。

## 时间与证据边界

机械臂使用 `/tf`、`/tf_static` 的记录变换，方块使用 `/edgegrasp/target_cube_pose`。
统一按仿真源时间显示，默认 30 FPS。位置线性插值，姿态四元数最短弧插值；
只允许源样本间隔不超过 200 ms，不向记录范围外外推。
`replay.json` 保留每个显示帧的原始时间括号，插值不代表新增的传感器精度证据。
在共同覆盖区间中的缺口会隐藏相关几何并加标记；覆盖区间外的数据不扩展进动画。

RGB-D 框使用当前帧之前、不超过 100 ms 的最新原子测量；缺失时明确提示不可用。
接触框使用不超过 20 ms 的最近既往接触样本。30 FPS 可能漏掉短暂接触，
不得用动画帧数重新计算接触保持时间。独立物理结论沿用 `trial.log`，不重新判定成功。
阶段时间向后映射到下一显示帧，状态字符串保留录包接收时间，不冒充精确源时间。
真实硬件验证标志保持 false。

源 MCAP、URDF、网格、场景及日志 SHA-256 随导出保存。
场景合同摘要必须与试验结果一致；静态 TF 改变、父帧改变、坐标树断开、时钟重置会拒绝导出。
输入不齐全时保留部分输出供诊断，修复后使用新输出目录。

## 已制作的两个样例

- 成功抓取：`C:/Users/huang/Documents/edgegrasp-replays/success/replay.blend`。
  693 帧，约 23 秒；包括抓取、保持及之后的释放阶段。正常类型化释放仍失败，动画不改变该事实。
- 闭合前拒绝：`C:/Users/huang/Documents/edgegrasp-replays/failed-service/replay.blend`。
  496 帧；可见 `SAFE_STOP` 和 `gripper_dispatch_rejected:execute:execute_trajectory_unavailable`。

两个样例的共同覆盖区间均无变换缺口，包含识别框后分别核对 5,494 和 3,968 个烘焙变换，
并验证父子层级、静态变换及缺失数据时的可见性。
使用轻量 Workbench 渲染，不要求光线追踪显卡。本机已实际生成场景、PNG 和成功样例 MP4；
实时交互帧率尚未专门测量，不能把离线渲染速度当作播放帧率保证。

初次回放验证中，新增插值、过期和缺口回归 3 项通过；当时完整 Windows 检查
346 项通过、3 项明确跳过。加入清理盘点回归后的发布前检查为 348 项通过、3 项跳过。
运行后以输出目录的 `blender-verification.json` 为该次转换结果依据。
后续换实验只需替换 Run、World、Output；无需另开会话或手工摆放模型。
