# RGB-D 抓取回放演示

`rgbd-grasp-replay.mp4` 是已有 Gazebo 记录的 Blender 烘焙回放，不是实机录像，
也不是重新运行物理仿真的结果。来源实验为 `rgbd_measured_pad_trial_20260906b`。
完整源文件与回放 SHA-256 见 [回放验证收据](../observations/2026-09-06-blender-replay-validation.json)。
该次抓取的独立物理判定通过，正常类型化释放失败；视频保留全部 693 帧、23.10 秒。

生成方式见 [Blender 工作流](../blender-replay.md)。从其输出目录压缩：

```powershell
ffmpeg -n -i replay.mp4 -an -vf scale=768:-2 -c:v libx264 -preset slow -crf 30 -pix_fmt yuv420p -movflags +faststart rgbd-grasp-replay.mp4
ffmpeg -n -ss 17 -i rgbd-grasp-replay.mp4 -frames:v 1 -q:v 3 rgbd-grasp-replay.jpg
ffmpeg -v error -i rgbd-grasp-replay.mp4 -f null -
```

视频源为 935,609 字节，压缩后为 92,521 字节；JPEG 预览为 18,090 字节。
有损压缩仅用于演示，不用于量测定位误差或接触时间。需要逐帧诊断时使用完整 .blend 和源数据。
机器人外观来自固定版本的 SO-101 模型；相关上游为
[adoodevv/so101_ros2](https://github.com/adoodevv/so101_ros2)（BSD-3-Clause）与
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)（Apache-2.0），
版本和许可范围见 [上游清单](../upstream-manifest.json)。本目录不包含原始网格或 Blender 安装包。
