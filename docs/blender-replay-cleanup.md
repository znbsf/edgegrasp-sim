# Blender 回放清理 runbook

用途：回收本次下载副本、失败安装和已被最终回放替代的中间输出。
不清理原始实验录包，也不因为实验失败而删除失败证据。
项目 Skill 入口为 [edgegrasp-blender-replay](../.agents/skills/edgegrasp-blender-replay/SKILL.md)。

## 1. 先盘点

检查 [示例清理计划](blender-cleanup-plan.example.json) 中的保护路径和候选路径。
它对应 2026-09-06 这次本机工作，不是对其他机器、版本或整个 AppData 的删除授权。
以后新任务应在外部工作目录创建自己的计划，保留同一 JSON 结构，填入精确路径和用途。

```powershell
powershell -NoProfile -File scripts/inspect_blender_cleanup.ps1 `
  -Plan docs/blender-cleanup-plan.example.json `
  -Output C:/Users/huang/Documents/edgegrasp-replays/cleanup-inventory-new.json
```

Output 必须是新文件，父目录应已存在。不指定 Output 则只打印 JSON。
脚本只读取指定树，返回每个候选的字节数、文件数和 blockers，**没有删除模式**。
保护路径的父目录与子目录都被拦截；目录链接不跟随；含 MCAP/DB3 的候选会被标记。
`review_candidate=true` 仅表示可以继续人工/代理审核，不表示已获授权或已经删除。
读失败直接报错，不能把部分统计冒充可回收总量。

## 2. 核对保留项

| 必须保留 | 核对方式 |
|---|---|
| 可用 Blender 目录 | 当前 executable 的 `--version` 成功；不是待清理的安装副本 |
| 成功和失败最终回放 | `.blend`、`replay.json`、验证 JSON 和已有视频/预览均存在 |
| 下载来源与校验信息 | 保留 `.sha256` 和 `blender-install.json`；不需要保留重复 ZIP |
| 原始实验 | `/home/edgegrasp/ros2_ws/test_results/`，不进入回放清理范围 |
| 代码、文档、验证收据 | 仓库文件与清理盘点报告保留 |

检查候选目录的子项是否仍全部属于本任务；已增加其他内容时先缩小候选范围。
目录名中的 `control`、`temp`、`failed` 不能单独作为可删除依据。
对于未来任务，将所有最终输出、当前仓库和原始数据路径补进 protected。

## 3. 执行或停止

用户已明确要求清理这份具体清单时不重复索要授权；只要求盘点时不删除。
在允许执行的环境中，重新核对绝对路径、保护路径重叠、目录链接、文件变化与正在使用的进程。
使用原生 PowerShell 的 `Remove-Item -LiteralPath`，只对已审核的精确候选执行，
并保留错误；不拼接其他 shell 的删除命令，不终止正在使用的 Blender。

如果自动审批/工具策略拒绝，停止删除并报告原始原因。不要改用 WSL、另一种 shell、
脚本包装、回收站 API 或定时任务重试同一被拒操作。Skill 和用户授权不改变工具权限。
如实区分“已列入清单”和“已删除”，保留可供之后审核的盘点报告。

2026-09-06 的真实状态：约 4.22 GiB 冗余项已盘点；此前删除请求返回
`blocked by policy`，没有更具体原因。**截至本 runbook 建立时，未释放这部分空间。**
本次只建立和验证盘点流程，不把编写 Skill 当作重新执行受阻删除的理由。

## 4. 验证与重复使用

允许删除后逐项检查路径不存在，并复核上表保留项。报告本次实际删除的字节数，
同时注明它是文件长度之和，不等于精确的磁盘可用空间增量。
对删除失败或已变化的条目重新盘点，不继续扩大清理范围。

回放复现见 [Blender 自动回放工作流](blender-replay.md)。先检查现有 Blender，
只在缺失时运行下载脚本，避免删掉 ZIP 后再次调用安装脚本又下载一份。
本机便携版安装路径放在 Documents，避免 Codex 的 AppData 重定向问题。
