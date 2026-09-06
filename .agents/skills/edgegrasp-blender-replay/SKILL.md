---
name: edgegrasp-blender-replay
description: Convert existing EdgeGrasp Gazebo recordings into verified Blender replay scenes and videos, or inventory and clean their temporary downloads and intermediate outputs. Use for this project's recorded grasp visualization and replay storage maintenance, not for running new robot experiments or general Blender modeling.
---

# EdgeGrasp Blender replay

Resolve the repository root as three parents above this skill directory. Read its
`AGENTS.md` and [replay runbook](../../../docs/blender-replay.md). Reuse the scripts
in the repository; do not copy a second implementation into this skill.

## Replay an existing experiment

- Identify the actual recording directory, matching world SDF, and a **new** output
  directory. Reuse paths already supplied in the conversation. Do not silently
  substitute a successful experiment for the requested failed experiment.
- Check the existing portable Blender before downloading. Prefer a normal
  Documents directory on this Windows host: AppData writes from Codex were
  redirected by MSIX and the extracted executable could not locate dependencies.
  Use `scripts/install_blender_portable.ps1` only if needed, within authorization.
- Run `scripts/run_blender_replay.ps1` with Run, World and Output; add RenderVideo
  when a video is wanted. The runbook gives exact commands and dependencies.
- Review `blender-verification.json` and the rendered overview/close-up. A successful
  export alone is not a successful grasp. Keep source time brackets, missing data,
  historical safety stops and typed-release failures visible.
- Deliver the .blend, requested video and runbook link. The scene is baked and
  requires no script auto-run. Preserve recorded-simulation-only claims; do not
  start ROS nodes, Gazebo, controllers or hardware to make a replay.

## Inspect or clean replay storage

Read [cleanup procedure](../../../docs/blender-replay-cleanup.md) only for this mode.

- Run `scripts/inspect_blender_cleanup.ps1` against a reviewed plan. This script
  only inventories; it never deletes. Validate the protected paths and each exact
  candidate's provenance before treating any row as a deletion candidate.
- Preserve the usable Blender installation, final success **and failure** replays,
  source experiment recordings, source code, checksum manifests and validation
  receipts. An old filename or a failed experiment is not grounds for deletion.
- A cleanup request authorizes only its scoped redundant files. Do not ask again
  when the current conversation already authorizes those exact items. If authority
  is missing, finish the path/size inventory first and ask about the concrete list.
- When deletion is allowed, recheck exact absolute targets, protected-path overlap,
  links, changed contents and open processes. Use native PowerShell literal-path
  operations within that scope. Never close Blender or broaden to an app cache root.
- If the tool rejects deletion, stop that operation and report the rejection.
  Do not retry through WSL, another shell, a generated script, a scheduled job or a
  different deletion API. A Skill does not change permissions. Preserve the inventory
  and say that space has **not** been reclaimed.
- After allowed deletion, verify absence and retained files, then report measured
  removed bytes separately from remaining candidates. Do not claim success from
  an intended cleanup plan. Do not commit or push as part of this skill unless asked.
