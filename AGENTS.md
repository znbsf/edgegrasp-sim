# EdgeGrasp agent boundary

## Default safe validation

- On Windows, use `scripts/check.ps1`, Ruff, JSON parsing, and shell syntax
  checks. These do not start ROS, MoveGroup, Gazebo, a remote host, or hardware.
- For the cancellation/goal-response contract on Ubuntu/Jazzy, use only
  `bash scripts/run_injected_moveit_fail_closed.sh NEW_ARTIFACT_DIR`. It uses an
  in-process fake MoveGroup ActionServer; write artifacts outside the repository.
- Treat simulation, plan-only, injected, controller, and hardware evidence as
  separate claim classes. Never promote proxy simulation to hardware evidence.

## Historical negative evidence

- `docs/observations/2026-08-29-real-moveit-fail-closed-runtime.json` is
  read-only historical evidence of two upstream MoveGroup process failures.
- Do not recreate or execute `scripts/probe_real_moveit_fail_closed.py` or
  `scripts/run_real_moveit_fail_closed.sh`. They are intentionally absent from
  this repository.
- Do not inject `SIGSTOP` or `SIGCONT` into MoveGroup or reproduce the frozen P2
  process-interruption experiments. Use the in-process fake dependency instead.

## Process and external-system scope

- Existing experiment cleanup may target only exact process IDs or an explicitly
  selected `ROS_DOMAIN_ID` belonging to the scoped disposable run. Do not
  broaden cleanup to unrelated processes, domains, or hosts.
- Do not access a remote host or physical robot unless the user explicitly puts
  that system in scope for the current task.
- Preserve `accepted_goal_result_timeout_verified=false` and
  `strong_move_group_request_id_correlation=false` until direct correlated
  evidence exists.
