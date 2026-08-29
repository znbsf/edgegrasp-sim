#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root/src${PYTHONPATH:+:$PYTHONPATH}"
python_bin="${PYTHON:-python3}"

"$python_bin" -m compileall -q src tests ros_ws/src
"$python_bin" -m pytest
"$python_bin" scripts/validate_project.py
"$python_bin" -m edgegrasp replay --runs "${1:-100}"
