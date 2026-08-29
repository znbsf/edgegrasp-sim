#!/usr/bin/env python3
"""Generate a Gazebo SDF world from the shared EdgeGrasp scene contract."""

from __future__ import annotations

import argparse
from pathlib import Path

from edgegrasp.scene import load_scene_contract, render_gazebo_sdf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = (
    PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_ros" / "config" / "scene.json"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    contract = load_scene_contract(args.config)
    output = render_gazebo_sdf(contract)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    print(f"SCENE_CONTRACT_SHA256={contract.digest}")
    print(f"OUTPUT={args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
