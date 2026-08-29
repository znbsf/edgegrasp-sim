#!/usr/bin/env python3
"""Validate Candidate012 pad friction in an actual ``gz sdf`` output."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from edgegrasp.contact_material import (
    load_pad_contact_material_contract,
    select_pad_contact_material_profile,
    validate_converted_pad_contact_material,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    PROJECT_ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_pad_contact_materials.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    contract = load_pad_contact_material_contract(args.contract)
    profile = select_pad_contact_material_profile(contract, args.profile)
    report = validate_converted_pad_contact_material(
        args.sdf.read_text(encoding="utf-8"), profile
    )
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
