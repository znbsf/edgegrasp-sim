#!/usr/bin/env python3
"""Generate the two predeclared static translations, without launching anything."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

from edgegrasp.scene import load_scene_contract, render_gazebo_sdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new external directory")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if output.is_relative_to(root):
        parser.error("keep generated deployment files outside the repository")
    output.mkdir(parents=True, exist_ok=False)
    config = root / "ros_ws/src/edgegrasp_ros/config"
    scope = json.loads((config / "rgbd_static_scope.json").read_text())
    scene_base = json.loads((config / scope["scene_baseline"]).read_text())
    candidate_base = json.loads((config / scope["candidate_baseline"]).read_text())
    records = []
    for position in scope["positions"]:
        if position["id"] == "control":
            continue
        name = position["id"]
        if name not in ("x_minus_1mm", "x_plus_1mm"):
            raise ValueError("undeclared position")
        scene_name, world_name = f"scene_rgbd_{name}.json", f"table_cube_rgbd_{name}.sdf"
        candidate_name = f"so101_rgbd_{name}.json"
        scene, candidate = copy.deepcopy(scene_base), copy.deepcopy(candidate_base)
        cube = next(item for item in scene["objects"] if item["id"] == "target_cube")
        cube["pose_world"]["position_m"] = position["center_m"]
        scene["experiment"] = dict(candidate=name, changed_variable="predeclared_x_translation",
                                    baseline_scene=scope["scene_baseline"],
                                    target_cube_yaw_rad=scope["declared_yaw_rad"])
        candidate.update(name=f"edgegrasp_rgbd_{name}", scene_config_filename=scene_name,
                         world_filename=world_name)
        candidate["target"]["center_m"] = position["center_m"]
        candidate["changed_variable"] = dict(name="predeclared_x_translation", scope="rgbd_static_scope.json")
        candidate["claim_boundary"] = ["Predeclared static translation. Requires its own RGB-D observation, plan and independent physics evidence."]
        for filename, data in ((scene_name, scene), (candidate_name, candidate)):
            (output / filename).write_text(json.dumps(data, indent=2) + "\n")
        contract = load_scene_contract(output / scene_name)
        (output / world_name).write_text(render_gazebo_sdf(contract))
        records.append(dict(position_id=name, center_m=position["center_m"],
            scene_digest=contract.digest, files={filename: hashlib.sha256((output / filename).read_bytes()).hexdigest()
                for filename in (scene_name, candidate_name, world_name)}))
    (output / "manifest.json").write_text(json.dumps(dict(
        claim="generated_inputs_only_no_motion_or_success", positions=records), indent=2) + "\n")
    print(json.dumps(records))


if __name__ == "__main__":
    main()
