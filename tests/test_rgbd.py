"""Independent ray/box image fixtures; truth is used only by assertions."""

import pytest

np = pytest.importorskip("numpy", reason="install the perception extra for RGB-D tests")

from edgegrasp.rgbd import ObservationRejected, estimate_cube  # noqa: E402


def rendered_cube(center=(0.24, 0.13, 0.205), yaw=0.65, pixel_offset=0.0, tilt_deg=0.0):
    width, height = 424, 240
    k = np.array([[225., 0, width / 2], [0, 225., height / 2], [0, 0, 1]])
    origin = np.array([0.22, -0.29, 0.5])
    forward = np.array([0., np.cos(np.deg2rad(25)), -np.sin(np.deg2rad(25))])
    right = np.array([1., 0., 0.])
    down = np.cross(forward, right)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((right, down, forward))
    transform[:3, 3] = origin
    v, u = np.indices((height, width))
    rays = np.stack(((u + pixel_offset - k[0, 2]) / k[0, 0],
                     (v + pixel_offset - k[1, 2]) / k[1, 1], np.ones_like(u)), axis=-1)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    t = np.deg2rad(tilt_deg)
    rotation = rotation @ np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])
    local_origin = (origin - center) @ rotation
    local_rays = rays @ transform[:3, :3].T @ rotation
    with np.errstate(divide="ignore", invalid="ignore"):
        a = (-0.025 - local_origin) / local_rays
        b = (0.025 - local_origin) / local_rays
    near = np.minimum(a, b).max(axis=-1)
    far = np.maximum(a, b).min(axis=-1)
    hit = (near > 0) & (near < far)
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[hit] = [210, 30, 20]
    depth = np.where(hit, near, np.nan)
    return dict(rgb=rgb, depth_m=depth, intrinsics=k,
                optical_to_planning=transform, source_timestamp_ns=1_000_000_000,
                depth_timestamp_ns=1_000_000_000, info_timestamp_ns=1_000_000_000,
                now_ns=1_050_000_000, frame_id="base_link", clock_domain="ros_sim",
                clock_epoch=2, yaw_rad=0.65, pixel_center_offset=pixel_offset)


@pytest.mark.parametrize("center", [(0.24, 0.13, 0.205), (0.245, 0.125, 0.205)])
def test_center_from_visible_planes_and_preserved_provenance(center):
    result = estimate_cube(**rendered_cube(center))
    assert np.max(np.abs(np.array(result.center_m) - center)) < 1e-6
    assert result.source_timestamp_ns == 1_000_000_000
    assert result.clock_epoch == 2
    assert result.frame_id == "base_link"


@pytest.mark.parametrize(("changes", "reason"), [
    ({"now_ns": 1_100_000_001}, "source_not_fresh"),
    ({"now_ns": 999_999_999}, "source_not_fresh"),
    ({"depth_timestamp_ns": 999_999_999}, "unsynchronized"),
    ({"clock_domain": "ros_system"}, "invalid_identity"),
    ({"yaw_rad": 0.9}, "inconsistent_cube|missing_visible"),
])
def test_untrusted_observation_rejected(changes, reason):
    inputs = rendered_cube()
    inputs.update(changes)
    with pytest.raises(ObservationRejected, match=reason):
        estimate_cube(**inputs)


def test_no_target_and_invalid_depth_rejected():
    inputs = rendered_cube()
    inputs["depth_m"][:] = np.nan
    with pytest.raises(ObservationRejected, match="invalid_target_depth"):
        estimate_cube(**inputs)


def test_gazebo_pixel_centers_preserve_submillimetre_geometry():
    inputs = rendered_cube(pixel_offset=0.5)
    result = estimate_cube(**inputs)
    assert np.linalg.norm(np.asarray(result.center_m) - [0.24, 0.13, 0.205]) < 1e-6
    inputs["pixel_center_offset"] = 0.0
    wrong = estimate_cube(**inputs)
    assert np.linalg.norm(np.asarray(wrong.center_m) - [0.24, 0.13, 0.205]) > 0.001
    inputs["rgb"][:] = 0
    with pytest.raises(ObservationRejected, match="insufficient_red"):
        estimate_cube(**inputs)


@pytest.mark.parametrize("tilt", [-3.0, 2.0])
def test_bounded_orientation_is_measured_from_depth(tilt):
    inputs = rendered_cube(tilt_deg=tilt)
    estimate = estimate_cube(**inputs, max_rotation_deg=5.0)
    assert np.linalg.norm(np.asarray(estimate.center_m) - [0.24, 0.13, 0.205]) < 0.00025
    assert abs(estimate.orientation_deviation_rad - np.deg2rad(abs(tilt))) < 0.001
    with pytest.raises(ObservationRejected):
        estimate_cube(**rendered_cube(tilt_deg=10), max_rotation_deg=5.0)


@pytest.mark.parametrize("tilt", [-12.0, 15.0])
def test_larger_rotation_requires_depth_supported_center(tilt):
    inputs = rendered_cube(tilt_deg=tilt)
    result = estimate_cube(**inputs, max_rotation_deg=20.0)
    assert np.linalg.norm(np.asarray(result.center_m) - [0.24, 0.13, 0.205]) < 0.00025
    assert abs(result.orientation_deviation_rad - np.deg2rad(abs(tilt))) < 0.001
    with pytest.raises(ObservationRejected):
        estimate_cube(**rendered_cube(tilt_deg=30), max_rotation_deg=20.0)


@pytest.mark.parametrize("tilt", [-30.0, 30.0])
def test_post_contact_rotation_requires_three_measured_faces(tilt):
    result = estimate_cube(**rendered_cube(tilt_deg=tilt), max_rotation_deg=40.0)
    assert np.linalg.norm(np.asarray(result.center_m) - [.24, .13, .205]) < .00025
    assert abs(result.orientation_deviation_rad - np.deg2rad(abs(tilt))) < .001
    axes = np.asarray(result.orientation_rows)
    assert np.allclose(axes.T @ axes, np.eye(3)) and np.linalg.det(axes) > .999
    c, s = np.cos(.65), np.sin(.65)
    t = np.deg2rad(tilt)
    expected = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.array(
        [[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])
    angle_error = np.arccos(np.clip((np.trace(expected.T @ axes) - 1) / 2, -1, 1))
    assert angle_error < .001  # Rotation error, in radians; no elementwise exact-fit claim.
    with pytest.raises(ObservationRejected):
        estimate_cube(**rendered_cube(tilt_deg=tilt), max_rotation_deg=20.0)


def test_bounded_planar_yaw_derivation_preserves_lateral_constraint():
    import runpy
    from pathlib import Path
    module = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts/derive_rgbd_yaw_candidate.py"))
    derive, rotate = module["derive_delta"], module["rz"]
    normal = np.array([-np.sin(.65), np.cos(.65)])
    baseline = np.array([.202249, .129587])
    for dx in (-.001, .001):
        observed = baseline + [dx, 0]
        delta = derive(normal, baseline, observed)
        aligned = (rotate(-delta) @ [*observed, 0])[:2]
        assert abs(normal @ aligned - normal @ baseline) < 1e-12
        assert abs(delta) < np.deg2rad(.5)
    with pytest.raises(ValueError, match="half-degree"):
        derive(normal, baseline, baseline + [.01, 0])
