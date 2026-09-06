"""Known-size red cube localization near a declared orientation from registered RGB-D only.

No scene file, object pose, or simulator truth enters this module. The calibrated
camera transform maps optical coordinates (+Z forward) into the planning frame.
Yaw and size are declared task constraints. Optional depth-derived rotation is
bounded to a declared 5, 20 or 40 degree range around the initial orientation.
"""

from dataclasses import dataclass
import math

import numpy as np


class ObservationRejected(ValueError):
    """An observation must not be published as an actionable target."""


@dataclass(frozen=True)
class CubeEstimate:
    center_m: tuple[float, float, float]
    source_timestamp_ns: int
    frame_id: str
    clock_domain: str
    clock_epoch: int
    pixels: int
    face_samples: tuple[int, int, int]
    surface_residual_p95_m: float
    orientation_deviation_rad: float = 0.0
    orientation_rows: tuple[tuple[float, float, float], ...] | None = None


def estimate_cube(
    rgb, depth_m, intrinsics, optical_to_planning, *,
    source_timestamp_ns: int, depth_timestamp_ns: int, info_timestamp_ns: int,
    now_ns: int, frame_id: str, clock_domain: str, clock_epoch: int,
    yaw_rad: float, max_rotation_deg: float = 0.0, size_m: float = 0.05, pixel_center_offset: float = 0.0,
) -> CubeEstimate:
    """Fit the three camera-facing planes of one fully visible static cube.

    Inputs must be exactly synchronized and rectified/registered. Reject missing
    faces, clipped images, invalid depth, extra red objects and poor cuboid fits.
    A fitted residual is a quality gate, not independent localization accuracy.
    """
    if (source_timestamp_ns <= 0 or source_timestamp_ns != depth_timestamp_ns
            or source_timestamp_ns != info_timestamp_ns):
        raise ObservationRejected("unsynchronized_source")
    if not 0 <= now_ns - source_timestamp_ns <= 100_000_000:
        raise ObservationRejected("source_not_fresh_for_admission")
    if not frame_id or clock_domain != "ros_sim" or clock_epoch < 0:
        raise ObservationRejected("invalid_identity")
    rgb = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=float)
    k = np.asarray(intrinsics, dtype=float)
    transform = np.asarray(optical_to_planning, dtype=float)
    if (depth.ndim != 2 or rgb.shape != (*depth.shape, 3)
            or rgb.dtype != np.uint8 or k.shape != (3, 3)
            or transform.shape != (4, 4)):
        raise ObservationRejected("invalid_layout")
    if (not np.isfinite(k).all() or not np.isfinite(transform).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or k[0, 1] != 0 or k[1, 0] != 0
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
            or not math.isclose(np.linalg.det(transform[:3, :3]), 1.0)
            or not math.isfinite(yaw_rad) or size_m != 0.05
            or pixel_center_offset not in (0.0, 0.5)
            or max_rotation_deg not in (0.0, 5.0, 20.0, 40.0)):
        raise ObservationRejected("invalid_calibration")
    colors = rgb.astype(float)
    red, green, blue = (colors[:, :, axis] for axis in range(3))
    mask = (red > 60) & (red > 1.8 * green) & (red > 1.8 * blue)
    if mask.sum() < 60:
        raise ObservationRejected("insufficient_red_pixels")
    if mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
        raise ObservationRejected("clipped_target")
    if not (np.isfinite(depth[mask]) & (depth[mask] > 0.05)
            & (depth[mask] < 1.5)).all():
        raise ObservationRejected("invalid_target_depth")
    # Remove silhouette pixels where rasterized color/depth edges can disagree.
    interior = mask.copy()
    interior[1:-1, 1:-1] &= (mask[:-2, 1:-1] & mask[2:, 1:-1]
                            & mask[1:-1, :-2] & mask[1:-1, 2:])
    rows, cols = np.nonzero(interior)
    z = depth[rows, cols]
    optical = np.column_stack(((cols + pixel_center_offset - k[0, 2]) * z / k[0, 0],
                               (rows + pixel_center_offset - k[1, 2]) * z / k[1, 1], z))
    points = optical @ transform[:3, :3].T + transform[:3, 3]
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    axes = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    deviation = 0.0
    if max_rotation_deg:
        # Estimate face normals from adjacent measured depth points. Neither
        # object truth nor an assumed gripper attachment enters this fit.
        grid = np.full((*depth.shape, 3), np.nan)
        grid[rows, cols] = points
        normals = np.cross(grid[1:-1, 2:] - grid[1:-1, 1:-1],
                           grid[2:, 1:-1] - grid[1:-1, 1:-1])
        lengths = np.linalg.norm(normals, axis=-1)
        valid = np.isfinite(lengths) & (lengths > 1e-12)
        normals = normals[valid] / lengths[valid, None]
        samples = grid[1:-1, 1:-1][valid]
        alignment = normals @ axes
        labels = np.argmax(np.abs(alignment), axis=1)
        fitted = []
        for axis in range(3):
            selected = (labels == axis) & (np.abs(alignment[:, axis]) > math.cos(math.radians(max_rotation_deg + 3)))
            directions = normals[selected] * np.sign(alignment[selected, axis])[:, None]
            if len(directions) < 12:
                raise ObservationRejected("missing_orientation_face")
            dominant = np.median(directions, axis=0)
            dominant /= np.linalg.norm(dominant)
            # Mixed normals at cube creases can be near a nominal axis while
            # belonging to another face. Retain the dominant planar direction.
            coherent = directions @ dominant > math.cos(math.radians(2))
            face = samples[selected][coherent]
            if len(face) < 12:
                raise ObservationRejected("missing_orientation_face")
            for _ in range(2):
                middle = np.mean(face, axis=0)
                _, _, vh = np.linalg.svd(face - middle, full_matrices=False)
                normal = vh[-1]
                residual = np.abs((face - middle) @ normal)
                face = face[residual <= 0.0002]
                if len(face) < 12:
                    raise ObservationRejected("inconsistent_orientation_face")
            if np.dot(normal, axes[:, axis]) < 0:
                normal = -normal
            fitted.append(normal)
        u, _, vh = np.linalg.svd(np.column_stack(fitted))
        rotated = u @ vh
        deviation = math.acos(float(np.clip((np.trace(axes.T @ rotated) - 1) / 2, -1, 1)))
        if np.linalg.det(rotated) < 0 or deviation > math.radians(max_rotation_deg):
            raise ObservationRejected("orientation_outside_declared_bound")
        axes = rotated
    local = points @ axes
    camera = transform[:3, 3] @ axes
    signs = np.sign(camera - np.median(local, axis=0))
    centers, counts = [], []
    for axis in range(3):
        values = local[:, axis]
        extreme = np.quantile(values, 0.98 if signs[axis] > 0 else 0.02)
        face = np.abs(values - extreme) <= 0.0005
        if face.sum() < max(12, len(points) * 0.04):
            raise ObservationRejected("missing_visible_face")
        centers.append(float(np.median(values[face]) - signs[axis] * size_m / 2))
        counts.append(int(face.sum()))
    delta = np.abs(local - centers)
    residual = np.min(np.abs(delta - size_m / 2), axis=1)
    p95 = float(np.quantile(residual, 0.95))
    if p95 > 0.0005 or np.any(delta > size_m / 2 + 0.001):
        raise ObservationRejected("inconsistent_cube_surfaces")
    if np.any(np.ptp(local, axis=0) < size_m * 0.7):
        raise ObservationRejected("incomplete_cube_extent")
    center = np.asarray(centers) @ axes.T
    return CubeEstimate(tuple(float(v) for v in center), source_timestamp_ns,
                        frame_id, clock_domain, clock_epoch, len(points),
                        tuple(counts), p95, deviation,
                        tuple(tuple(float(v) for v in row) for row in axes))
