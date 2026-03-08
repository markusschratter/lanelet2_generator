"""Geometry: pose to boundary lines and segment splitting."""

import numpy as np


def pose2line(pose_array, width=3.0, offset=(0, 0, 0)):
    """
    Compute left, right, center boundary lines from poses.

    Uses vectorized quaternion rotation (rotation matrix form) so no
    tf_transformations / transforms3d dependency is needed.

    Args:
        pose_array: (N, 7) [x,y,z,qx,qy,qz,qw]
        width: Lane width [m]
        offset: (x,y,z) offset from centerline

    Returns:
        left, right, center: (N, 3) ndarrays of 3D points
    """
    pose_array = np.asarray(pose_array, dtype=np.float64)
    if pose_array.ndim != 2 or pose_array.shape[1] < 7:
        raise ValueError(
            f"pose_array must be (N, 7) [x,y,z,qx,qy,qz,qw], got shape {pose_array.shape}"
        )

    pos = pose_array[:, :3]
    qx, qy, qz, qw = pose_array[:, 3], pose_array[:, 4], pose_array[:, 5], pose_array[:, 6]

    hw = width / 2.0
    # Rotate [0, hw, 0] by each quaternion using the rotation matrix second column:
    # R[:,1] = [2(qx*qy - qw*qz), 1 - 2(qx^2 + qz^2), 2(qy*qz + qw*qx)]
    lat = np.column_stack([
        hw * 2.0 * (qx * qy - qw * qz),
        hw * (1.0 - 2.0 * (qx ** 2 + qz ** 2)),
        hw * 2.0 * (qy * qz + qw * qx),
    ])

    ox, oy, oz = float(offset[0]), float(offset[1]), float(offset[2])
    if ox == 0.0 and oy == 0.0 and oz == 0.0:
        off = 0.0
    else:
        # Full rotation matrix applied to offset vector
        off = np.column_stack([
            ox * (1 - 2 * (qy ** 2 + qz ** 2)) + oy * 2 * (qx * qy - qw * qz) + oz * 2 * (qx * qz + qw * qy),
            ox * 2 * (qx * qy + qw * qz) + oy * (1 - 2 * (qx ** 2 + qz ** 2)) + oz * 2 * (qy * qz - qw * qx),
            ox * 2 * (qx * qz - qw * qy) + oy * 2 * (qy * qz + qw * qx) + oz * (1 - 2 * (qx ** 2 + qy ** 2)),
        ])

    left = pos + off + lat
    right = pos + off - lat
    center = pos + off
    return left, right, center


def _yaw_from_quat(pose_array):
    """Vectorized yaw extraction from quaternion [x,y,z,w] columns."""
    qx, qy, qz, qw = pose_array[:, 3], pose_array[:, 4], pose_array[:, 5], pose_array[:, 6]
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy ** 2 + qz ** 2))


def _angle_diff(a, b):
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def split_segments(
    center,
    pose_array,
    split_distance=None,
    max_direction_change_deg=None,
    direction_change_window_m=None,
):
    """
    Compute segment indices for splitting a path.

    Splits at: distance intervals (split_distance), and sharp direction changes
    (max_direction_change_deg within direction_change_window_m).

    Uses precomputed cumulative distances for O(n) complexity instead of
    the naive O(n^2) backward search.

    Args:
        center: (N, 3) ndarray or list of center points
        pose_array: (N, 7) poses
        split_distance: Split every M meters
        max_direction_change_deg: Split when direction changes more than this
        direction_change_window_m: Within this distance [m]

    Returns:
        List of (start, end) index pairs with overlapping boundaries
    """
    pose_array = np.asarray(pose_array, dtype=np.float64)
    center_arr = np.asarray(center, dtype=np.float64)
    n = len(center_arr)
    if n == 0:
        return []
    if n == 1:
        return [(0, 1)]

    # Precompute cumulative path distances — O(n) once, used by all split logic
    diffs = np.linalg.norm(np.diff(center_arr, axis=0), axis=1)
    cum_dist = np.empty(n)
    cum_dist[0] = 0.0
    np.cumsum(diffs, out=cum_dist[1:])

    split_indices = [0]

    if split_distance is not None and split_distance > 0:
        last_split_dist = 0.0
        for i in range(1, n):
            if cum_dist[i] - last_split_dist >= split_distance:
                split_indices.append(i)
                last_split_dist = cum_dist[i]

    if (
        max_direction_change_deg is not None
        and direction_change_window_m is not None
        and max_direction_change_deg > 0
        and direction_change_window_m > 0
    ):
        yaws = _yaw_from_quat(pose_array)
        max_rad = np.deg2rad(max_direction_change_deg)
        window_m = direction_change_window_m
        last_split_cum = -1.0

        for i in range(1, n):
            if last_split_cum >= 0.0 and cum_dist[i] - last_split_cum < window_m:
                continue

            # Binary search for point j that is ~window_m back from i
            target = cum_dist[i] - window_m
            if target <= 0.0:
                if cum_dist[i] < window_m * 0.3:
                    continue
                j = 0
            else:
                j = int(np.searchsorted(cum_dist, target, side="right")) - 1
                j = max(j, 0)

            delta = abs(_angle_diff(yaws[i], yaws[j]))
            if delta >= max_rad:
                split_indices.append(j)
                split_indices.append(i)
                last_split_cum = cum_dist[i]

    split_indices = sorted(set(split_indices))
    if split_indices[-1] != n:
        split_indices.append(n)

    segments = []
    for k in range(len(split_indices) - 1):
        start = split_indices[k]
        next_start = split_indices[k + 1]
        end = next_start + 1 if k + 1 < len(split_indices) - 1 else split_indices[-1]
        segments.append((start, end))
    return segments
