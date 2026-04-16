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


def smooth_path(pose_array, window=0):
    """
    Densify poses with an interpolating cubic curve that preserves samples.

    The generated path passes through every original pose position exactly and
    only inserts smooth intermediate points between them.

    Args:
        pose_array: (N, 7) [x,y,z,qx,qy,qz,qw]
        window: Number of subdivisions per original segment. Values <= 1
            disable smoothing.

    Returns:
        Smoothed/densified (M, 7) pose array
    """
    pose_array = np.asarray(pose_array, dtype=np.float64)
    if pose_array.ndim != 2 or pose_array.shape[1] < 7:
        raise ValueError(
            f"pose_array must be (N, 7) [x,y,z,qx,qy,qz,qw], got shape {pose_array.shape}"
        )

    n = len(pose_array)
    if n < 3 or window is None or window <= 1:
        return pose_array.copy()

    subdivisions = int(window)
    if subdivisions <= 1:
        return pose_array.copy()

    positions = pose_array[:, :3]
    out_positions = [positions[0].copy()]
    tangents = []

    for i in range(n - 1):
        p0 = positions[i - 1] if i > 0 else positions[i]
        p1 = positions[i]
        p2 = positions[i + 1]
        p3 = positions[i + 2] if i + 2 < n else positions[i + 1]

        m1 = 0.5 * (p2 - p0)
        m2 = 0.5 * (p3 - p1)

        for step in range(1, subdivisions + 1):
            t = step / subdivisions
            t2 = t * t
            t3 = t2 * t

            h00 = 2.0 * t3 - 3.0 * t2 + 1.0
            h10 = t3 - 2.0 * t2 + t
            h01 = -2.0 * t3 + 3.0 * t2
            h11 = t3 - t2
            point = h00 * p1 + h10 * m1 + h01 * p2 + h11 * m2

            dh00 = 6.0 * t2 - 6.0 * t
            dh10 = 3.0 * t2 - 4.0 * t + 1.0
            dh01 = -6.0 * t2 + 6.0 * t
            dh11 = 3.0 * t2 - 2.0 * t
            tangent = dh00 * p1 + dh10 * m1 + dh01 * p2 + dh11 * m2

            out_positions.append(point)
            tangents.append(tangent)

    out_positions = np.asarray(out_positions, dtype=np.float64)
    out_count = len(out_positions)

    yaws = np.empty(out_count, dtype=np.float64)
    if tangents:
        tangent_arr = np.asarray(tangents, dtype=np.float64)
        yaws[1:] = np.arctan2(tangent_arr[:, 1], tangent_arr[:, 0])
        yaws[0] = yaws[1]
    else:
        deltas = np.diff(out_positions[:, :2], axis=0)
        seg_yaws = np.arctan2(deltas[:, 1], deltas[:, 0])
        yaws[0] = seg_yaws[0]
        yaws[1:] = seg_yaws

    smoothed = np.zeros((out_count, 7), dtype=np.float64)
    smoothed[:, :3] = out_positions
    smoothed[:, 5] = np.sin(yaws / 2.0)
    smoothed[:, 6] = np.cos(yaws / 2.0)
    return smoothed


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
