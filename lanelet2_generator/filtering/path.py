"""Path filtering: reduce number of points by distance or downsampling."""

import numpy as np


def filter_backward_segments(
    poses,
    *,
    lookback=8,
    dot_threshold=-0.05,
    min_segment_m=0.1,
):
    """
    Drop points that belong to short reverse-driving runs along the path.

    The first and last pose are always kept.
    """
    poses = np.asarray(poses, dtype=np.float64)
    if lookback is None or lookback < 1:
        lookback = 1
    n = len(poses)
    if n < 3:
        return poses, 0

    ref_dir = None
    init_end = min(n - 1, int(lookback))
    init = poses[init_end, :2] - poses[0, :2]
    init_len = float(np.linalg.norm(init))
    if init_len >= min_segment_m:
        ref_dir = init / init_len

    backward_edge = np.zeros(n - 1, dtype=bool)
    for i in range(n - 1):
        seg = poses[i + 1, :2] - poses[i, :2]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < min_segment_m:
            continue
        seg_u = seg / seg_len

        if ref_dir is None:
            ref_dir = seg_u.copy()
            continue

        if float(np.dot(seg_u, ref_dir)) < dot_threshold:
            backward_edge[i] = True
            continue

        ref_dir = 0.85 * ref_dir + 0.15 * seg_u
        norm = float(np.linalg.norm(ref_dir))
        if norm > 1e-9:
            ref_dir /= norm

    keep = np.ones(n, dtype=bool)
    i = 0
    while i < n - 1:
        if not backward_edge[i]:
            i += 1
            continue
        j = i
        while j < n - 1 and backward_edge[j]:
            j += 1
        for k in range(i + 1, min(j + 1, n)):
            keep[k] = False
        i = j

    keep[0] = True
    keep[-1] = True

    removed = int(n - int(np.count_nonzero(keep)))
    if removed == 0:
        return poses, 0

    filtered = poses[keep]
    if len(filtered) < 2:
        return poses[[0, -1]].copy(), max(0, n - 2)
    return filtered, removed


def filter_stationary_clusters(
    poses,
    *,
    radius_m=0.3,
    min_cluster_points=3,
):
    """
    Collapse dense GPS jitter while the vehicle barely moves.

    When ``min_cluster_points`` or more consecutive poses stay within
    ``radius_m`` of the cluster start, keep only the first and last pose.
    """
    poses = np.asarray(poses, dtype=np.float64)
    n = len(poses)
    if n < 3 or radius_m is None or radius_m <= 0:
        return poses, 0

    keep_idx = []
    removed = 0
    i = 0
    while i < n:
        anchor = poses[i, :2]
        j = i + 1
        while j < n and float(np.linalg.norm(poses[j, :2] - anchor)) <= radius_m:
            j += 1

        cluster_len = j - i
        if cluster_len >= max(2, int(min_cluster_points)):
            keep_idx.append(i)
            if j - 1 != i:
                keep_idx.append(j - 1)
            removed += cluster_len - (2 if j - 1 != i else 1)
            i = j
            continue

        keep_idx.append(i)
        i += 1

    if not keep_idx:
        return poses, 0

    keep_idx = sorted(set(keep_idx))
    if keep_idx[0] != 0:
        keep_idx = [0] + keep_idx
    if keep_idx[-1] != n - 1:
        keep_idx.append(n - 1)

    filtered = poses[keep_idx]
    return filtered, n - len(filtered)


def filter_micro_loops(
    poses,
    *,
    max_leg_m=1.5,
    dot_threshold=0.0,
):
    """Remove interior points that form tiny hairpin folds (short opposing legs)."""
    poses = np.asarray(poses, dtype=np.float64)
    if len(poses) < 3:
        return poses, 0

    working = [poses[i].copy() for i in range(len(poses))]
    removed = 0
    changed = True
    while changed and len(working) >= 3:
        changed = False
        next_poses = [working[0]]
        i = 1
        while i < len(working) - 1:
            p0 = next_poses[-1][:2]
            p1 = working[i][:2]
            p2 = working[i + 1][:2]
            v1 = p1 - p0
            v2 = p2 - p1
            n1 = float(np.linalg.norm(v1))
            n2 = float(np.linalg.norm(v2))
            if (
                n1 > 1e-6
                and n2 > 1e-6
                and n1 <= max_leg_m
                and n2 <= max_leg_m
                and float(np.dot(v1, v2) / (n1 * n2)) < dot_threshold
            ):
                removed += 1
                changed = True
                i += 1
                continue
            next_poses.append(working[i])
            i += 1
        next_poses.append(working[-1])
        working = next_poses

    return np.asarray(working, dtype=np.float64), removed


def filter_by_min_distance(poses, min_distance):
    """
    Keep only poses at least min_distance [m] apart.
    Always preserves the first and last point.

    Args:
        poses: (N, 7) array [x,y,z,qx,qy,qz,qw]
        min_distance: Minimum Euclidean distance between consecutive points [m]

    Returns:
        Filtered pose array
    """
    if min_distance is None or min_distance <= 0 or len(poses) == 0:
        return poses
    if len(poses) == 1:
        return poses.copy()

    result = [poses[0]]
    for i in range(1, len(poses) - 1):
        d = np.linalg.norm(poses[i, :3] - result[-1][:3])
        if d >= min_distance:
            result.append(poses[i])

    result.append(poses[-1])
    return np.array(result)


def filter_downsample(poses, step):
    """
    Keep every Nth point. Always preserves the last point.

    Args:
        poses: (N, 7) pose array
        step: Keep poses[::step]

    Returns:
        Downsampled pose array
    """
    if step is None or step <= 1:
        return poses
    sampled = poses[::step]
    if len(poses) > 1 and (len(poses) - 1) % step != 0:
        sampled = np.vstack([sampled, poses[-1:]])
    return sampled


def filter_path(
    poses,
    *,
    min_distance=None,
    step=1,
    remove_backward=True,
    backward_lookback=8,
    backward_dot_threshold=-0.05,
    backward_min_segment_m=0.1,
    collapse_stationary=True,
    stationary_radius_m=0.3,
    stationary_min_cluster_points=3,
    remove_micro_loops=True,
    micro_loop_max_leg_m=1.5,
    micro_loop_dot_threshold=0.0,
):
    """
    Apply filtering in order: stationary jitter, micro-loops, backward segments,
    downsample, then min_distance.
    """
    stats = {
        "stationary_removed": 0,
        "micro_loop_removed": 0,
        "backward_removed": 0,
    }

    if collapse_stationary:
        poses, stats["stationary_removed"] = filter_stationary_clusters(
            poses,
            radius_m=stationary_radius_m,
            min_cluster_points=stationary_min_cluster_points,
        )
    if remove_micro_loops:
        poses, stats["micro_loop_removed"] = filter_micro_loops(
            poses,
            max_leg_m=micro_loop_max_leg_m,
            dot_threshold=micro_loop_dot_threshold,
        )
    if remove_backward:
        poses, stats["backward_removed"] = filter_backward_segments(
            poses,
            lookback=backward_lookback,
            dot_threshold=backward_dot_threshold,
            min_segment_m=backward_min_segment_m,
        )
    if step is not None and step > 1:
        poses = filter_downsample(poses, step)
    if min_distance is not None and min_distance > 0:
        poses = filter_by_min_distance(poses, min_distance)
    return poses, stats
