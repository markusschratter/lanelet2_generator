"""Path filtering: reduce number of points by distance or downsampling."""

import numpy as np


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


def filter_path(poses, *, min_distance=None, step=1):
    """
    Apply filtering options in order: downsample, then min_distance.

    Args:
        poses: (N, 7) pose array
        min_distance: Keep poses at least min_distance [m] apart
        step: Downsample step (keep every Nth point)

    Returns:
        Reduced pose array
    """
    if step is not None and step > 1:
        poses = filter_downsample(poses, step)
    if min_distance is not None and min_distance > 0:
        poses = filter_by_min_distance(poses, min_distance)
    return poses
