"""Read poses from PCD by inferring yaw from point-to-point direction."""

import numpy as np
import open3d as o3d


def read_pcd(path):
    """
    Load poses from PCD point cloud coordinates.

    The PCD format does not include orientation, so yaw is inferred from the
    direction between consecutive points. The final point reuses the previous
    yaw. For single-point clouds yaw defaults to 0.

    Args:
        path: Path to PCD file.

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]

    Raises:
        ValueError: If the point cloud is empty.
    """
    pcd = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pcd.points, dtype=np.float64)
    if pts.size == 0:
        raise ValueError(f"PCD has no points: {path}")
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"PCD points must be Nx3, got shape {pts.shape} from {path}")

    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]

    if len(pts) == 1:
        yaw = np.zeros(1, dtype=np.float64)
    else:
        dx = np.diff(x)
        dy = np.diff(y)
        yaw = np.arctan2(dy, dx)
        yaw = np.concatenate([yaw, yaw[-1:]])

    qx = np.zeros(len(yaw), dtype=np.float64)
    qy = np.zeros(len(yaw), dtype=np.float64)
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return np.column_stack([x, y, z, qx, qy, qz, qw])
