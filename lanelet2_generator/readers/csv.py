"""Read poses from CSV (x, y, z, yaw, velocity, change_flag)."""

import numpy as np


def read_csv(path):
    """
    Load poses from CSV with columns: x, y, z, yaw, velocity, change_flag.

    Args:
        path: Path to CSV file.

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]

    Raises:
        ValueError: If CSV has fewer than 4 columns.
    """
    data = np.loadtxt(
        path,
        delimiter=",",
        skiprows=1,
        usecols=(0, 1, 2, 3),
        dtype=np.float64,
    )
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 4:
        raise ValueError(
            f"CSV must have at least 4 columns (x, y, z, yaw), got {data.shape[1]}"
        )

    x, y, z, yaw = data[:, 0], data[:, 1], data[:, 2], data[:, 3]
    qx = np.zeros(len(yaw))
    qy = np.zeros(len(yaw))
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return np.column_stack([x, y, z, qx, qy, qz, qw])
