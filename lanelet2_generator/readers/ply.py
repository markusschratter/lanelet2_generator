"""Read poses from PLY (x, y, z, q_w, q_x, q_y, q_z)."""

import numpy as np
from plyfile import PlyData


def read_ply(path):
    """
    Load poses from PLY with vertex properties: x, y, z, q_w, q_x, q_y, q_z.

    Args:
        path: Path to PLY file.

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]

    Raises:
        ValueError: If PLY is missing required vertex properties.
    """
    ply = PlyData.read(path)
    if "vertex" not in ply:
        raise ValueError("PLY file must contain a 'vertex' element")

    v = ply["vertex"]
    required = ("x", "y", "z", "q_w", "q_x", "q_y", "q_z")
    missing = [p for p in required if p not in v.data.dtype.names]
    if missing:
        raise ValueError(
            f"PLY vertex element missing properties: {', '.join(missing)}. "
            f"Required: {', '.join(required)}"
        )

    x = np.asarray(v["x"], dtype=np.float64)
    y = np.asarray(v["y"], dtype=np.float64)
    z = np.asarray(v["z"], dtype=np.float64)
    qw = np.asarray(v["q_w"], dtype=np.float64)
    qx = np.asarray(v["q_x"], dtype=np.float64)
    qy = np.asarray(v["q_y"], dtype=np.float64)
    qz = np.asarray(v["q_z"], dtype=np.float64)
    return np.column_stack([x, y, z, qx, qy, qz, qw])
