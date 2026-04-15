"""Read poses from YAML mission files (waypoints with position and heading)."""

from pathlib import Path

import numpy as np
import yaml


def read_yaml(path):
    """
    Load poses from YAML with a top-level ``waypoints`` list.

    Each waypoint expects ``position: {x, y, z}`` and optional
    ``orientation: {heading_degree}``. Heading is converted to radians and
    mapped to the same quaternion convention as :func:`read_csv`.

    Args:
        path: Path to ``.yaml`` / ``.yml`` file.

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]

    Raises:
        ValueError: If structure is invalid or waypoints are empty.
    """
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping, got {type(data).__name__}")

    waypoints = data.get("waypoints")
    if waypoints is None:
        raise ValueError("YAML must contain a top-level 'waypoints' key")
    if not isinstance(waypoints, list):
        raise ValueError(f"'waypoints' must be a list, got {type(waypoints).__name__}")
    if len(waypoints) == 0:
        raise ValueError("'waypoints' must be non-empty")

    xs, ys, zs, yaws = [], [], [], []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            raise ValueError(f"waypoints[{i}] must be a mapping, got {type(wp).__name__}")
        pos = wp.get("position")
        if not isinstance(pos, dict):
            raise ValueError(f"waypoints[{i}] must have a 'position' mapping")
        try:
            x = float(pos["x"])
            y = float(pos["y"])
            z = float(pos["z"])
        except KeyError as e:
            raise ValueError(f"waypoints[{i}].position missing key: {e}") from e
        except (TypeError, ValueError) as e:
            raise ValueError(f"waypoints[{i}].position x/y/z must be numeric") from e

        heading_deg = 0.0
        ori = wp.get("orientation")
        if ori is not None:
            if not isinstance(ori, dict):
                raise ValueError(f"waypoints[{i}].orientation must be a mapping or omitted")
            if "heading_degree" in ori:
                try:
                    heading_deg = float(ori["heading_degree"])
                except (TypeError, ValueError) as e:
                    raise ValueError(
                        f"waypoints[{i}].orientation.heading_degree must be numeric"
                    ) from e

        xs.append(x)
        ys.append(y)
        zs.append(z)
        yaws.append(np.deg2rad(heading_deg))

    x = np.asarray(xs, dtype=np.float64)
    y = np.asarray(ys, dtype=np.float64)
    z = np.asarray(zs, dtype=np.float64)
    yaw = np.asarray(yaws, dtype=np.float64)
    qx = np.zeros(len(yaw))
    qy = np.zeros(len(yaw))
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return np.column_stack([x, y, z, qx, qy, qz, qw])
