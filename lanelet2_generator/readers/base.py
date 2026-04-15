"""Unified path loading: dispatches to appropriate reader by extension."""

from pathlib import Path

from lanelet2_generator.readers.csv import read_csv
from lanelet2_generator.readers.ply import read_ply
from lanelet2_generator.readers.yaml_waypoints import read_yaml


def load_path(path, **kwargs):
    """
    Load poses from file. Dispatches by extension.

    Args:
        path: Path to CSV, PLY, YAML, MCAP, or rosbag2 directory
        **kwargs: Passed to reader (e.g. interval for bags)

    Returns:
        (N, 7) ndarray [x, y, z, qx, qy, qz, qw]
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Input '{path}' not found.")

    suffix = path.suffix.lower()
    if suffix == ".csv":
        return read_csv(path)
    if suffix == ".ply":
        return read_ply(path)
    if suffix in (".yaml", ".yml"):
        return read_yaml(path)
    if suffix == ".mcap":
        from lanelet2_generator.readers.bag import read_bag
        return read_bag(path, **kwargs)
    if path.is_dir():
        from lanelet2_generator.readers.bag import read_bag
        return read_bag(path, **kwargs)

    raise ValueError(
        f"Unsupported input format: {path} (expect .csv, .ply, .yaml, .yml, .mcap, or directory)"
    )
