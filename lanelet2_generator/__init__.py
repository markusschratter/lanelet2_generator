"""
Lanelet2 generator: create lanelet2 maps from path data.
Supports CSV, PLY, YAML waypoints, MCAP bag, sqlite3 rosbag2, and ROS route points.
"""

from pathlib import Path

import yaml

from lanelet2_generator.readers import load_path, read_csv, read_ply, read_offset, read_yaml
from lanelet2_generator.filtering import filter_path, filter_by_min_distance, filter_downsample
from lanelet2_generator.geometry import pose2line, split_segments
from lanelet2_generator.lanelet import to_lanelet, LaneletMap

__all__ = [
    "load_path",
    "read_bag",
    "read_csv",
    "read_ply",
    "read_offset",
    "read_yaml",
    "filter_path",
    "filter_by_min_distance",
    "filter_downsample",
    "pose2line",
    "split_segments",
    "to_lanelet",
    "LaneletMap",
    "generate",
]


def __getattr__(name):
    if name == "read_bag":
        from lanelet2_generator.readers.bag import read_bag
        return read_bag
    raise AttributeError(f"module 'lanelet2_generator' has no attribute {name}")


def generate(
    input_path=None,
    output_dir=None,
    *,
    poses=None,
    width=2.0,
    mgrs="33TWN",
    map_projector_info=None,
    offset=(0.0, 0.0, 0.0),
    geo_origin=None,
    use_centerline=False,
    min_distance=None,
    step=1,
    interval=(0.1, 2.0),
    split_distance=500,
    max_direction_change_deg=None,
    direction_change_window_m=None,
    speed_limit=30,
    bidirectional=True,
):
    """
    Generate lanelet2 map from input path or pose array.

    Args:
        input_path: Path to CSV, PLY, YAML, MCAP, or rosbag2 directory (ignored if poses given)
        output_dir: Output directory for .osm file
        poses: Optional (N,7) pose array [x,y,z,qx,qy,qz,qw]; overrides input_path
        width: Lane width [m]
        mgrs: MGRS code
        map_projector_info: Optional path to map_projector_info.yaml. If set and
            it contains mgrs_grid, this value overrides mgrs.
        offset: Offset from centerline (x,y,z)
        geo_origin: UTM origin (easting, northing, elevation) of the input local
            frame.  Auto-detected from a companion .offset file for PLY inputs
            when not provided.
        use_centerline: Add centerline to lanelets
        min_distance: Min distance between points [m]
        step: Downsample step
        interval: (min, max) pose interval for bag (bag only)
        split_distance: Split lanelet every M meters
        max_direction_change_deg: Split on direction change (deg)
        direction_change_window_m: Window for direction change [m]
        speed_limit: Speed limit [km/h]
        bidirectional: Generate opposite-direction lanelets too

    Returns:
        Path to saved .osm file
    """
    if output_dir is None:
        raise ValueError("output_dir is required")

    if map_projector_info is not None:
        map_projector_path = Path(map_projector_info)
        if not map_projector_path.exists() and input_path is not None:
            # Docker wrapper mounts input directory as /input; if the caller
            # passed a host-relative path, also try resolving by filename next
            # to the provided input path.
            map_projector_alt = Path(input_path).parent / map_projector_path.name
            if map_projector_alt.exists():
                map_projector_path = map_projector_alt
        if not map_projector_path.exists():
            raise FileNotFoundError(f"map_projector_info not found: {map_projector_path}")
        with open(map_projector_path, "r", encoding="utf-8") as f:
            proj = yaml.safe_load(f) or {}
        if not isinstance(proj, dict):
            raise ValueError(f"Invalid map_projector_info format: {map_projector_path}")
        mgrs_grid = proj.get("mgrs_grid")
        if mgrs_grid is None:
            raise ValueError(f"'mgrs_grid' not found in {map_projector_path}")
        mgrs = str(mgrs_grid).strip()
        if not mgrs:
            raise ValueError(f"Empty 'mgrs_grid' in {map_projector_path}")
        print(f"Using MGRS from {map_projector_path}: {mgrs}")

    if poses is None:
        if input_path is None:
            raise ValueError("Either input_path or poses must be provided")
        poses = load_path(Path(input_path), interval=interval)

    if geo_origin is None and input_path is not None:
        p = Path(input_path)
        offset_path = p.with_suffix(".offset")
        if p.suffix.lower() == ".ply" and offset_path.exists():
            geo_origin = read_offset(offset_path)
            print(f"Using geo origin from {offset_path}: E={geo_origin[0]:.1f} N={geo_origin[1]:.1f} Z={geo_origin[2]:.1f}")

    poses = filter_path(poses, min_distance=min_distance, step=step)

    return to_lanelet(
        poses,
        output_dir,
        width=width,
        mgrs=mgrs,
        offset=offset,
        geo_origin=geo_origin,
        use_centerline=use_centerline,
        split_distance=split_distance,
        max_direction_change_deg=max_direction_change_deg,
        direction_change_window_m=direction_change_window_m,
        speed_limit=speed_limit,
        bidirectional=bidirectional,
    )
