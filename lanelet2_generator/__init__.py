"""
Lanelet2 generator: create lanelet2 maps from path data.
Supports CSV, PLY, MCAP bag, sqlite3 rosbag2, and ROS route points.
"""

from pathlib import Path

from lanelet2_generator.readers import load_path, read_csv, read_ply
from lanelet2_generator.filtering import filter_path, filter_by_min_distance, filter_downsample
from lanelet2_generator.geometry import pose2line, split_segments
from lanelet2_generator.lanelet import to_lanelet, LaneletMap

__all__ = [
    "load_path",
    "read_bag",
    "read_csv",
    "read_ply",
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
    offset=(0.0, 0.0, 0.0),
    use_centerline=False,
    min_distance=None,
    step=1,
    interval=(0.1, 2.0),
    split_distance=500,
    max_direction_change_deg=None,
    direction_change_window_m=None,
    speed_limit=30,
):
    """
    Generate lanelet2 map from input path or pose array.

    Args:
        input_path: Path to CSV, PLY, MCAP, or rosbag2 directory (ignored if poses given)
        output_dir: Output directory for .osm file
        poses: Optional (N,7) pose array [x,y,z,qx,qy,qz,qw]; overrides input_path
        width: Lane width [m]
        mgrs: MGRS code
        offset: Offset from centerline (x,y,z)
        use_centerline: Add centerline to lanelets
        min_distance: Min distance between points [m]
        step: Downsample step
        interval: (min, max) pose interval for bag (bag only)
        split_distance: Split lanelet every M meters
        max_direction_change_deg: Split on direction change (deg)
        direction_change_window_m: Window for direction change [m]
        speed_limit: Speed limit [km/h]

    Returns:
        Path to saved .osm file
    """
    if output_dir is None:
        raise ValueError("output_dir is required")

    if poses is None:
        if input_path is None:
            raise ValueError("Either input_path or poses must be provided")
        poses = load_path(Path(input_path), interval=interval)

    poses = filter_path(poses, min_distance=min_distance, step=step)

    return to_lanelet(
        poses,
        output_dir,
        width=width,
        mgrs=mgrs,
        offset=offset,
        use_centerline=use_centerline,
        split_distance=split_distance,
        max_direction_change_deg=max_direction_change_deg,
        direction_change_window_m=direction_change_window_m,
        speed_limit=speed_limit,
    )
