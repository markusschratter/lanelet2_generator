"""
Lanelet2 generator: create lanelet2 maps from path data.
Supports CSV, PCD, PLY, YAML waypoints, MCAP bag, sqlite3 rosbag2, and ROS route points.
"""

from pathlib import Path

import yaml

from lanelet2_generator.readers import load_path, read_csv, read_pcd, read_ply, read_offset, read_yaml
from lanelet2_generator.filtering import filter_path, filter_by_min_distance, filter_downsample
from lanelet2_generator.geometry import pose2line, smooth_path, split_segments
from lanelet2_generator.lanelet import to_lanelet, LaneletMap
from lanelet2_generator.osm_merge import (
    apply_id_offset,
    compute_auto_offsets,
    compute_step_offsets,
    merge_lanelet_osm_files,
)

__all__ = [
    "load_path",
    "read_bag",
    "read_csv",
    "read_pcd",
    "read_ply",
    "read_offset",
    "read_yaml",
    "filter_path",
    "filter_by_min_distance",
    "filter_downsample",
    "pose2line",
    "smooth_path",
    "split_segments",
    "to_lanelet",
    "LaneletMap",
    "generate",
    "merge_lanelet_osm_files",
    "compute_auto_offsets",
    "compute_step_offsets",
    "apply_id_offset",
]


def __getattr__(name):
    if name == "read_bag":
        from lanelet2_generator.readers.bag import read_bag
        return read_bag
    raise AttributeError(f"module 'lanelet2_generator' has no attribute {name}")


def _parse_map_origin_dict(origin):
    """Parse map_origin-like mapping to (lat, lon, alt). Returns None if invalid."""
    if not isinstance(origin, dict):
        return None
    try:
        lat = float(origin.get("latitude", origin.get("lat")))
        lon = float(origin.get("longitude", origin.get("lon")))
    except (TypeError, ValueError):
        return None
    try:
        alt = float(
            origin.get(
                "altitude",
                origin.get("elevation", origin.get("ele", origin.get("alt", 0.0))),
            )
        )
    except (TypeError, ValueError):
        alt = 0.0
    return (lat, lon, alt)


def _map_origin_from_projector_yaml(proj):
    mo = proj.get("map_origin")
    return _parse_map_origin_dict(mo)


def _map_origin_from_map_config(path):
    """Read /** ros__parameters map_origin from Autoware-style map_config.yaml."""
    path = Path(path)
    if not path.is_file():
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        return None
    if not isinstance(cfg, dict):
        return None
    block = cfg.get("/**")
    if not isinstance(block, dict):
        return None
    params = block.get("ros__parameters")
    if not isinstance(params, dict):
        return None
    return _parse_map_origin_dict(params.get("map_origin"))


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
    min_distance=1.0,
    step=1,
    interval=(0.1, 2.0),
    split_distance=500,
    max_direction_change_deg=None,
    direction_change_window_m=None,
    speed_limit=30,
    bidirectional=True,
    smooth_window=0,
    output_file=None,
):
    """
    Generate lanelet2 map from input path or pose array.

    Args:
        input_path: Path to CSV, PCD, PLY, YAML, MCAP, or rosbag2 directory (ignored if poses given)
        output_dir: Output directory for .osm file
        poses: Optional (N,7) pose array [x,y,z,qx,qy,qz,qw]; overrides input_path
        width: Lane width [m]
        mgrs: MGRS code
        map_projector_info: Optional path to map_projector_info.yaml. If set,
            ``projector_type`` and related fields are read: ``MGRS`` uses
            ``mgrs_grid``; ``local`` exports nodes with empty ``lat``/``lon`` and
            only ``local_x`` / ``local_y`` / ``ele`` tags (optional ``map_origin``
            in YAML is kept for metadata / future use).
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
        smooth_window: Interpolating smoothing subdivisions per segment;
            0 disables
        output_file: Optional exact output .osm filename (absolute or relative
            to output_dir)

    Returns:
        Path to saved .osm file
    """
    if output_dir is None:
        raise ValueError("output_dir is required")

    projector_type = "mgrs"
    map_origin_latlon_alt = (0.0, 0.0, 0.0)

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

        pt_raw = proj.get("projector_type")
        if pt_raw is None:
            projector_type = "mgrs"
        else:
            projector_type = str(pt_raw).strip().lower()

        if projector_type == "local":
            mo = _map_origin_from_projector_yaml(proj)
            if mo is None:
                mo = _map_origin_from_map_config(
                    map_projector_path.parent / "map_config.yaml"
                )
            if mo is None:
                mo = (0.0, 0.0, 0.0)
                print(
                    "map_projector_info: projector_type=local - OSM nodes use "
                    "local_x/local_y/ele only (lat/lon empty); optional map_origin not set."
                )
            else:
                print(
                    f"map_projector_info: projector_type=local - map_origin {mo} "
                    "(metadata; node geometry uses tags only)."
                )
            map_origin_latlon_alt = mo
            mgrs = "33TWN"
        else:
            mgrs_grid = proj.get("mgrs_grid")
            if mgrs_grid is None:
                raise ValueError(
                    f"'mgrs_grid' not found in {map_projector_path} "
                    f"(projector_type={projector_type!r})"
                )
            mgrs = str(mgrs_grid).strip()
            if not mgrs:
                raise ValueError(f"Empty 'mgrs_grid' in {map_projector_path}")
            print(f"Using MGRS from {map_projector_path}: {mgrs}")

    if poses is None:
        if input_path is None:
            raise ValueError("Either input_path or poses must be provided")
        poses = load_path(Path(input_path), interval=interval)

    if projector_type != "local" and geo_origin is None and input_path is not None:
        p = Path(input_path)
        offset_path = p.with_suffix(".offset")
        if p.suffix.lower() == ".ply" and offset_path.exists():
            geo_origin = read_offset(offset_path)
            print(f"Using geo origin from {offset_path}: E={geo_origin[0]:.1f} N={geo_origin[1]:.1f} Z={geo_origin[2]:.1f}")

    poses = filter_path(poses, min_distance=min_distance, step=step)
    poses = smooth_path(poses, window=smooth_window)

    return to_lanelet(
        poses,
        output_dir,
        width=width,
        mgrs=mgrs,
        projector_type=projector_type,
        map_origin_latlon_alt=map_origin_latlon_alt,
        offset=offset,
        geo_origin=geo_origin,
        use_centerline=use_centerline,
        split_distance=split_distance,
        max_direction_change_deg=max_direction_change_deg,
        direction_change_window_m=direction_change_window_m,
        speed_limit=speed_limit,
        bidirectional=bidirectional,
        output_file=output_file,
    )
