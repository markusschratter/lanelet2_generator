"""
CLI: UTM LAS/LAZ to local MGRS point cloud (PCD) + map_projector_info.yaml.

Requires optional dependencies: pip install 'lanelet2_generator[las]'
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from pyproj import CRS, Transformer

from lanelet2_generator.mgrs_utils import mgrs_grid_origin_utm, mgrs_to_wgs


def _try_import_las_stack():
    try:
        import laspy  # noqa: F401
        import mgrs  # noqa: F401
        import open3d  # noqa: F401
        import matplotlib  # noqa: F401
    except ImportError as e:
        print(
            "Missing LAS tooling dependencies. Install with:\n"
            "  pip install 'lanelet2_generator[las]'\n"
            "or: pip install laspy mgrs open3d matplotlib",
            file=sys.stderr,
        )
        raise SystemExit(1) from e


def _get_matplotlib_cmap(name: str):
    import matplotlib

    try:
        return matplotlib.colormaps.get_cmap(name)
    except AttributeError:
        from matplotlib import cm

        return cm.get_cmap(name)


def _las_dimension_names(las):
    return frozenset(las.point_format.dimension_names)


def _parse_utm_frame(utm_frame):
    """
    Parse UTM frame like '32N' or '32S' into (zone, south_flag).
    """
    if utm_frame is None:
        return None, None
    text = str(utm_frame).strip().upper()
    if len(text) < 2:
        raise ValueError(f"Invalid --utm-frame '{utm_frame}'. Expected format like 32N or 32S.")
    hemi = text[-1]
    zone_txt = text[:-1]
    if hemi not in ("N", "S"):
        raise ValueError(f"Invalid --utm-frame '{utm_frame}'. Last character must be N or S.")
    try:
        zone = int(zone_txt)
    except ValueError as e:
        raise ValueError(f"Invalid --utm-frame '{utm_frame}'. Zone must be an integer.") from e
    if zone < 1 or zone > 60:
        raise ValueError(f"Invalid --utm-frame '{utm_frame}'. Zone must be in 1..60.")
    return zone, hemi == "S"


def _resolve_crs(las, epsg, utm_zone, south, utm_frame):
    if epsg is not None:
        return CRS.from_epsg(int(epsg))
    if utm_frame is not None:
        z, is_south = _parse_utm_frame(utm_frame)
        code = (32700 if is_south else 32600) + z
        return CRS.from_epsg(code)
    if utm_zone is not None:
        z = int(utm_zone)
        code = (32700 if south else 32600) + z
        return CRS.from_epsg(code)
    try:
        crs = las.header.parse_crs()
    except Exception:
        crs = None
    if crs is not None:
        if isinstance(crs, str):
            return CRS.from_wkt(crs)
        return CRS.from_user_input(crs)
    raise ValueError(
        "No CRS in LAS header. Set --epsg or --utm-zone (optionally --south)."
    )


def _auto_color_mode(las, requested):
    if requested != "auto":
        return requested
    names = _las_dimension_names(las)
    if {"red", "green", "blue"}.issubset(names):
        return "rgb"
    if "intensity" in names:
        return "intensity"
    if "classification" in names:
        return "classification"
    return "none"


def _color_values(las, color_by, p_lo, p_hi, cmap_name):
    names = _las_dimension_names(las)
    if color_by == "none":
        return None
    if color_by == "rgb":
        if not {"red", "green", "blue"}.issubset(names):
            raise ValueError(
                "LAS has no RGB dimensions. Required: red, green, blue."
            )
        r = np.asarray(las.red, dtype=np.float64)
        g = np.asarray(las.green, dtype=np.float64)
        b = np.asarray(las.blue, dtype=np.float64)
        rgb = np.column_stack([r, g, b])
        vmax = float(np.max(rgb)) if rgb.size else 0.0
        scale = 65535.0 if vmax > 255.0 else 255.0
        return np.clip(rgb / scale, 0.0, 1.0)
    if color_by not in names:
        raise ValueError(
            f"LAS has no dimension '{color_by}'. Available: {sorted(names)}"
        )
    v = np.asarray(getattr(las, color_by), dtype=np.float64)
    if v.size == 0:
        return None
    lo, hi = np.percentile(v, [p_lo, p_hi])
    if hi <= lo:
        hi = lo + 1e-9
    t = (v - lo) / (hi - lo)
    t = np.clip(t, 0.0, 1.0)
    cmap = _get_matplotlib_cmap(cmap_name)
    rgba = cmap(t)
    return np.asarray(rgba[:, :3], dtype=np.float64)


def _detect_mgrs_grid(easting, northing, crs, m_impl):
    """Centroid UTM -> lat,lon -> MGRS string -> first 5 chars."""
    wgs84 = CRS.from_epsg(4326)
    transformer = Transformer.from_crs(crs, wgs84, always_xy=True)
    lon, lat = transformer.transform(
        float(np.mean(easting)),
        float(np.mean(northing)),
    )
    full = m_impl.toMGRS(float(lat), float(lon))
    return full[:5]


def main():
    parser = argparse.ArgumentParser(
        description="Convert UTM LAS/LAZ to local MGRS frame, optional downsample, write PCD + map_projector_info.yaml."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=None,
        help="Path to .las / .laz (or use --input)",
    )
    parser.add_argument(
        "output_dir",
        nargs="?",
        type=Path,
        default=None,
        help="Output directory, or path ending in .pcd (or use --output)",
    )
    parser.add_argument(
        "--input",
        dest="input_opt",
        type=Path,
        default=None,
        metavar="PATH",
        help="Input .las / .laz (alternative to positional)",
    )
    parser.add_argument(
        "--output",
        dest="output_opt",
        type=Path,
        default=None,
        metavar="PATH",
        help="Output directory or full path to .pcd file (alternative to positional)",
    )
    parser.add_argument("--epsg", type=int, default=None, help="Override CRS: EPSG code")
    parser.add_argument(
        "--utm-frame",
        default=None,
        help="Override CRS using UTM frame notation (e.g. 32N or 32S)",
    )
    parser.add_argument("--utm-zone", dest="utm_zone", type=int, default=None, metavar="Z", help="UTM zone 1..60")
    parser.add_argument("--south", action="store_true", help="Southern hemisphere (EPSG 327nn with --utm-zone)")
    parser.add_argument("--swap-xy", action="store_true", help="Swap X/Y before treating as easting/northing")
    parser.add_argument("--mgrs-grid", default=None, help="Force 5-char MGRS grid instead of auto from centroid")

    parser.add_argument(
        "--color-by",
        choices=["auto", "none", "rgb", "intensity", "classification"],
        default="auto",
        help="Color PCD from LAS data (default: auto pick rgb, then intensity, then classification)",
    )
    parser.add_argument("--colormap", default="viridis", help="matplotlib colormap name")
    parser.add_argument(
        "--color-percentiles",
        type=float,
        nargs=2,
        default=[2.0, 98.0],
        metavar=("PLOW", "PHIGH"),
        help="Percentiles for intensity normalization (outlier clamp)",
    )

    parser.add_argument("--voxel-size", type=float, default=None, metavar="M", help="Voxel downsample size [m]")
    parser.add_argument("--stride", type=int, default=None, help="Keep every k-th point (no Open3D voxel)")
    parser.add_argument("--max-points", type=int, default=None, help="Random subsample to at most N points")
    parser.add_argument("--random-seed", type=int, default=42, help="Seed for --max-points")

    parser.add_argument(
        "--pcd-name",
        default="pointcloud_map.pcd",
        help="Output PCD filename when output is a directory (ignored if output path ends in .pcd)",
    )
    parser.add_argument("--yaml-name", default="map_projector_info.yaml", help="Output projector YAML filename")
    parser.add_argument("--map-config-name", default="map_config.yaml", help="Output map config YAML filename")
    parser.add_argument("--ascii-pcd", action="store_true", help="Write ASCII PCD instead of binary")

    args = parser.parse_args()

    _try_import_las_stack()

    import laspy
    import mgrs
    import open3d as o3d

    inp = args.input_opt if args.input_opt is not None else args.input
    raw_out = args.output_opt if args.output_opt is not None else args.output_dir
    if inp is None:
        parser.error("Provide input as first argument or --input PATH")
    inp = Path(inp)

    if raw_out is not None:
        raw_out = Path(raw_out)
        if raw_out.suffix.lower() == ".pcd":
            output_dir = raw_out.parent
            pcd_name = raw_out.name
        else:
            output_dir = raw_out
            pcd_name = args.pcd_name
    else:
        output_dir = inp.parent
        pcd_name = args.pcd_name

    if not inp.exists():
        raise FileNotFoundError(f"Input not found: {inp}")
    if inp.suffix.lower() not in (".las", ".laz"):
        raise ValueError(
            f"Unsupported input format: {inp} (expected .las or .laz)"
        )

    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        las = laspy.read(str(inp))
    except Exception as e:
        msg = str(e)
        if "No LazBackend selected" in msg and inp.suffix.lower() == ".laz":
            raise RuntimeError(
                "Reading .laz requires a laspy backend. Install one with:\n"
                "  pip install lazrs\n"
                "or install tool extras:\n"
                "  pip install 'lanelet2_generator[las]'"
            ) from e
        raise
    crs = _resolve_crs(las, args.epsg, args.utm_zone, args.south, args.utm_frame)

    x = np.asarray(las.x, dtype=np.float64)
    y = np.asarray(las.y, dtype=np.float64)
    z = np.asarray(las.z, dtype=np.float64)

    if args.swap_xy:
        x, y = y, x

    easting, northing = x, y

    m_impl = mgrs.MGRS()
    if args.mgrs_grid:
        grid5 = str(args.mgrs_grid).strip()[:5]
    else:
        grid5 = _detect_mgrs_grid(easting, northing, crs, m_impl)
    # map_origin should anchor the local frame (x=0,y=0), i.e. the MGRS grid origin.
    lat0, lon0 = mgrs_to_wgs(grid5)
    ele0 = 0.0

    base_e, base_n = mgrs_grid_origin_utm(grid5)
    x_loc = easting - base_e
    y_loc = northing - base_n
    z_loc = z

    pts = np.column_stack([x_loc, y_loc, z_loc])

    color_by = _auto_color_mode(las, args.color_by)
    p_lo, p_hi = args.color_percentiles
    colors = _color_values(las, color_by, p_lo, p_hi, args.colormap)

    n = len(pts)
    if args.stride is not None and args.stride > 1:
        sl = slice(None, None, args.stride)
        pts = pts[sl]
        if colors is not None:
            colors = colors[sl]
        n = len(pts)
    if args.max_points is not None and n > args.max_points:
        rng = np.random.default_rng(args.random_seed)
        idx = rng.choice(n, size=args.max_points, replace=False)
        pts = pts[idx]
        if colors is not None:
            colors = colors[idx]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if colors is not None and len(colors) == len(pts):
        pcd.colors = o3d.utility.Vector3dVector(colors)

    if args.voxel_size is not None and args.voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=float(args.voxel_size))

    pcd_path = output_dir / pcd_name
    o3d.io.write_point_cloud(str(pcd_path), pcd, write_ascii=bool(args.ascii_pcd))

    doc = {
        "projector_type": "MGRS",
        "vertical_datum": "WGS84",
        "mgrs_grid": grid5,
    }
    yaml_path = output_dir / args.yaml_name
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    # Keep map origin stable/readable in YAML (avoid tiny floating-point tails).
    lat0 = round(float(lat0), 10)
    lon0 = round(float(lon0), 11)
    map_cfg = {
        "/**": {
            "ros__parameters": {
                "map_origin": {
                    "latitude": lat0,
                    "longitude": lon0,
                    "elevation": ele0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                }
            }
        }
    }
    map_cfg_path = output_dir / args.map_config_name
    with open(map_cfg_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(map_cfg, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

    print(f"Wrote {pcd_path}")
    print(f"Wrote {yaml_path}")
    print(f"Wrote {map_cfg_path}")
    print(f"mgrs_grid={grid5}  (reference MGRS from centroid: auto)" if not args.mgrs_grid else f"mgrs_grid={grid5}  (forced)")


if __name__ == "__main__":
    main()
