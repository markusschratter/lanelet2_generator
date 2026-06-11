"""
CLI: UTM LAS/LAZ/PCD to local MGRS point cloud (PCD).

Map frame YAML (map_projector_info.yaml) is produced by map_projector /
mcap_map_projector and passed in with --map-projector-info.

Requires optional dependencies: pip install 'lanelet2_generator[las]'
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from pyproj import CRS, Transformer

from lanelet2_generator.mgrs_utils import (
    mgrs_grid_origin_utm,
    mgrs_to_wgs,
    utm_frame_from_mgrs_grid,
)


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
    crs = None
    if las is not None:
        try:
            crs = las.header.parse_crs()
        except Exception:
            crs = None
    if crs is not None:
        if isinstance(crs, str):
            return CRS.from_wkt(crs)
        return CRS.from_user_input(crs)
    raise ValueError(
        "No CRS available from input. Set --epsg, --utm-frame, --utm-zone, "
        "or --mgrs-grid (UTM frame is derived from mgrs_grid, e.g. 33TWN -> 33N)."
    )


def _load_map_projector_yaml(path):
    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        raise ValueError(f"Invalid map_projector_info format: {path}")
    return doc


def _apply_map_projector_info(args):
    """Configure local/georef mode from a pre-generated map_projector_info.yaml."""
    if args.map_projector_info is None:
        return

    path = Path(args.map_projector_info)
    if not path.is_file():
        raise FileNotFoundError(f"map_projector_info not found: {path}")

    doc = _load_map_projector_yaml(path)
    proj_type = str(doc.get("projector_type", "")).strip().lower()
    if proj_type in ("local", ""):
        args.local_frame = True
        print(f"Using local frame from {path}")
        return

    if proj_type != "mgrs":
        raise ValueError(
            f"Unsupported projector_type {doc.get('projector_type')!r} in {path}. "
            "Expected Local or MGRS."
        )

    mgrs_grid = doc.get("mgrs_grid")
    if not mgrs_grid:
        raise ValueError(f"MGRS map_projector_info missing mgrs_grid: {path}")

    args.local_frame = False
    if args.mgrs_grid is None:
        args.mgrs_grid = str(mgrs_grid).strip()[:5]
    if args.utm_frame is None and args.epsg is None and args.utm_zone is None:
        args.utm_frame = utm_frame_from_mgrs_grid(args.mgrs_grid)
    print(
        f"Using map frame from {path}: mgrs_grid={args.mgrs_grid}"
        + (f"  utm_frame={args.utm_frame}" if args.utm_frame else "")
    )


def _has_explicit_georef_cli(args):
    return (
        args.epsg is not None
        or args.utm_frame is not None
        or args.utm_zone is not None
        or args.mgrs_grid is not None
        or args.subtract_xy_from_mgrs
    )


def _resolve_frame_mode(args):
    """
    Default processing mode is local (keep XYZ, no UTM/MGRS conversion).

    Georef mode when --map-projector-info (MGRS), --utm-frame, --epsg, --mgrs-grid, etc.
    """
    if args.local_frame:
        return

    if _has_explicit_georef_cli(args):
        args.local_frame = False
        return

    args.local_frame = True
    print(
        "Using local frame (default; pass --map-projector-info or --utm-frame for georef)"
    )


def _apply_georef_from_mgrs_grid(args):
    """Derive source --utm-frame from --mgrs-grid when georef mode and CRS not set."""
    if args.local_frame:
        return
    if (
        args.mgrs_grid
        and args.utm_frame is None
        and args.epsg is None
        and args.utm_zone is None
    ):
        args.utm_frame = utm_frame_from_mgrs_grid(args.mgrs_grid)
        print(
            f"Using UTM frame {args.utm_frame} from --mgrs-grid "
            f"(source CRS for LAS/UTM input)"
        )


def _apply_local_frame_overrides(args):
    if not args.local_frame:
        return
    ignored = []
    if args.epsg is not None:
        ignored.append("--epsg")
    if args.utm_frame is not None:
        ignored.append("--utm-frame")
    if args.utm_zone is not None:
        ignored.append("--utm-zone")
    if args.south:
        ignored.append("--south")
    if args.mgrs_grid:
        ignored.append("--mgrs-grid")
    if args.subtract_xy_from_mgrs:
        ignored.append("--subtract-xy-from-mgrs")
    if ignored:
        print(
            "local frame: ignoring georeferencing options: " + ", ".join(ignored),
            file=sys.stderr,
        )
    args.epsg = None
    args.utm_frame = None
    args.utm_zone = None
    args.south = False
    args.mgrs_grid = None
    args.subtract_xy_from_mgrs = False


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


_LAS_SUFFIXES = (".las", ".laz")


def _collect_las_inputs(path: Path) -> list[Path]:
    """Return one or more LAS/LAZ paths: a single file, or all files in a directory."""
    path = Path(path)
    if path.is_file():
        if path.suffix.lower() not in _LAS_SUFFIXES:
            raise ValueError(
                f"Unsupported LAS input: {path} (expected .las or .laz)"
            )
        return [path]
    if path.is_dir():
        files = sorted(
            p for p in path.iterdir()
            if p.is_file() and p.suffix.lower() in _LAS_SUFFIXES
        )
        if not files:
            raise ValueError(f"No .las/.laz files found in directory: {path}")
        return files
    raise FileNotFoundError(f"Input not found: {path}")


def _read_las_file(las_path):
    import laspy

    try:
        return laspy.read(str(las_path))
    except Exception as e:
        msg = str(e)
        if "No LazBackend selected" in msg and las_path.suffix.lower() == ".laz":
            raise RuntimeError(
                "Reading .laz requires a laspy backend. Install one with:\n"
                "  pip install lazrs\n"
                "or install tool extras:\n"
                "  pip install 'lanelet2_generator[las]'"
            ) from e
        raise


def _sample_las_coords(las_files, sample_stride=1000):
    """Subsampled source XY from all tiles for frame reference (low memory)."""
    xs, ys = [], []
    for las_path in las_files:
        las = _read_las_file(las_path)
        stride = max(1, int(sample_stride))
        xs.append(np.asarray(las.x, dtype=np.float64)[::stride])
        ys.append(np.asarray(las.y, dtype=np.float64)[::stride])
    return np.concatenate(xs), np.concatenate(ys)


def _resolve_subtract_xy(x, y, args):
    sub_x, sub_y = float(args.subtract_xy[0]), float(args.subtract_xy[1])
    if args.subtract_xy_from_mgrs:
        if not args.mgrs_grid:
            raise ValueError("--subtract-xy-from-mgrs requires --mgrs-grid (e.g. 33TWN)")
        shift_grid = str(args.mgrs_grid).strip()[:5]
        shift_e, shift_n = mgrs_grid_origin_utm(shift_grid)
        sub_x, mode_x = _choose_axis_subtract(x, shift_e)
        sub_y, mode_y = _choose_axis_subtract(y, shift_n)
        print(
            f"Using auto subtract from MGRS grid {shift_grid}: "
            f"X={sub_x:.3f} ({mode_x}) Y={sub_y:.3f} ({mode_y})"
        )
    return sub_x, sub_y


def _resolve_map_frame(x, y, crs, args, m_impl, local_frame):
    """Return grid5, lat0, lon0, ele0, base_e, base_n, sub_x, sub_y."""
    if local_frame:
        return None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    sub_x, sub_y = _resolve_subtract_xy(x, y, args)
    if args.mgrs_grid:
        grid5 = str(args.mgrs_grid).strip()[:5]
    else:
        grid5 = _detect_mgrs_grid(x, y, crs, m_impl)
    lat0, lon0 = mgrs_to_wgs(grid5)
    base_e, base_n = mgrs_grid_origin_utm(grid5)
    return grid5, lat0, lon0, 0.0, base_e, base_n, sub_x, sub_y


def _arrays_to_local_points(x, y, z, swap_xy, sub_x, sub_y, local_frame, base_e, base_n):
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    if swap_xy:
        x, y = y, x
    if sub_x != 0.0 or sub_y != 0.0:
        x = x - float(sub_x)
        y = y - float(sub_y)
    if local_frame:
        return np.column_stack([x, y, z])
    return np.column_stack([x - base_e, y - base_n, z])


def _las_to_local_points(las, swap_xy, sub_x, sub_y, local_frame, base_e, base_n):
    return _arrays_to_local_points(
        las.x, las.y, las.z, swap_xy, sub_x, sub_y, local_frame, base_e, base_n
    )


def _points_to_pcd(o3d, pts, colors, stride, voxel_size):
    if stride is not None and stride > 1:
        pts = pts[::stride]
        if colors is not None:
            colors = colors[::stride]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if colors is not None and len(colors) == len(pts):
        pcd.colors = o3d.utility.Vector3dVector(colors)
    if voxel_size is not None and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))
    return pcd


def _merge_point_clouds(o3d, pcds):
    merged = pcds[0]
    for pcd in pcds[1:]:
        merged += pcd
    return merged


def _subsample_pcd(o3d, pcd, max_points, random_seed):
    n = len(pcd.points)
    if max_points is None or n <= max_points:
        return pcd
    rng = np.random.default_rng(random_seed)
    idx = rng.choice(n, size=max_points, replace=False)
    return pcd.select_by_index(idx)


def _write_pcd(o3d, pcd, path, write_ascii):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = o3d.io.write_point_cloud(str(path), pcd, write_ascii=write_ascii)
    if not ok:
        raise RuntimeError(f"Failed to write point cloud: {path}")
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"Point cloud file is empty after write: {path}")
    return path


def _resolve_tile_pcd_dir(output_dir, tile_pcd_dir):
    """
    Resolve per-tile PCD directory under ``output_dir``.

    ``--tile-pcd-dir`` is a subdirectory name (e.g. ``pcd_tiles``), not a host path.
    If an absolute path is passed, only the final component is used under ``output_dir``
    so Docker volume mounts to ``--output`` still work.
    """
    text = (tile_pcd_dir or "").strip()
    if not text:
        return None
    path = Path(text)
    out = Path(output_dir)
    if path.is_absolute():
        resolved = out / path.name
        print(
            f"tile-pcd-dir: writing tiles to {resolved} "
            f"(subfolder of output; full path {path} ignored in container)"
        )
        return resolved
    return out / path


def _process_las_inputs(
    las_files, args, crs, o3d, mgrs, output_dir=None, write_ascii=False, *, input_is_dir=False
):
    """
    Process LAS/LAZ inputs tile-by-tile: voxel each file, then merge.
    Keeps peak memory low compared to merging full clouds before downsampling.
    """
    p_lo, p_hi = args.color_percentiles
    local_frame = args.local_frame
    multi_tile = len(las_files) > 1

    las_cache = {}
    if multi_tile:
        ref_x, ref_y = _sample_las_coords(las_files)
    else:
        las0 = _read_las_file(las_files[0])
        las_cache[las_files[0]] = las0
        ref_x = np.asarray(las0.x, dtype=np.float64)
        ref_y = np.asarray(las0.y, dtype=np.float64)

    if args.swap_xy:
        ref_x, ref_y = ref_y, ref_x
    m_impl = None if local_frame else mgrs.MGRS()
    grid5, lat0, lon0, ele0, base_e, base_n, sub_x, sub_y = _resolve_map_frame(
        ref_x, ref_y, crs, args, m_impl, local_frame
    )

    tile_dir = _resolve_tile_pcd_dir(output_dir, args.tile_pcd_dir)
    save_tiles = tile_dir is not None and output_dir is not None and (input_is_dir or len(las_files) > 1)
    if save_tiles and (args.voxel_size is None or args.voxel_size <= 0):
        print(
            "WARNING: --voxel-size is strongly recommended when saving per-tile PCDs; "
            "large LAS tiles can exceed available memory without downsampling.",
            file=sys.stderr,
        )

    merged_pcd = None
    for i, las_path in enumerate(las_files):
        las = las_cache.get(las_path) or _read_las_file(las_path)
        n_raw = len(las.points)
        mode = _auto_color_mode(las, args.color_by)
        colors = _color_values(las, mode, p_lo, p_hi, args.colormap)
        pts = _las_to_local_points(
            las, args.swap_xy, sub_x, sub_y, local_frame, base_e, base_n
        )
        pcd = _points_to_pcd(o3d, pts, colors, args.stride, args.voxel_size)
        print(
            f"{'Reading' if i == 0 else '  +'} {las_path.name}: "
            f"{n_raw} -> {len(pcd.points)} points"
        )
        if args.voxel_size and args.voxel_size > 0 and len(pcd.points) >= n_raw:
            print(
                f"    warning: --voxel-size {args.voxel_size} did not reduce {las_path.name}",
                file=sys.stderr,
            )
        if save_tiles:
            tile_path = tile_dir / f"{las_path.stem}.pcd"
            _write_pcd(o3d, pcd, tile_path, write_ascii)
            size_mb = tile_path.stat().st_size / (1024 * 1024)
            print(f"    saved {tile_path} ({len(pcd.points)} points, {size_mb:.1f} MiB)")
        if merged_pcd is None:
            merged_pcd = pcd
        else:
            merged_pcd += pcd
        del las, pts, colors, pcd

    if merged_pcd is None:
        raise RuntimeError("No LAS/LAZ tiles were processed")

    if multi_tile:
        pcd = merged_pcd
        if args.voxel_size is not None and args.voxel_size > 0:
            before = len(pcd.points)
            pcd = pcd.voxel_down_sample(voxel_size=float(args.voxel_size))
            print(
                f"Merged {len(las_files)} tiles: {before} -> {len(pcd.points)} points "
                f"(final voxel {args.voxel_size} m)"
            )
        else:
            print(f"Merged {len(las_files)} tiles -> {len(pcd.points)} points")
    else:
        pcd = merged_pcd

    pcd = _subsample_pcd(o3d, pcd, args.max_points, args.random_seed)
    return pcd, grid5, lat0, lon0, ele0


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


def _choose_axis_subtract(values, origin):
    """
    Pick subtract value (0, +origin, or -origin) that best recenters values near zero.
    """
    med = float(np.median(values))
    zero = 0.0
    pos = float(origin)
    neg = -float(origin)
    # Smaller absolute median after subtraction indicates better local recentering.
    score_zero = abs(med - zero)
    score_pos = abs(med - pos)
    score_neg = abs(med - neg)
    best = min(
        [(score_zero, zero, "none"), (score_pos, pos, "positive-origin"), (score_neg, neg, "negative-origin")],
        key=lambda t: t[0],
    )
    return best[1], best[2]


def main():
    parser = argparse.ArgumentParser(
        description="Convert UTM LAS/LAZ/PCD to local MGRS frame PCD. "
        "Map frame YAML comes from a prior map_projector / mcap_map_projector step."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=None,
        help="Path to .las / .laz / .pcd, or directory of .las/.laz files (or use --input)",
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
        help="Input .las / .laz / .pcd, or directory of .las/.laz files (alternative to positional)",
    )
    parser.add_argument(
        "--output",
        dest="output_opt",
        type=Path,
        default=None,
        metavar="PATH",
        help="Output directory or full path to .pcd file (alternative to positional)",
    )
    parser.add_argument(
        "--map-projector-info",
        type=Path,
        default=None,
        metavar="PATH",
        help="map_projector_info.yaml from map_projector / mcap_map_projector (step 1)",
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
    parser.add_argument(
        "--local-frame",
        action="store_true",
        help="Keep source XYZ (default when no georef is configured). Explicit flag is optional.",
    )
    parser.add_argument(
        "--subtract-xy",
        type=float,
        nargs=2,
        default=[0.0, 0.0],
        metavar=("X", "Y"),
        help="Subtract (x-=A, y-=B) from source coordinates; optional fine-tune in --local-frame mode",
    )
    parser.add_argument(
        "--subtract-xy-from-mgrs",
        action="store_true",
        help="Auto-set --subtract-xy from --mgrs-grid origin (ignored with --local-frame)",
    )
    parser.add_argument(
        "--mgrs-grid",
        default=None,
        help="Override 5-char MGRS grid (default: from --map-projector-info or LAS centroid)",
    )

    parser.add_argument(
        "--color-by",
        choices=["auto", "none", "rgb", "intensity", "classification"],
        default="auto",
        help="Color output PCD from source data (LAS dims or input PCD colors when available)",
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
    parser.add_argument(
        "--tile-pcd-dir",
        default="pcd_tiles",
        help="Subfolder under --output for per-tile PCDs on folder input (name only, e.g. pcd_tiles; '' to disable)",
    )
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
        raw_out = Path(str(raw_out).strip())
        if raw_out.suffix.lower() == ".pcd":
            output_dir = raw_out.parent
            pcd_name = raw_out.name.strip()
        else:
            output_dir = raw_out
            pcd_name = args.pcd_name.strip()
    else:
        output_dir = inp if inp.is_dir() else inp.parent
        pcd_name = args.pcd_name

    if not inp.exists():
        raise FileNotFoundError(f"Input not found: {inp}")

    _apply_map_projector_info(args)
    _resolve_frame_mode(args)
    _apply_local_frame_overrides(args)
    _apply_georef_from_mgrs_grid(args)

    output_dir.mkdir(parents=True, exist_ok=True)
    if args.voxel_size is not None:
        print(f"voxel-size={args.voxel_size} m")
    tile_dir = _resolve_tile_pcd_dir(output_dir, args.tile_pcd_dir)
    if tile_dir is not None:
        print(f"Output directory: {output_dir}")
        print(f"Tile PCD directory: {tile_dir}")

    if inp.is_dir() or inp.suffix.lower() in _LAS_SUFFIXES:
        las_files = _collect_las_inputs(inp)
        crs = None
        if not args.local_frame:
            crs = _resolve_crs(
                _read_las_file(las_files[0]),
                args.epsg,
                args.utm_zone,
                args.south,
                args.utm_frame,
            )
        pcd, grid5, _, _, _ = _process_las_inputs(
            las_files,
            args,
            crs,
            o3d,
            mgrs,
            output_dir=output_dir,
            write_ascii=bool(args.ascii_pcd),
            input_is_dir=inp.is_dir(),
        )
    elif inp.suffix.lower() == ".pcd":
        pcd_in = o3d.io.read_point_cloud(str(inp))
        pts_in = np.asarray(pcd_in.points, dtype=np.float64)
        if pts_in.size == 0:
            raise ValueError(f"PCD has no points: {inp}")
        if pts_in.ndim != 2 or pts_in.shape[1] != 3:
            raise ValueError(f"PCD points must be Nx3, got shape {pts_in.shape} from {inp}")
        crs = None if args.local_frame else _resolve_crs(None, args.epsg, args.utm_zone, args.south, args.utm_frame)
        x = pts_in[:, 0]
        y = pts_in[:, 1]
        z = pts_in[:, 2]
        colors = np.asarray(pcd_in.colors, dtype=np.float64) if pcd_in.has_colors() else None
        ref_x, ref_y = (y, x) if args.swap_xy else (x, y)
        m_impl = None if args.local_frame else mgrs.MGRS()
        grid5, _, _, _, base_e, base_n, sub_x, sub_y = _resolve_map_frame(
            ref_x, ref_y, crs, args, m_impl, args.local_frame
        )
        pts = _arrays_to_local_points(
            x, y, z, args.swap_xy, sub_x, sub_y, args.local_frame, base_e, base_n
        )
        pcd = _points_to_pcd(o3d, pts, colors, args.stride, args.voxel_size)
        pcd = _subsample_pcd(o3d, pcd, args.max_points, args.random_seed)
    else:
        raise ValueError(
            f"Unsupported input: {inp} (expected .las, .laz, .pcd, or directory of .las/.laz)"
        )

    pcd_path = output_dir / pcd_name
    _write_pcd(o3d, pcd, pcd_path, bool(args.ascii_pcd))

    print(f"Wrote {pcd_path}")
    if args.local_frame:
        print("local_frame=true (UTM/MGRS conversion skipped)")
    else:
        source = "map_projector_info" if args.map_projector_info else (
            "forced" if args.mgrs_grid else "LAS centroid"
        )
        print(f"mgrs_grid={grid5}  utm_frame={args.utm_frame}  (source: {source})")


if __name__ == "__main__":
    main()
