"""CLI: generate map_projector_info.yaml and map_config.yaml (local or georef)."""

import argparse
from pathlib import Path

from lanelet2_generator.map_projector import generate_map_projector_files


def main():
    parser = argparse.ArgumentParser(
        description="Generate map_projector_info.yaml and map_config.yaml before "
        "pointcloud conversion or lanelet generation. Same local/georef flags as "
        "the pointcloud converter; default is local."
    )
    parser.add_argument(
        "output_dir",
        nargs="?",
        type=Path,
        default=None,
        help="Output directory for YAML files (or use --output)",
    )
    parser.add_argument(
        "--output",
        dest="output_opt",
        type=Path,
        default=None,
        help="Output directory (alternative to positional)",
    )
    parser.add_argument(
        "--local-frame",
        action="store_true",
        help="Force local map YAML (default when no georef flags are set)",
    )
    parser.add_argument(
        "--utm-frame",
        default=None,
        help="UTM source frame for georef maps (e.g. 32N); use with --mgrs-grid",
    )
    parser.add_argument("--utm-zone", type=int, default=None, metavar="Z", help="UTM zone 1..60")
    parser.add_argument("--south", action="store_true", help="Southern hemisphere (with --utm-zone)")
    parser.add_argument("--epsg", type=int, default=None, help="EPSG code (georef mode)")
    parser.add_argument(
        "--mgrs-grid",
        default=None,
        help="5-char MGRS grid for georef YAML (required for georef; e.g. 32TNT)",
    )
    parser.add_argument(
        "--elevation",
        type=float,
        default=0.0,
        help="map_origin elevation [m] in map_config.yaml (default: 0)",
    )
    parser.add_argument("--yaml-name", default="map_projector_info.yaml")
    parser.add_argument("--map-config-name", default="map_config.yaml")

    args = parser.parse_args()
    output_dir = args.output_opt if args.output_opt is not None else args.output_dir
    if output_dir is None:
        parser.error("Output directory required (positional or --output)")

    yaml_path, map_cfg_path, info = generate_map_projector_files(
        output_dir,
        local_frame=args.local_frame,
        mgrs_grid=args.mgrs_grid,
        utm_frame=args.utm_frame,
        epsg=args.epsg,
        utm_zone=args.utm_zone,
        south=args.south,
        elevation=args.elevation,
        yaml_name=args.yaml_name,
        map_config_name=args.map_config_name,
    )

    print(f"Wrote {yaml_path}")
    print(f"Wrote {map_cfg_path}")
    if info["mode"] == "local":
        print(f"mode=local ({info.get('reason', 'default')})")
    else:
        origin = info["map_origin"]
        print(
            f"mode=mgrs  mgrs_grid={info['mgrs_grid']}  "
            f"map_origin=({origin[0]:.8f}, {origin[1]:.8f}, {origin[2]:.3f})"
            + (f"  utm_frame={info['utm_frame']}" if info.get("utm_frame") else "")
        )


if __name__ == "__main__":
    main()
