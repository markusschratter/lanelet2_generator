"""CLI for lanelet2_generator."""

import argparse
from pathlib import Path

from lanelet2_generator import generate


def main():
    parser = argparse.ArgumentParser(
        description="Create lanelet2 map from path data. "
        "Input: CSV (.csv), PLY (.ply), YAML (.yaml/.yml), MCAP bag (.mcap), or rosbag2 directory (sqlite3)",
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=None,
        help="input path: CSV, PLY, YAML, MCAP, or rosbag2 directory (or use --input)",
    )
    parser.add_argument(
        "--input",
        dest="input_path_flag",
        type=Path,
        default=None,
        metavar="PATH",
        help="same as positional input; use for explicit, flag-style invocation",
    )
    parser.add_argument(
        "output_lanelet",
        nargs="?",
        default=None,
        help="output lanelet2 save path (directory) (or use --output)",
    )
    parser.add_argument(
        "--output",
        dest="output_path_flag",
        type=Path,
        default=None,
        metavar="PATH",
        help="same as positional output_lanelet; use for explicit, flag-style invocation",
    )
    parser.add_argument("--width", type=float, default=2.0, help="lane width [m]")
    parser.add_argument("--mgrs", default="33TWN", help="MGRS code")
    parser.add_argument(
        "--map-projector-info",
        type=Path,
        default=None,
        help="optional map_projector_info.yaml; uses mgrs_grid from file",
    )
    parser.add_argument("--offset", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="offset [m] from centerline")
    parser.add_argument("--center", action="store_true", help="add centerline to lanelet")
    parser.add_argument("--min-distance", type=float, default=1.0, metavar="M", help="minimum distance [m] between points (default: 1.0)")
    parser.add_argument("--interval", type=float, nargs=2, default=[0.1, 2.0], metavar=("MIN", "MAX"),
        help="[bag/mcap only] min and max interval between tf poses [m]")
    parser.add_argument("--step", type=int, default=1, help="[CSV/PLY only] downsample step (default: 1)")
    parser.add_argument("--split-distance", type=float, default=500, metavar="M", help="split lanelet every M meters (default: 500)")
    parser.add_argument("--split-direction", type=float, nargs=2, default=None, metavar=("DEG", "M"),
        help="split when direction changes more than DEG deg within M m (e.g. 80 30)")
    parser.add_argument("--speed-limit", type=float, default=30, metavar="KMH", help="speed limit [km/h]")
    parser.add_argument("--output-file", default=None, help="exact output .osm filename (absolute or relative to output dir)")
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=0,
        metavar="N",
        help="interpolating smoothing subdivisions per segment; 0 disables",
    )
    parser.add_argument(
        "--no-bidirectional",
        dest="bidirectional",
        action="store_false",
        default=True,
        help="disable opposite-direction lanelet generation",
    )

    args = parser.parse_args()
    if (
        args.input_path_flag is not None
        and args.output_path_flag is None
        and args.output_lanelet is None
        and args.input is not None
    ):
        # argparse assigns the single remaining positional to "input";
        # reinterpret it as output when input comes from --input.
        args.output_lanelet = args.input
        args.input = None

    if args.input_path_flag is not None:
        input_path = Path(args.input_path_flag)
        if args.input is not None and Path(args.input).resolve() != input_path.resolve():
            parser.error("positional input and --input must refer to the same path (use only one)")
    elif args.input is not None:
        input_path = Path(args.input)
    else:
        parser.error("input path required: pass as first positional argument or use --input")

    if not input_path.exists():
        raise FileNotFoundError(f"Input '{input_path}' not found.")
    if args.output_path_flag is not None:
        output_path = Path(args.output_path_flag)
        if args.output_lanelet is not None and Path(args.output_lanelet).resolve() != output_path.resolve():
            parser.error("positional output_lanelet and --output must refer to the same path (use only one)")
    elif args.output_lanelet is not None:
        output_path = Path(args.output_lanelet)
    else:
        parser.error("output path required: pass as second positional argument or use --output")

    max_deg = args.split_direction[0] if args.split_direction else None
    window_m = args.split_direction[1] if args.split_direction else None

    result = generate(
        input_path=input_path,
        output_dir=output_path,
        width=args.width,
        mgrs=args.mgrs,
        map_projector_info=args.map_projector_info,
        offset=tuple(args.offset),
        use_centerline=args.center,
        min_distance=args.min_distance,
        step=args.step,
        interval=tuple(args.interval),
        split_distance=args.split_distance,
        max_direction_change_deg=max_deg,
        direction_change_window_m=window_m,
        speed_limit=args.speed_limit,
        bidirectional=args.bidirectional,
        smooth_window=args.smooth_window,
        output_file=args.output_file,
    )
    print(f"Saved: {result}")


if __name__ == "__main__":
    main()
